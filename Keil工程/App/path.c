#include "stm32f10x.h"
#include <math.h>
#include "path.h"
#include "app_config.h"
#include "imu.h"
#include "gray_uart.h"

#define PATH_MAX_SEGMENTS 16

static const Segment g_path_q1[] =
{
    { SEG_STRAIGHT, TRACK_AB_MM, 1.00f, 0, 1 },
    { SEG_STOP,     0.0f,        0.00f, 0, 0 }
};

/* Q2: A->B straight, right 180 deg arc, C->D straight, right 180 deg arc.
 *
 *   SEG_LINE     - line following straight: drives straight until the tape
 *                  shows up, then follows it (the start of the run is often
 *                  before the tape).
 *   SEG_ARC_LINE - line following arc, the 400 mm geometry only provides the
 *                  curvature feedforward.  When the arc length is reached the
 *                  car stands still and pivots to exactly 180 deg before the
 *                  next straight starts (see ARC_SETTLE_TOL).
 *   SEG_STRAIGHT - C->D is pure MPU6050 heading hold at the 180 deg handed
 *                  over by the arc, no grayscale.
 * Change a SEG_LINE back to SEG_STRAIGHT / a SEG_ARC_LINE back to SEG_ARC to
 * get the old MPU / dead-reckoned behaviour for that segment. */
static const Segment g_path_q2[] =
{
    { SEG_LINE,     TRACK_AB_MM,          1.00f,  0, 1 }, /* B */
    { SEG_ARC_LINE, TRACK_ARC_MM,         0.75f, -1, 1 }, /* C, right arc, grayscale */
    { SEG_STRAIGHT, TRACK_CD_MM,          1.00f,  0, 1 }, /* D, MPU 180 deg */
    { SEG_ARC_LINE, TRACK_ARC_MM,         0.75f, -1, 1 }, /* A, right arc, grayscale */
    { SEG_STOP,     0.0f,                 0.00f,  0, 0 }
};

static const Segment g_path_q3[] =
{
    { SEG_TURN,     +TRACK_ANGLE_DEG,     0.55f,  0, 0 }, /* A: turn left to AC */
    { SEG_STRAIGHT, TRACK_DIAG_MM - TURN_FORWARD_MM, 1.00f, 0, 1 }, /* C */
    { SEG_TURN,     -TRACK_ANGLE_DEG,     0.55f,  0, 0 }, /* C: face east into arc */
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, -1, 1 }, /* B, right arc */
    { SEG_TURN,     -TRACK_ANGLE_DEG,     0.55f,  0, 0 }, /* B: face BD */
    { SEG_STRAIGHT, TRACK_DIAG_MM - TURN_FORWARD_MM, 1.00f, 0, 1 }, /* D */
    { SEG_TURN,     +TRACK_ANGLE_DEG,     0.55f,  0, 0 }, /* D: face west into arc */
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, +1, 1 }, /* A, left arc */
    { SEG_STOP,     0.0f,                 0.00f,  0, 0 }
};

static PathState g_state = PATH_IDLE;
static uint8_t   g_mode = 0;
static uint8_t   g_seg_index = 0;
static uint8_t   g_lap = 0;
static uint8_t   g_laps_total = 1;
static uint8_t   g_need_begin = 0;

static const Segment *g_path = 0;
static uint8_t g_path_len = 0;

static float g_seg_start_dist = 0.0f;
static float g_target_yaw = 0.0f;
static float g_seg_yaw_start = 0.0f;  /* heading when the segment began */
static float g_arc_yaw_start = 0.0f;  /* heading when the arc began */
static float g_turn_target_yaw = 0.0f;
static float g_last_line_err = 0.0f;
static float g_gray_corr = 0.0f;      /* deg, heading offset from the grayscale */
static float g_yaw_ref_dbg = 0.0f;    /* last line/arc heading reference */
static float g_nominal_yaw = 0.0f;    /* track heading handed to the next straight */
static float g_arc_radius = 0.0f;     /* measured arc radius, for the log */
static float g_line_theta = 0.0f;     /* deg, line-following heading offset */
static float g_line_rate_dbg = 0.0f;  /* deg/s, last line rate command */
static float g_line_d_f = 0.0f;       /* filtered d(line error)/dt */
static uint8_t g_arc_mode = 0;        /* 1 = grayscale line following active */
static uint8_t g_arc_why = 0;         /* why the last arc ended */
static ArcReport g_arc_rep;
static uint8_t  g_arc_rep_pending = 0;
static int16_t  g_rep_e_min, g_rep_e_max;
static int16_t  g_rep_gc_min, g_rep_gc_max;
static uint16_t g_rep_lost;
static uint8_t  g_rep_g_min, g_rep_g_max;
static uint8_t g_nominal_yaw_valid = 0;
static uint8_t g_line_lost = 0;
static uint16_t g_turn_ms = 0;
static uint16_t g_seg_ms = 0;
static uint8_t  g_settle_state = 0;   /* arc->straight pivot: 0 idle 1 brake 2 turn */
static uint16_t g_settle_ms = 0;
static uint8_t  g_settle_used = 0;    /* this arc needed the pivot */
static float    g_track_yaw = 0.0f;   /* nominal track heading, run start +/- n*180 */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float line_error(uint8_t bits, uint8_t *lost)
{
    int i;
    int count = 0;
    int weighted = 0;
    int active = 0;

    for (i = 0; i < 8; i++)
    {
        if (GRAY_MASK & (1u << i))
        {
            continue;              /* faulty channel, ignore it */
        }
        active++;
        if (bits & (1u << i))
        {
            weighted += i;
            count++;
        }
    }

    /* lost = 1: all white, the line is out of the array's view on one side.
     * lost = 2: all black, junction / line wider than the array, no side.
     * The caller needs the difference: in case 1 it can still steer back
     * toward the side the line left on, in case 2 it cannot. */
    if (count == 0)
    {
        *lost = 1;
        return g_last_line_err;
    }

    if (count >= active)
    {
        *lost = 2;
        return g_last_line_err;
    }

    *lost = 0;

    return (((float)weighted / (float)count) - 3.5f) / 3.5f;
}

static void begin_segment(const PathInput *in)
{
    const Segment *seg = &g_path[g_seg_index];

    g_seg_start_dist = in->distance_mm;
    g_seg_yaw_start = Imu_GetYaw();
    g_arc_yaw_start = g_seg_yaw_start;
    /* g_last_line_err is NOT cleared: it is the only memory of which side the
     * line was last seen on, and the line follower needs it to steer back
     * after an off-line entry. */
    g_gray_corr = 0.0f;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;
    g_arc_mode = 0;
    g_yaw_ref_dbg = Imu_GetYaw();
    g_arc_radius = 0.0f;
    g_rep_e_min = 1000;  g_rep_e_max = -1000;
    g_rep_gc_min = 1000; g_rep_gc_max = -1000;
    g_rep_g_min = 255;   g_rep_g_max = 0;
    g_rep_lost = 0;
    g_line_lost = 0;
    g_turn_ms = 0;
    g_seg_ms = 0;
    g_settle_state = 0;
    g_settle_ms = 0;
    g_settle_used = 0;
    g_arc_why = 0;

    /* After an arc, hold the nominal track heading (arc start +/- 180) on the
     * following straight instead of the measured yaw, so the few degrees left
     * over from the arc are actively driven out over the straight. */
    if ((seg->type == SEG_STRAIGHT) && g_nominal_yaw_valid)
    {
        g_target_yaw = g_nominal_yaw;
    }
    else
    {
        g_target_yaw = Imu_GetYaw();
    }
    g_nominal_yaw_valid = 0;

    if (seg->type == SEG_TURN)
    {
        g_turn_target_yaw = Imu_GetYaw() + seg->length;
    }
}

static void advance_segment(const PathInput *in, PathOutput *out)
{
    const Segment *seg = &g_path[g_seg_index];

    if (seg->announce)
    {
        out->event = PATH_EVENT_POINT;
    }

    g_seg_index++;

    if (g_seg_index >= g_path_len)
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    if (g_path[g_seg_index].type == SEG_STOP)
    {
        if (g_lap + 1 < g_laps_total)
        {
            g_lap++;
            g_seg_index = 0;
            begin_segment(in);
            out->left_mmps = in->base_speed_mmps;
            out->right_mmps = in->base_speed_mmps;
            return;
        }

        g_state = PATH_DONE;
        out->brake = 1;
        out->left_mmps = 0.0f;
        out->right_mmps = 0.0f;
        out->event = PATH_EVENT_STOP;
        return;
    }

    begin_segment(in);

    /* keep rolling for one 5 ms tick while the next segment takes over */
    out->left_mmps = in->base_speed_mmps;
    out->right_mmps = in->base_speed_mmps;
}

/* -------------------------------------------------------------------------
 * Grayscale line steering, shared by SEG_LINE and SEG_ARC_LINE.
 *
 *   w_line    = -(KP * e + KD * de/dt)            deg/s, clamped
 *   theta     += w_line * dt, clamped             deg (the integral of the
 *                                                 line rate: this is what
 *                                                 removes a standing offset)
 *   yaw_ref   = yaw_tangent + theta
 *   rate_ref  = rate_ff + w_line
 *   turn_corr = ARC_YAW_KP * (yaw_ref - yaw)
 *             - ARC_YAW_KD * (gyro_z - rate_ref)
 *   wheels    = base -/+ (turn_ff + turn_corr)
 *
 * yaw_tangent is the direction the segment is supposed to have right now
 * (constant on a straight, rotating on an arc).  turn_ff / rate_ff are the
 * curvature feedforward of that segment (zero on a straight).
 * ------------------------------------------------------------------------- */
static void line_steer(const PathInput *in, PathOutput *out, float base,
                       float yaw_tangent, float turn_ff, float rate_ff,
                       float err, float prev_err, uint8_t lost, uint8_t reacq)
{
    float w_line = 0.0f;
    float yaw_ref;
    float rate_ref;
    float turn_corr;

    if (lost == 1)
    {
        if (g_line_lost < 250) { g_line_lost++; }
        /* Line out of the array: put the heading offset on the side the line
         * left on, and keep it there until the line is seen again. */
        if (reacq)
        {
            if (g_last_line_err > 0.02f)
            {
                g_line_theta = -ARC_LINE_REACQ_DEG;
            }
            else if (g_last_line_err < -0.02f)
            {
                g_line_theta = ARC_LINE_REACQ_DEG;
            }
            else
            {
                g_line_theta *= ARC_GRAY_LOST_DECAY;
            }
        }
        else
        {
            g_line_theta *= ARC_GRAY_LOST_DECAY;
        }
    }
    else if (lost == 2)
    {
        if (g_line_lost < 250) { g_line_lost++; }
        /* Junction / line wider than the array: no side information, hold the
         * offset we already had. */
    }
    else
    {
        float de = (err - prev_err) / in->dt;

        /* The derivative is noisy on a digital 8-step error: filter it. */
        g_line_d_f = (1.0f - ARC_LINE_D_FILT) * g_line_d_f +
                     ARC_LINE_D_FILT * de;

        w_line = -(ARC_LINE_KP * err + ARC_LINE_KD * g_line_d_f);
        w_line = clampf(w_line, -ARC_LINE_W_MAX, ARC_LINE_W_MAX);

        g_line_theta += w_line * in->dt;
        g_line_theta = clampf(g_line_theta,
                              -ARC_LINE_THETA_MAX, ARC_LINE_THETA_MAX);

        g_line_lost = 0;
    }

    g_line_rate_dbg = w_line;
    g_gray_corr = g_line_theta;          /* one number for both modes/log */
    yaw_ref = yaw_tangent + g_line_theta;
    rate_ref = rate_ff + w_line;
    g_yaw_ref_dbg = yaw_ref;

    if (Imu_IsReady())
    {
        /* Rate feedback on (gyro - rate_ref): a steady turn produces no
         * offset, it only damps overshoot. */
        turn_corr = ARC_YAW_KP * (yaw_ref - in->yaw_deg) -
                    ARC_YAW_KD * (in->gyro_z_dps - rate_ref);
    }
    else
    {
        /* No gyro: a heading reference is meaningless, fall back to a plain
         * grayscale differential so the car still follows the line. */
        turn_corr = -(ARC_LINE_FALLBACK_KP * clampf(err, -1.0f, 1.0f));
    }
    turn_corr = clampf(turn_corr, -ARC_CORR_LIMIT, ARC_CORR_LIMIT);

    out->left_mmps  = base - (turn_ff + turn_corr);
    out->right_mmps = base + (turn_ff + turn_corr);
}

/* Original MPU heading hold straight (also the SEG_LINE fallback when the
 * grayscale link is down). */
static void straight_steer(const PathInput *in, PathOutput *out, float base)
{
    float err = g_target_yaw - in->yaw_deg;
    float turn = YAW_KP * err - YAW_KD * in->gyro_z_dps;

    turn = clampf(turn, -YAW_TURN_LIMIT, YAW_TURN_LIMIT);

    out->left_mmps = base - turn;
    out->right_mmps = base + turn;
}

/* Finish the current arc: snap / hand the nominal heading to the next
 * straight, publish the one-line report, move on.
 *
 * target is the heading this arc must end on.  For a grayscale arc it is the
 * nominal track heading (run start +/- 180, then +/-180 per arc), so a crab
 * angle picked up on a line-followed straight is not carried into the next
 * one; for a schedule arc it stays relative to the arc start. */
static void arc_finish(const PathInput *in, PathOutput *out, float dist,
                       float target)
{
    float resid = fabsf(target - in->yaw_deg);

    /* Snap only when the arc really did end on the target; a few degrees left
     * over are handed to the next straight as the nominal track heading. */
    if (resid <= ARC_YAW_SNAP_DEG)
    {
        Imu_SetYaw(target);
    }
    else if ((resid <= ARC_YAW_FIX_MAX) || (g_arc_mode && (resid <= 45.0f)))
    {
        g_nominal_yaw = target;
        g_nominal_yaw_valid = 1;
    }

    if (g_arc_mode)
    {
        g_track_yaw = target;        /* next arc counts its 180 deg from here */
    }

    g_gray_corr = 0.0f;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;

    if (g_rep_e_min > 999)  { g_rep_e_min = 0; }
    if (g_rep_e_max < -999) { g_rep_e_max = 0; }
    g_arc_rep.dist_mm   = (int16_t)dist;
    g_arc_rep.yaw_deg   = (int16_t)in->yaw_deg;
    g_arc_rep.radius_mm = (int16_t)g_arc_radius;
    g_arc_rep.e_min     = g_rep_e_min;
    g_arc_rep.e_max     = g_rep_e_max;
    g_arc_rep.gc_min    = (int8_t)g_rep_gc_min;
    g_arc_rep.gc_max    = (int8_t)g_rep_gc_max;
    g_arc_rep.g_min     = g_rep_g_min;
    g_arc_rep.g_max     = g_rep_g_max;
    g_arc_rep.lost      = g_rep_lost;
    g_arc_rep.mode      = g_arc_mode;
    g_arc_rep.why       = g_arc_why;
    g_arc_rep_pending   = 1;

    advance_segment(in, out);
}

/* Is the segment after the current one a straight?  Only then does it make
 * sense to stand still and pivot to the exact 180 deg at the end of an arc. */
static uint8_t Path_NextIsStraight(void)
{
    uint8_t next = (uint8_t)(g_seg_index + 1u);

    if (next >= g_path_len)
    {
        return 0;
    }
    if ((g_path[next].type == SEG_STRAIGHT) || (g_path[next].type == SEG_LINE))
    {
        return 1;
    }
    return 0;
}

/* Stand still, then pivot in place to target_yaw, then let the caller finish
 * the segment.  Returns 1 once the heading is there (or the pivot timed out),
 * 0 while it is still braking / turning.
 *
 * This is the "180 deg handover": the line-following arc ends by distance and
 * may be a few degrees short, so the car stops and turns the rest of the way
 * before the straight starts - the straight then really points along the
 * track instead of inheriting the arc error. */
static uint8_t arc_settle(const PathInput *in, PathOutput *out, float target_yaw)
{
    float err = target_yaw - in->yaw_deg;
    float v;

    if (g_settle_state == 0u)
    {
        g_settle_state = 1u;
        g_settle_ms = 0;
    }

    g_settle_ms = (uint16_t)(g_settle_ms + (uint16_t)(in->dt * 1000.0f));

    if (g_settle_state == 1u)
    {
        /* let the car come to a full stop before turning in place */
        out->left_mmps = 0.0f;
        out->right_mmps = 0.0f;
        if (g_settle_ms >= ARC_SETTLE_BRAKE_MS)
        {
            g_settle_state = 2u;
            g_settle_ms = 0;
        }
        return 0;
    }

    if (fabsf(err) <= ARC_SETTLE_DONE_TOL)
    {
        Imu_SetYaw(target_yaw);          /* residual is small: snap the model */
        g_settle_state = 0u;
        g_settle_ms = 0;
        return 1;
    }

    if (g_settle_ms > TURN_TIMEOUT_MS)
    {
        /* Could not reach it (IMU drift / blocked wheel): give up without
         * snapping, arc_finish hands the residual to the next straight. */
        g_settle_state = 0u;
        g_settle_ms = 0;
        return 1;
    }

    v = TURN_KP * fabsf(err);
    v = clampf(v, TURN_MIN_MMPS, TURN_MAX_MMPS);

    if (err > 0.0f)
    {
        /* left turn: right wheel is the outer wheel */
        out->left_mmps = 0.0f;
        out->right_mmps = v;
    }
    else
    {
        out->left_mmps = v;
        out->right_mmps = 0.0f;
    }
    return 0;
}

/* One 5 ms step of an arc.
 *
 * use_line = 0: the original release1.9 behaviour.  The heading reference is
 *               the dead-reckoned 180 deg schedule, the grayscale may trim it
 *               by at most ARC_GRAY_CORR_MAX degrees.  Kept for Q3/Q4 and as
 *               the fallback when the serial link is down.
 * use_line = 1: Q2 behaviour.  The grayscale owns the steering: its error is
 *               integrated into a heading offset (theta_line, bounded by
 *               ARC_LINE_THETA_MAX) that is added to the geometric tangent.
 *               The schedule therefore only sets the pace of the turn, while
 *               the line decides where the car actually goes. */
static void arc_step(const PathInput *in, PathOutput *out, const Segment *seg,
                     float base, uint8_t use_line)
{
    float dist = in->distance_mm - g_seg_start_dist;
    float progress;
    float yaw_sched;
    float err;
    float prev_err;
    uint8_t lost;
    float turn_ff;
    float rate_ff;
    float yaw_done;
    float line_done;
    float dist_done;

    g_arc_mode = use_line;

    /* 1) Scheduled heading: the 180 deg of the arc spread over its length,
     *    driven by the encoder distance.  This is what makes the reference
     *    independent of the actual speed.
     *    In line mode the schedule stops exactly at 180 deg (clamp 1.00): when
     *    the segment is held open waiting for the real 180 deg, the reference
     *    must stay on the target and not creep past it. */
    progress = dist / seg->length;
    progress = clampf(progress, 0.0f, use_line ? 1.00f : 1.05f);
    yaw_sched = g_arc_yaw_start + (float)seg->arc_dir * 180.0f * progress;

    /* 2) Line error.  line_err > 0 = line is on the OUT8 side. */
    prev_err = g_last_line_err;
    err = line_error(in->gray_bits, &lost);

    /* 3) Finish.  yaw_done: gross over-rotation guard.  line_done: the line
     *    really ended near the end of the arc.  dist_done: encoder distance.
     *    For a line arc followed by a straight, a heading that is still off
     *    is fixed by the stand-still pivot in 3b before the segment ends. */
    yaw_done = (fabsf(in->yaw_deg - g_arc_yaw_start) > ARC_YAW_DONE_DEG) &&
               (dist > seg->length * 0.97f);
    line_done = (dist > seg->length * 0.95f) &&
                (g_line_lost >= ARC_LINE_LOST_SAMPLES) &&
                (fabsf((g_arc_yaw_start + (float)seg->arc_dir * 180.0f) -
                       in->yaw_deg) <= 10.0f);
    dist_done = (dist > seg->length + ARC_FINISH_MARGIN_MM);

    if (yaw_done || line_done || dist_done)
    {
        /* Heading this arc has to end on.  A grayscale arc aims at the
         * nominal track heading (run start +/- 180 deg, advanced by 180 deg
         * per arc), so a crab angle from the preceding line-followed straight
         * does not get carried into the next straight. */
        float target;
        float resid;

        if (use_line)
        {
#if ARC_SETTLE_ABS_YAW
            /* 180 deg away from the heading the run started with */
            target = g_track_yaw + (float)seg->arc_dir * 180.0f;
#else
            /* 180 deg away from this arc's entry heading */
            target = g_arc_yaw_start + (float)seg->arc_dir * 180.0f;
#endif
        }
        else
        {
            target = g_arc_yaw_start + (float)seg->arc_dir * 180.0f;
        }
        resid = fabsf(target - in->yaw_deg);

        /* 3b) 180 deg handover.  If a straight follows this line arc, stop
         *     and pivot to the exact heading first, so the straight starts
         *     pointing the right way instead of inheriting a few degrees of
         *     arc error.  arc_settle() returns 0 while it is still turning. */
        if (use_line && Imu_IsReady() && Path_NextIsStraight() &&
            ((resid > ARC_SETTLE_TOL) || (g_settle_used != 0u)))
        {
            g_settle_used = 1u;
            if (!arc_settle(in, out, target))
            {
                return;              /* still braking / pivoting */
            }
            g_arc_why = 4u;          /* ended after the pivot */
        }
        else if (g_settle_used == 0u)
        {
            /* 1 = yaw guard, 2 = line lost, 3 = encoder distance. */
            g_arc_why = yaw_done ? 1u : (line_done ? 2u : 3u);
        }

        arc_finish(in, out, dist, target);
        return;
    }

    turn_ff = (float)seg->arc_dir * base * CAR_TRACK_WIDTH_MM * ARC_FF_TRIM /
              (2.0f * TRACK_RADIUS_MM);
    rate_ff = (float)seg->arc_dir * 180.0f * base / seg->length;

    if (use_line)
    {
        line_steer(in, out, base, yaw_sched, turn_ff, rate_ff,
                   err, prev_err, lost, (dist < seg->length * 0.95f));
    }
    else
    {
        /* ---- original schedule arc, grayscale only trims the heading ---- */
        float yaw_ref;
        float yaw_err;
        float turn_corr;

        if (lost == 1)
        {
            if (g_line_lost < 250) { g_line_lost++; }
            if (dist < seg->length * 0.95f)
            {
                if (g_last_line_err > 0.02f)
                {
                    g_gray_corr = -ARC_REACQ_DEG;
                }
                else if (g_last_line_err < -0.02f)
                {
                    g_gray_corr = ARC_REACQ_DEG;
                }
                else
                {
                    g_gray_corr *= ARC_GRAY_LOST_DECAY;
                }
            }
            else
            {
                g_gray_corr *= ARC_GRAY_LOST_DECAY;
            }
        }
        else if (lost == 2)
        {
            if (g_line_lost < 250) { g_line_lost++; }
            g_gray_corr *= ARC_GRAY_LOST_DECAY;
        }
        else
        {
            float corr = -(ARC_GRAY_TRIM_KP * err +
                           ARC_GRAY_TRIM_KD * (err - prev_err));
            g_gray_corr = clampf(corr, -ARC_GRAY_CORR_MAX, ARC_GRAY_CORR_MAX);
            g_line_lost = 0;
        }

        yaw_ref = yaw_sched + g_gray_corr;
        g_yaw_ref_dbg = yaw_ref;

        if (Imu_IsReady())
        {
            yaw_err = yaw_ref - in->yaw_deg;
            turn_corr = ARC_YAW_KP * yaw_err -
                        ARC_YAW_KD * (in->gyro_z_dps - rate_ff);
        }
        else
        {
            turn_corr = -(ARC_LINE_FALLBACK_KP * clampf(err, -1.0f, 1.0f));
        }
        turn_corr = clampf(turn_corr, -ARC_CORR_LIMIT, ARC_CORR_LIMIT);

        out->left_mmps  = base - (turn_ff + turn_corr);
        out->right_mmps = base + (turn_ff + turn_corr);
    }

    g_last_line_err = err;

    /* Accumulate statistics for the one-line arc report. */
    {
        int16_t ev = (int16_t)(err * 100.0f);
        int16_t gv = (int16_t)g_gray_corr;

        if (ev < g_rep_e_min)  { g_rep_e_min = ev; }
        if (ev > g_rep_e_max)  { g_rep_e_max = ev; }
        if (gv < g_rep_gc_min) { g_rep_gc_min = gv; }
        if (gv > g_rep_gc_max) { g_rep_gc_max = gv; }
        if (in->gray_bits < g_rep_g_min) { g_rep_g_min = in->gray_bits; }
        if (in->gray_bits > g_rep_g_max) { g_rep_g_max = in->gray_bits; }
        if (lost) { g_rep_lost++; }
    }

    /* Measured radius, for feedforward calibration:
     * R = travelled arc length / swept angle. Only meaningful once
     * the car has actually turned through ~60 deg, and only while the car is
     * still on the arc (not during the 180 deg handover overrun). */
    {
        float swept = fabsf(in->yaw_deg - g_arc_yaw_start) * 0.017453293f;
        if ((swept > 1.0472f) &&
            (dist <= seg->length + ARC_FINISH_MARGIN_MM))
        {
            g_arc_radius = dist / swept;
        }
    }
}

void Path_Init(void)
{
    g_state = PATH_IDLE;
    g_mode = 0;
    g_seg_index = 0;
    g_lap = 0;
    g_laps_total = 1;
    g_need_begin = 0;
    g_path = 0;
    g_path_len = 0;
    g_last_line_err = 0.0f;
    g_gray_corr = 0.0f;
    g_yaw_ref_dbg = 0.0f;
    g_nominal_yaw = 0.0f;
    g_nominal_yaw_valid = 0;
    g_line_lost = 0;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;
    g_arc_mode = 0;
    g_seg_yaw_start = 0.0f;
    g_settle_state = 0;
    g_settle_ms = 0;
}

void Path_Start(uint8_t mode)
{
    if (mode < 1) { mode = 1; }
    if (mode > 4) { mode = 4; }

    g_mode = mode;
    g_seg_index = 0;
    g_lap = 0;
    g_state = PATH_RUNNING;
    g_need_begin = 1;
    /* The nominal track heading starts where the run started (the UI resets
     * the yaw before Path_Start), then advances by +/-180 per line arc. */
    g_track_yaw = Imu_GetYaw();
    g_settle_state = 0;
    g_settle_ms = 0;
    g_settle_used = 0;

    switch (mode)
    {
        case 1:
            g_path = g_path_q1;
            g_path_len = (uint8_t)(sizeof(g_path_q1) / sizeof(g_path_q1[0]));
            g_laps_total = 1;
            break;
        case 2:
            g_path = g_path_q2;
            g_path_len = (uint8_t)(sizeof(g_path_q2) / sizeof(g_path_q2[0]));
            g_laps_total = 1;
            break;
        case 3:
            g_path = g_path_q3;
            g_path_len = (uint8_t)(sizeof(g_path_q3) / sizeof(g_path_q3[0]));
            g_laps_total = 1;
            break;
        case 4:
        default:
            g_path = g_path_q3;
            g_path_len = (uint8_t)(sizeof(g_path_q3) / sizeof(g_path_q3[0]));
            g_laps_total = 4;
            break;
    }
}

void Path_Stop(void)
{
    g_state = PATH_IDLE;
    g_need_begin = 0;
}

uint8_t Path_IsRunning(void)
{
    return (g_state == PATH_RUNNING) ? 1 : 0;
}

uint8_t Path_GetMode(void)
{
    return g_mode;
}

uint8_t Path_GetSegment(void)
{
    return g_seg_index;
}

void Path_Update(const PathInput *in, PathOutput *out)
{
    const Segment *seg;
    float base;
    float scale;

    out->left_mmps = 0.0f;
    out->right_mmps = 0.0f;
    out->brake = 0;
    out->event = PATH_EVENT_NONE;

    if (g_state != PATH_RUNNING)
    {
        /* Keep the serial e= field live while parked so the grayscale
         * left/right order can be checked without starting a run. */
        {
            uint8_t line_lost;
            g_last_line_err = line_error(in->gray_bits, &line_lost);
        }
        out->brake = 1;
        return;
    }

    if (g_need_begin)
    {
        g_need_begin = 0;
        begin_segment(in);
        out->event = PATH_EVENT_START;
    }

    g_seg_ms = (uint16_t)(g_seg_ms + (uint16_t)(in->dt * 1000.0f));
    if (g_seg_ms > SEGMENT_TIMEOUT_MS)
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    if (g_seg_index >= g_path_len)
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    seg = &g_path[g_seg_index];
    scale = (g_mode == 4) ? 1.15f : 1.0f;
    /* speed_ratio lets each segment ask for its own fraction of the base
     * speed (arcs use 0.75 so the car is not at full speed on a 400 mm
     * radius). */
    base = clampf(in->base_speed_mmps * scale * seg->speed_ratio,
                  120.0f, MAX_SPEED_MMPS);

    switch (seg->type)
    {
        case SEG_STRAIGHT:
        {
            float dist = in->distance_mm - g_seg_start_dist;

            /* Memory only: the straight is steered by the MPU6050, but the
             * last line position is kept so the next segment can still pick
             * a side to reacquire on. */
            {
                uint8_t line_lost;
                g_last_line_err = line_error(in->gray_bits, &line_lost);
            }

            if (dist >= seg->length)
            {
                advance_segment(in, out);
                return;
            }

            straight_steer(in, out, base);
            break;
        }

        case SEG_LINE:
        {
            float dist = in->distance_mm - g_seg_start_dist;
            uint8_t lost;
            uint8_t online;
            float err;

            if (dist >= seg->length)
            {
                advance_segment(in, out);
                return;
            }

            online = Gray_IsOnline();
            g_arc_mode = online;         /* the log's lm= field */

            if (!online)
            {
                /* No serial frames: keep the segment going on the MPU. */
                uint8_t line_lost;
                g_last_line_err = line_error(in->gray_bits, &line_lost);
                straight_steer(in, out, base);
                break;
            }

            err = line_error(in->gray_bits, &lost);
            line_steer(in, out, base, g_seg_yaw_start, 0.0f, 0.0f,
                       err, g_last_line_err, lost, 1);
            g_last_line_err = err;
            break;
        }

        case SEG_ARC:
            arc_step(in, out, seg, base, 0);
            break;

        case SEG_ARC_LINE:
            /* If the serial grayscale link is down there is no line to
             * follow: fall back to the dead-reckoned arc so the run still
             * finishes instead of flying blind on a stale bitmap. */
            arc_step(in, out, seg, base, Gray_IsOnline());
            break;

        case SEG_TURN:
        {
            float err = g_turn_target_yaw - in->yaw_deg;
            float v;

            g_turn_ms = (uint16_t)(g_turn_ms + (uint16_t)(in->dt * 1000.0f));

            if ((fabsf(err) < TURN_TOL_DEG) || (g_turn_ms > TURN_TIMEOUT_MS))
            {
                if (fabsf(err) < TURN_TOL_DEG)
                {
                    Imu_SetYaw(g_turn_target_yaw);
                }
                advance_segment(in, out);
                return;
            }

            v = TURN_KP * fabsf(err);
            v = clampf(v, TURN_MIN_MMPS, TURN_MAX_MMPS);

            if (err > 0.0f)
            {
                /* left turn: right wheel is the outer wheel */
                out->left_mmps = v * TURN_INNER_RATIO;
                out->right_mmps = v;
            }
            else
            {
                out->left_mmps = v;
                out->right_mmps = v * TURN_INNER_RATIO;
            }
            break;
        }

        case SEG_STOP:
        default:
            g_state = PATH_DONE;
            out->brake = 1;
            out->left_mmps = 0.0f;
            out->right_mmps = 0.0f;
            out->event = PATH_EVENT_STOP;
            return;
    }

    if (out->left_mmps < 0.0f)  { out->left_mmps = 0.0f; }
    if (out->right_mmps < 0.0f) { out->right_mmps = 0.0f; }
    if (out->left_mmps > MAX_SPEED_MMPS)  { out->left_mmps = MAX_SPEED_MMPS; }
    if (out->right_mmps > MAX_SPEED_MMPS) { out->right_mmps = MAX_SPEED_MMPS; }
}

/* Debug helpers: normalized line error and the last arc heading reference. */
float Path_GetLineError(void)
{
    return g_last_line_err;
}

float Path_GetYawRef(void)
{
    return g_yaw_ref_dbg;
}

float Path_GetGrayCorr(void)
{
    return g_gray_corr;
}

float Path_GetArcRadius(void)
{
    return g_arc_radius;
}

float Path_GetLineTheta(void)
{
    return g_line_theta;
}

float Path_GetLineRate(void)
{
    return g_line_rate_dbg;
}

uint8_t Path_GetArcMode(void)
{
    return g_arc_mode;
}

uint8_t Path_GetArcWhy(void)
{
    return g_arc_why;
}

uint8_t Path_TakeArcReport(ArcReport *out)
{
    if (!g_arc_rep_pending)
    {
        return 0;
    }
    *out = g_arc_rep;
    g_arc_rep_pending = 0;
    return 1;
}
