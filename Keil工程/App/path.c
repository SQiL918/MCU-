#include "stm32f10x.h"
#include <math.h>
#include "path.h"
#include "app_config.h"
#include "imu.h"

#define PATH_MAX_SEGMENTS 16

static const Segment g_path_q1[] =
{
    { SEG_STRAIGHT, TRACK_AB_MM, 1.00f, 0, 1 },
    { SEG_STOP,     0.0f,        0.00f, 0, 0 }
};

static const Segment g_path_q2[] =
{
    { SEG_STRAIGHT, TRACK_AB_MM,          1.00f,  0, 1 }, /* B */
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, -1, 1 }, /* C, right arc */
    { SEG_STRAIGHT, TRACK_CD_MM,          1.00f,  0, 1 }, /* D */
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, +1, 1 }, /* A, left arc */
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
static float g_arc_yaw_start = 0.0f;
static float g_turn_target_yaw = 0.0f;
static float g_last_line_err = 0.0f;
static uint8_t g_line_lost = 0;
static uint16_t g_turn_ms = 0;
static uint16_t g_seg_ms = 0;

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

    for (i = 0; i < 8; i++)
    {
        if (bits & (1u << i))
        {
            weighted += i;
            count++;
        }
    }

    if (count == 0)
    {
        *lost = 1;
        return g_last_line_err;
    }

    *lost = 0;

    if (count >= 7)
    {
        return 0.0f;   /* all black: cross line / intersection */
    }

    return (((float)weighted / (float)count) - 3.5f) / 3.5f;
}

static void begin_segment(const PathInput *in)
{
    const Segment *seg = &g_path[g_seg_index];

    g_seg_start_dist = in->distance_mm;
    g_target_yaw = Imu_GetYaw();
    g_arc_yaw_start = Imu_GetYaw();
    g_last_line_err = 0.0f;
    g_line_lost = 0;
    g_turn_ms = 0;
    g_seg_ms = 0;

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
    base = clampf(in->base_speed_mmps * scale, 120.0f, MAX_SPEED_MMPS);

    switch (seg->type)
    {
        case SEG_STRAIGHT:
        {
            float dist = in->distance_mm - g_seg_start_dist;

            if (dist >= seg->length)
            {
                advance_segment(in, out);
                return;
            }

            {
                float err = g_target_yaw - in->yaw_deg;
                float turn = YAW_KP * err - YAW_KD * in->gyro_z_dps;

                turn = clampf(turn, -YAW_TURN_LIMIT, YAW_TURN_LIMIT);

                out->left_mmps = base - turn;
                out->right_mmps = base + turn;
            }
            break;
        }

        case SEG_ARC:
        {
            float dist = in->distance_mm - g_seg_start_dist;
            float err;
            float prev_err;
            uint8_t lost;
            float turn_ff;
            float turn_pd;
            float yaw_done;
            float line_done;
            float dist_done;

            prev_err = g_last_line_err;
            err = line_error(in->gray_bits, &lost);
            g_last_line_err = err;

            if (lost) { g_line_lost++; }
            else      { g_line_lost = 0; }

            yaw_done = (fabsf(in->yaw_deg - g_arc_yaw_start) > 170.0f) &&
                       (dist > seg->length * 0.80f);
            line_done = (dist > seg->length * 0.85f) &&
                        (g_line_lost >= ARC_LINE_LOST_SAMPLES);
            dist_done = (dist > seg->length + ARC_FINISH_MARGIN_MM);

            if (yaw_done || line_done || dist_done)
            {
                /* The physical arc is exactly 180 degrees, so snap the
                 * integrated heading to remove gyro drift. */
                Imu_SetYaw(g_arc_yaw_start + (float)seg->arc_dir * 180.0f);
                advance_segment(in, out);
                return;
            }

            turn_ff = (float)seg->arc_dir * base * CAR_TRACK_WIDTH_MM /
                      (2.0f * TRACK_RADIUS_MM);
            turn_pd = -(LINE_KP * err + LINE_KD * (err - prev_err));

            turn_ff = clampf(turn_ff, -LINE_TURN_LIMIT, LINE_TURN_LIMIT);
            turn_pd = clampf(turn_pd, -LINE_TURN_LIMIT, LINE_TURN_LIMIT);

            out->left_mmps = base - (turn_ff + turn_pd);
            out->right_mmps = base + (turn_ff + turn_pd);
            break;
        }

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
