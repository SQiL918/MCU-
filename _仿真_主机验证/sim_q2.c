/* =========================================================================
 * Host simulation of App/path.c (Q2 line-following arc).
 *
 * Compiles the REAL path.c + app_config.h and drives it with a simple
 * differential-drive plant on the Q2 stadium: 1000 mm straight, 180 deg
 * right arc (R = 400 mm), 1000 mm straight, 180 deg right arc.
 *
 * The 8-channel array is modelled at the axle: channel i sits at
 * (i - 3.5) * 7 mm to the right of the centreline (OUT1 = leftmost, which
 * is the wiring the firmware assumes; set GRAY_REVERSE = 1 if your ribbon
 * is the other way round).  A channel is black when it is within 10 mm of
 * the line.  The array is sampled at 100 Hz, the control runs at 200 Hz.
 * ========================================================================= */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32f10x.h"
#include "app_config.h"
#include "path.h"

#define DT              (0.005f)
#define SENSOR_SPACING  7.0f
#define LINE_HALF_W     10.0f
#define MOTOR_TAU       0.08f

/* ---------------- plant state ---------------- */
static float p_x, p_y, p_th, p_vl, p_vr, p_dist, p_yaw, p_gyro;

/* ---------------- stubs for the firmware modules ---------------- */
static int     s_imu_ready = 1;
static int     s_gray_online = 1;

float   Imu_GetYaw(void)          { return p_yaw; }
float   Imu_GetGyroZ(void)        { return p_gyro; }
void    Imu_SetYaw(float y)       { p_yaw = y; }
void    Imu_ResetYaw(void)        { p_yaw = 0.0f; p_gyro = 0.0f; }
uint8_t Imu_IsReady(void)         { return (uint8_t)s_imu_ready; }
uint8_t Gray_IsOnline(void)       { return (uint8_t)s_gray_online; }

/* ---------------- track: Q2 stadium ---------------- */
static float seg_dist(float px, float py, float ax, float ay, float bx, float by)
{
    float vx = bx - ax, vy = by - ay;
    float wx = px - ax, wy = py - ay;
    float t = (vx * wx + vy * wy) / (vx * vx + vy * vy);
    float dx, dy;

    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }
    dx = px - (ax + t * vx);
    dy = py - (ay + t * vy);
    return sqrtf(dx * dx + dy * dy);
}

/* right = 1: arc spans the +x half of its circle (first arc)
 * right = 0: arc spans the -x half (second arc), which is what closes the
 *            Q2 stadium after the C->D straight. */
static float arc_dist(float px, float py, float cx, float cy, float r, int right)
{
    float dx = px - cx, dy = py - cy;
    float ang = atan2f(dy, dx);
    int in_span;

    in_span = right ? (fabsf(ang) <= 1.5707963f) : (fabsf(ang) >= 1.5707963f);
    if (in_span)
    {
        return fabsf(sqrtf(dx * dx + dy * dy) - r);
    }
    return 1.0e9f;   /* outside the arc span: the adjoining straight covers it */
}

/* Real track arc radius.  400 = what the firmware assumes.  Other values let
 * us test an arc wider/narrower than the model, which is what makes the car
 * reach the arc length before (wider) or after (narrower) the full 180 deg. */
static float g_R = 400.0f;

static float track_dist(float px, float py)
{
    float d = seg_dist(px, py, 0.0f, 0.0f, 1000.0f, 0.0f);
    float t;

    t = arc_dist(px, py, 1000.0f, -g_R, g_R, 1);
    if (t < d) { d = t; }
    t = seg_dist(px, py, 1000.0f, -2.0f * g_R, 0.0f, -2.0f * g_R);
    if (t < d) { d = t; }
    t = arc_dist(px, py, 0.0f, -g_R, g_R, 0);
    if (t < d) { d = t; }
    return d;
}

/* Signed error of the line under the array, + = line to the right. */
static uint8_t sensor_bits(float x, float y, float th)
{
    uint8_t bits = 0;
    int i;

    for (i = 0; i < 8; i++)
    {
        float s = -24.5f + SENSOR_SPACING * (float)i;   /* + = right */
        float rx = sinf(th);
        float ry = -cosf(th);
        float px = x + s * rx;
        float py = y + s * ry;

        if (track_dist(px, py) < LINE_HALF_W)
        {
            bits |= (uint8_t)(1u << i);
        }
    }
    return bits;
}

/* ---------------- plant ---------------- */
static void plant_reset(float x0, float y0, float th0_deg, float dist0)
{
    p_x = x0; p_y = y0; p_th = th0_deg * 0.017453293f;
    p_vl = 0.0f; p_vr = 0.0f;
    p_dist = dist0; p_yaw = 0.0f; p_gyro = 0.0f;
}

static void plant_step(float l, float r)
{
    float k = DT / (MOTOR_TAU + DT);
    float v;

    p_vl += (l - p_vl) * k;
    p_vr += (r - p_vr) * k;

    p_gyro = (p_vr - p_vl) / CAR_TRACK_WIDTH_MM * 57.29578f;
    p_yaw += p_gyro * DT;
    p_th += p_gyro * 0.017453293f * DT;

    v = 0.5f * (p_vl + p_vr);
    p_x += v * cosf(p_th) * DT;
    p_y += v * sinf(p_th) * DT;
    p_dist += v * DT;
}

/* ---------------- run one Q2 (or Q3) run ---------------- */
static int run(int mode, float base, float y0, float th0, const char *csv,
               float *out_maxdev, float *out_time, float *out_yaw,
               int drop_from, int drop_to)
{
    PathInput in;
    PathOutput out;
    uint8_t bits = 0;
    int tick;
    int tick_max = 12000;             /* 60 s: enough for 300 mm/s */
    int done_ticks = 0;
    float maxdev = 0.0f;
    FILE *f = 0;
    int arc = 0;
    int gray_ok = s_gray_online;

    plant_reset(0.0f, y0, th0, 0.0f);
    Path_Init();
    Path_Start((uint8_t)mode);

    bits = sensor_bits(p_x, p_y, p_th);

    if (csv)
    {
        f = fopen(csv, "w");
        if (f) { fprintf(f, "t,x,y,th,yaw,e,d,bits,lt,l,r\n"); }
    }

    for (tick = 0; tick < tick_max; tick++)
    {
        float dev;
        float l, r;

        if (drop_to > 0 && tick >= drop_from && tick < drop_to)
        {
            s_gray_online = 0;      /* link dropout: fall back to the schedule */
        }
        else
        {
            s_gray_online = gray_ok;
        }

        if ((tick & 1) == 0)
        {
            bits = sensor_bits(p_x, p_y, p_th);    /* 100 Hz array */
        }

        in.distance_mm = p_dist;
        in.yaw_deg = p_yaw;
        in.gyro_z_dps = p_gyro;
        in.gray_bits = bits;
        in.dt = DT;
        in.base_speed_mmps = base;

        Path_Update(&in, &out);

        {
            ArcReport rep;
            if (Path_TakeArcReport(&rep))
            {
                printf("  t=%.2f ARC %d: d=%d yaw=%d Rr=%d lm=%u e=%d..%d gc=%d..%d "
                       "g=%u..%u lost=%u why=%u\n", tick*DT, arc++, (int)rep.dist_mm,
                       (int)rep.yaw_deg, (int)rep.radius_mm, rep.mode,
                       rep.e_min, rep.e_max, rep.gc_min, rep.gc_max,
                       rep.g_min, rep.g_max, rep.lost, rep.why);
            }
        }

        if (out.brake)
        {
            l = 0.0f; r = 0.0f;
        }
        else
        {
            l = out.left_mmps;
            r = out.right_mmps;
        }

        plant_step(l, r);

        dev = track_dist(p_x, p_y);
        if (tick > 100 && dev > maxdev) { maxdev = dev; }

        if (f)
        {
            fprintf(f, "%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.1f,%u,%.1f,%.1f,%.1f\n",
                    tick * DT, p_x, p_y, p_th * 57.29578f, p_yaw,
                    (double)Path_GetLineError(), p_dist, bits,
                    (double)Path_GetLineTheta(), l, r);
        }

        if (!Path_IsRunning())
        {
            done_ticks++;
            if (done_ticks > 40) { break; }
        }
    }

    if (f) { fclose(f); }

    *out_maxdev = maxdev;
    *out_time = tick * DT;
    *out_yaw = p_yaw;

    return (tick < tick_max) ? 1 : 0;
}

int main(void)
{
    struct
    {
        const char *name;
        int mode;
        float base;
        float y0;
        float th0;
        int gray;
        const char *csv;
        int drop_from;
        int drop_to;
        float r;
    } cases[] =
    {
        { "Q2 600 centred  R=400",        2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 wider arc R=430",       2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 430.0f },
        { "Q2 600 tight arc R=370",       2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 370.0f },
        { "Q2 450 wider arc R=450",       2, 450.0f,   0.0f, 0.0f, 1, 0, 0, 0, 450.0f },
        { "Q2 600 10 mm",                 2, 600.0f,  10.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 20 mm",                 2, 600.0f,  20.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 30 mm (lost at start)", 2, 600.0f,  30.0f, 0.0f, 1, "traj_q2_30mm.csv", 0, 0, 400.0f },
        { "Q2 600 3 deg",                 2, 600.0f,   0.0f, 3.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 10mm/3deg",             2, 600.0f,  10.0f, 3.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 link drop 2..4s",       2, 600.0f,   0.0f, 0.0f, 1, 0, 400, 800, 400.0f },
        { "Q2 600 NO gray (fallback)",    2, 600.0f,   0.0f, 0.0f, 0, 0, 0, 0, 400.0f },
        { "Q2 450 centred  R=400",        2, 450.0f,   0.0f, 0.0f, 1, "traj_q2_450.csv", 0, 0, 400.0f },
        { "Q2 900 centred  R=400",        2, 900.0f,   0.0f, 0.0f, 1, 0, 0, 0, 400.0f }
    };
    unsigned i;

    printf("path.c host simulation\n");
    printf("%-32s %-8s %6s %8s %8s\n", "case", "result", "time", "maxdev", "yaw");

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        float dev, tm, yaw;
        int ok;

        s_gray_online = cases[i].gray;
        s_imu_ready = 1;
        g_R = cases[i].r;

        ok = run(cases[i].mode, cases[i].base, cases[i].y0, cases[i].th0,
                 cases[i].csv, &dev, &tm, &yaw,
                 cases[i].drop_from, cases[i].drop_to);

        printf("%-32s %-8s %5.2fs %7.1fmm %8.1f  end=(%.0f,%.0f)\n",
               cases[i].name, ok ? "DONE" : "TIMEOUT", tm, dev, yaw, p_x, p_y);
    }
    return 0;
}


