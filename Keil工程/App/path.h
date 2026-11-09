#ifndef __PATH_H
#define __PATH_H

#include "stm32f10x.h"

typedef enum
{
    PATH_IDLE = 0,
    PATH_RUNNING,
    PATH_DONE
} PathState;

typedef enum
{
    SEG_STOP = 0,
    SEG_STRAIGHT,   /* MPU heading hold */
    SEG_ARC,        /* dead-reckoned arc: yaw schedule + grayscale trim */
    SEG_LINE,       /* grayscale line-following straight (Q2) */
    SEG_ARC_LINE,   /* grayscale line-following arc (Q2) */
    SEG_TURN
} SegType;

typedef enum
{
    PATH_EVENT_NONE = 0,
    PATH_EVENT_START,
    PATH_EVENT_POINT,
    PATH_EVENT_STOP
} PathEvent;

typedef struct
{
    SegType type;
    float   length;       /* mm for straight/arc, degree for turn */
    float   speed_ratio;  /* multiplied by the UI base speed */
    int8_t  arc_dir;      /* +1 left arc, -1 right arc */
    uint8_t announce;     /* beep at the end of this segment */
} Segment;

typedef struct
{
    float   distance_mm;
    float   yaw_deg;
    float   gyro_z_dps;
    uint8_t gray_bits;
    float   dt;
    float   base_speed_mmps;
} PathInput;

/* One-line summary of the last arc, printed after the run so the serial does
 * not have to be watched live. */
typedef struct
{
    int16_t  dist_mm;
    int16_t  yaw_deg;
    int16_t  radius_mm;
    int16_t  e_min;
    int16_t  e_max;
    int8_t   gc_min;
    int8_t   gc_max;
    uint8_t  g_min;
    uint8_t  g_max;
    uint16_t lost;
    uint8_t  mode;        /* 1 = grayscale line following, 0 = schedule */
    uint8_t  why;         /* why the arc ended: 1 yaw, 2 line lost, 3 distance */
} ArcReport;
typedef struct
{
    float     left_mmps;
    float     right_mmps;
    uint8_t   brake;
    PathEvent event;
} PathOutput;

void Path_Init(void);
void Path_Start(uint8_t mode);      /* mode = 1..4 */
void Path_Stop(void);
uint8_t Path_IsRunning(void);
uint8_t Path_GetMode(void);
uint8_t Path_GetSegment(void);
void Path_Update(const PathInput *in, PathOutput *out);

/* Debug/tuning helpers */
float Path_GetLineError(void);   /* normalized gray error, -1..+1 */
float Path_GetYawRef(void);      /* last arc heading reference, degrees */
float Path_GetGrayCorr(void);    /* grayscale heading correction, degrees */
float Path_GetArcRadius(void);   /* measured arc radius, mm */
float Path_GetLineTheta(void);   /* line-following heading offset, degrees */
float Path_GetLineRate(void);    /* line-following rate command, deg/s */
uint8_t Path_GetArcMode(void);   /* 1 = line following, 0 = schedule */
uint8_t Path_GetArcWhy(void);    /* why the last arc ended: 1 yaw, 2 lost, 3 dist */
uint8_t Path_TakeArcReport(ArcReport *out);  /* 1 = a new report was ready */

#endif
