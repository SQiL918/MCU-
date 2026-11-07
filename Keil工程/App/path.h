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
    SEG_STRAIGHT,
    SEG_ARC,
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

#endif
