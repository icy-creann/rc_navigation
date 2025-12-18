#include <stdint.h>

typedef struct toNUC_t
{
    int16_t bullet : 16; 
    int16_t middleState : 16;
    int16_t resT : 16;
    int16_t stage : 16;
    int16_t HPself_div10 : 16;
    int16_t HPselfmax_div10 : 16;
    int16_t HPred_div10 : 16;
    int16_t HPblue_div10 : 16;
} toNUC_t;

typedef struct Target
{
    float x;
    float y;
    float z;
    float t;
    int type;
    int id;
}Target;

typedef struct Pose
{
    float x;
    float y;
    float yaw;
}Pose;