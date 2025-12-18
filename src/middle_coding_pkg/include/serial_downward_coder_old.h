#include <stdint.h>
#include <string.h>

#define INT14_MAX (0x1FFF)

/// @brief 指令结构体
typedef struct Serial_cmd
{
    float x; // -1 ~ 1
    float y; // -1 ~ 1
    float w; // -1 ~ 1

    float pitch_down; // -0.5 ~ 0.5
    float pitch_up; // -0.5 ~ 0.5

    float yaw_up; // -1 ~ 1

    int8_t shoot_up; 
    int8_t shoot_down;
    float yaw_down; // -1 ~ 1
}Serial_cmd;

/// @brief 用于转为字节数据的中间结构体
typedef struct Encoded_datas
{
    int8_t x;
    int8_t y;
    int8_t w;
    
    int8_t pitch_down;
    int16_t shoot_up: 1;
    int16_t shoot_down: 1; 
    int16_t yaw_down: 14;
    
    int8_t yaw_up;
    int8_t pitch_up;
}Encoded_datas;


#define LIMIT_INSIDE(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min)
#define PI 3.14159265358979323846f


/// @brief 编码函数
void encode_serial(Serial_cmd* input, uint8_t output[8])
{
    Encoded_datas edata;
    edata.x = LIMIT_INSIDE(MAP(input->x, -1.5, 1.5, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    edata.y = LIMIT_INSIDE(MAP(input->y, -1.5, 1.5, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    edata.w = LIMIT_INSIDE(MAP(input->w, -5, 5, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);

    edata.shoot_up=input->shoot_up ? 1 : 0;
    edata.shoot_down=input->shoot_down ? 1 : 0;
    edata.yaw_down=LIMIT_INSIDE(MAP(input->yaw_down, -1.0, 1.0, -INT14_MAX, INT14_MAX), -INT14_MAX, INT14_MAX);
    
    edata.yaw_up=LIMIT_INSIDE(MAP(input->yaw_up, -1.0, 1.0, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    
    edata.pitch_up=LIMIT_INSIDE(MAP(input->pitch_up, -0.3, 0.3, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    edata.pitch_down=LIMIT_INSIDE(MAP(input->pitch_down, -0.3, 0.3, -INT8_MAX, INT8_MAX), -INT8_MAX, INT8_MAX);
    
    memcpy(output,&edata,8);
}

#define int8_t2float(x) ((float)x/0x7F)
/// @brief 解码函数 
void decode_serial(uint8_t datas[8], Serial_cmd* output)
{
    Encoded_datas input;
    memcpy(&input,datas,8);
    
    output->x = MAP(input.x, -INT8_MAX, INT8_MAX, -1.5, 1.5);
    output->y = MAP(input.y, -INT8_MAX, INT8_MAX, -1.5, 1.5);
    output->w = MAP(input.w, -INT8_MAX, INT8_MAX, -5, 5);
    output->shoot_down = input.shoot_down;
    output->shoot_up = input.shoot_up;
    output->yaw_down = MAP(input.yaw_down, -INT14_MAX, INT14_MAX, -1.0, 1.0);
    output->yaw_up = MAP(input.yaw_up, -INT8_MAX, INT8_MAX, -1.0, 1.0);
    output->pitch_down = MAP(input.pitch_down, -INT8_MAX, INT8_MAX, -0.3, 0.3);
    output->pitch_up = MAP(input.pitch_up, -INT8_MAX, INT8_MAX, -0.3, 0.3);
}

