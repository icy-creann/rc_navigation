// 注：下面写的1，2，3，4 对应 Data_To_NUCNUC 结构体中的1234
// int16: 剩余发子弹量 + 中心增益区情况（占最后一位） ------ 1
// int16：当前阶段剩余时间 + 当前阶段（占最后三位）--------- 2
// int8：自身血量除以10 ---------------------------------- 3
// int8：自身血量上限除以10 
// int8：红方总血量除以10 -------------------------------- 4
// int8：蓝方总血量除以10 

#ifndef SERIAL_ENCODER2_H
#define SERIAL_ENCODER2_H

#include <stdint.h>
#include "interface_pkg/match_info.h"

// int16 bullet
// int16 middleState
// int16 resT 
// int16 stage
// int16 HPself_div10
// int16 HPselfmax_div10 
// int16 HPred_div10
// int16 HPblue_div10 

typedef struct Match_Info
{
    int bullet;
    int middleState;

    int stage;
    int resT;

    int HPself_div10;
    int HPselfmax_div10;

    int HPred_div10;
    int HPblue_div10;
} Match_Info;

typedef struct Data_To_NUC
{

    // 2 bytes --- 1
    uint16_t bullet: 15;     // 子弹数量  ------------------bullet
    uint16_t middleState: 1;      // 0 未占领；1 已占领  --------middleState，原名：center
    
    // 2 bytes --- 2
    uint16_t resT: 13;        // 剩余时间，单位秒 -----------resT 
    uint16_t stage: 3;      // 0 未开始；1 准备；2 自检；3 倒计时； 4 对战中； 5 比赛结束 ----------------------------------------stage

    // 2 bytes --- 3
    uint8_t HPself_div10;          // 自身血量/10 -------------------HPself_div10，原名：myHp
    uint8_t HPselfmax_div10 ;        // 自身上限血量/10 ----------------HPselfmax_div10 ， 原名：myupHp

    // 2 bytes --- 4
    uint8_t HPred_div10;         // 红方总血量/10  -------------------HPred_div10，原名：rHp
    uint8_t HPblue_div10;        // 蓝方总血量/10 ---------------------HPblue_div10，原名：bHp

} Data_To_NUCNUC;


void decode_upward_serial(uint8_t input[8], interface_pkg::match_info *output){
    Data_To_NUC data;
    memcpy(&data,input,8);
    output->bullet = data.bullet;
    output->middleState = data.middleState;

    output->resT = data.resT;
    output->stage = data.stage;

    output->HPself_div10 = data.HPself_div10;
    output->HPselfmax_div10 = data.HPselfmax_div10;

    output->HPred_div10 = data.HPred_div10;
    output->HPblue_div10 = data.HPblue_div10;
}
#endif