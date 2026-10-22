#ifndef __TIME_OUT_H__
#define __TIME_OUT_H__

#include "main.h"


/* 超时状态机定义 */
typedef enum {
    TIMEOUT_IDLE=0,          // 空闲状态
    TIMEOUT_RUNNING,         // 运行中
	  BLE_SACN_NAME_RUNNING,   // 扫描名字(绑定)
	  BLE_SACN_DATA_RUNNING,   //  扫描数据
	  LORA_SEND_RUNNING,         //LORA 发送数据时间
    TIMEOUT_TRIGGERED        // 已触发超时
} TimeoutState_t;
 
/* 超时控制器结构体 */
typedef struct {
    TimeoutState_t state;        // 当前状态
    uint32_t timeout_ms;         // 超时时间（毫秒）
    uint32_t start_tick;         // 启动时间戳
    void (*timeout_callback)(void); // 超时回调函数
} TimeoutController_t;
void Timeout_Init(TimeoutController_t* ctrl, void (*callback)(void)) ;
void Timeout_Start(TimeoutController_t* ctrl,uint32_t time)  ;
void Timeout_Stop(TimeoutController_t* ctrl);
void Timeout_Update(TimeoutController_t* ctrl) ;
#endif
