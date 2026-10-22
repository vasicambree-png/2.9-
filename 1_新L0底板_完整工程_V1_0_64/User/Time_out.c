#include "Time_out.h"
/********************** 使用示例 **********************/
// 定义控制器实例
TimeoutController_t B_ctrl;

/* 超时检测初始化 */
void Timeout_Init(TimeoutController_t* ctrl, void (*callback)(void)) {
    ctrl->state = TIMEOUT_IDLE;

    ctrl->timeout_callback = callback;
}
 
/* 启动超时检测 */
void Timeout_Start(TimeoutController_t* ctrl,uint32_t time) 
{
    ctrl->start_tick = HAL_GetTick();  // 使用HAL库获取当前时间
    ctrl->state = TIMEOUT_RUNNING;
	ctrl->timeout_ms=time;
}
 
/* 停止超时检测 */
void Timeout_Stop(TimeoutController_t* ctrl) {
    ctrl->state = TIMEOUT_IDLE;
}
 
/* 状态机更新函数（需在主循环中周期调用） */
void Timeout_Update(TimeoutController_t* ctrl) {
    if (ctrl->state != TIMEOUT_IDLE) {
        // 计算已运行时间
        uint32_t elapsed = HAL_GetTick() - ctrl->start_tick;
        
        if (elapsed >= ctrl->timeout_ms) 
				{
            ctrl->state = TIMEOUT_TRIGGERED;
            if (ctrl->timeout_callback != NULL)
						{
                ctrl->timeout_callback();  // 执行超时回调
            }
        }
    }
}




