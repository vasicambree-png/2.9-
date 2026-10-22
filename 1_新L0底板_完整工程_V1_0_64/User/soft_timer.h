
#ifndef __SOFT_TIMER_H
#define __SOFT_TIMER_H
#include <stdint.h>
#include <string.h>



//#0 ****************************   宏定义(根据需要修改)   *******************************
#define SOFT_TIMER_MAX          10  /*!< 软件定时器数量 */
#define SOFT_TIMER_TICK_1MS     10  /*!< 软件定时器心跳基准(ms) */



//#1 ****************************   数据定义   *******************************************
/** 回调类型 */
typedef enum {
    SOFT_TIMER_CB_MAIN = 0, /*!< 主函数执行 */
    SOFT_TIMER_CB_IRQ,      /*!< 中断执行 */
} soft_timer_cb_e;

/** 软件定时器结构体 */
typedef struct {
    uint8_t create;         /*!< 创建标志位 */
    uint8_t start;          /*!< 运行标志位 */
    uint8_t end;            /*!< 时间到标志位 */
    uint8_t type;           /*!< 回调类型标志位 */
    uint16_t delay;         /*!< 软件定时时间片 */
    uint16_t delayAuto;     /*!< 软件定时时间片自动重装 */
    void (*callback)(void); /*!< 软件定时回调函数 */
} soft_timer_t;



//#2 ****************************   函数声明   *******************************************
void soft_timer_init(void);
void soft_timer_create(void (*callback)(void), uint16_t ms, soft_timer_cb_e type);
void soft_timer_delete(void (*callback)(void));
void soft_timer_restart(void (*callback)(void));
void soft_timer_suspend(void (*callback)(void));
void soft_timer_tick(void);
void soft_timer_loop_async(void);

#endif
