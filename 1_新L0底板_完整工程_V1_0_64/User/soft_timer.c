
#include "soft_timer.h"



//#0 ****************************   变量定义   ***************************************************************
static volatile soft_timer_t stSoftTimer[SOFT_TIMER_MAX];

//#1 ****************************   功能实现   ***************************************************************
/**
 * @brief 初始化软件定时器
 */
void soft_timer_init(void)
{
    memset((void*)&stSoftTimer, 0, sizeof(stSoftTimer));
}

/**
 * @brief 创建一个软件定时器
 * @param callback: 回调函数
 * @param ms: 定时时间(ms)
 * @param type: 回调函数执行类型
 */
void soft_timer_create(void (*callback)(void), uint16_t ms, soft_timer_cb_e type)
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    ms /= SOFT_TIMER_TICK_1MS;
    if ((callback == NULL) || (ms == 0)) return;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (!pstSoftTimer->create)
        {
            pstSoftTimer->delay     = ms;
            pstSoftTimer->delayAuto = ms;
            pstSoftTimer->callback  = callback;
            pstSoftTimer->type      = type;
            pstSoftTimer->end       = 0;
            pstSoftTimer->create    = 1;
            pstSoftTimer->start     = 1;
            return;
        }
    }
}

/**
 * @brief 删除一个软件定时器
 * @param callback: 回调函数
 */
void soft_timer_delete(void (*callback)(void))
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (pstSoftTimer->create)
        {
            if (callback == pstSoftTimer->callback)
            {
                pstSoftTimer->start = 0;
                memset((void*)pstSoftTimer, 0, sizeof(soft_timer_t));
                return;
            }
        }
    }
}

/**
 * @brief 重启动软件定时器
 * @param callback: 回调函数
 */
void soft_timer_restart(void (*callback)(void))
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (pstSoftTimer->create)
        {
            if (callback == pstSoftTimer->callback)
            {
                pstSoftTimer->start = 0;
                pstSoftTimer->delay = pstSoftTimer->delayAuto;
                pstSoftTimer->end   = 0;
                pstSoftTimer->start = 1;
                return;
            }
        }
    }
}

/**
 * @brief 暂停软件定时器
 * @param callback: 回调函数
 */
void soft_timer_suspend(void (*callback)(void))
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (pstSoftTimer->create)
        {
            if (callback == pstSoftTimer->callback)
            {
                pstSoftTimer->start = 0;
                pstSoftTimer->end   = 0;
                pstSoftTimer->delay = 0;
                return;
            }
        }
    }
}

/**
 * @brief 软件定时器扫描(决定一个时间片的时长)
 */
void soft_timer_tick(void)
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (pstSoftTimer->start)
        {
            if (!(--pstSoftTimer->delay))
            {
                pstSoftTimer->delay = pstSoftTimer->delayAuto;
                if (pstSoftTimer->type == SOFT_TIMER_CB_MAIN) {
                    pstSoftTimer->end = 1;
                } else {
                    pstSoftTimer->callback();
                }
            }
        }
    }
}

/**
 * @brief 回调函数执行
 * @note  主函数调用(异步执行回调函数)
 */
void soft_timer_loop_async(void)
{
    uint8_t i = 0;
    volatile soft_timer_t *pstSoftTimer = NULL;

    for (i = 0; i < SOFT_TIMER_MAX; i++)
    {
        pstSoftTimer = &stSoftTimer[i];
        if (pstSoftTimer->start && pstSoftTimer->end)
        {
            pstSoftTimer->end = 0;
            pstSoftTimer->callback();
        }
    }
}
