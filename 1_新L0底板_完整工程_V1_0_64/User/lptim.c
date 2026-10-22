#include "lptim.h"
#include "usart.h"
LPTIM_HandleTypeDef   LptimHandle;
//100ms
void Lptim_Init(void)
{
	LptimHandle.Instance = LPTIM1;
  
  LptimHandle.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	LptimHandle.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
  LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV32;  
  LptimHandle.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE; //软件触发
  LptimHandle.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  LptimHandle.Init.UpdateMode         = LPTIM_UPDATE_IMMEDIATE; //更新模式

  if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
  {
//    Error_Handler();
  }
	  // 启动100ms中断（100次计数）
  if (HAL_LPTIM_Counter_Start_IT(&LptimHandle, 100) != HAL_OK) {

  }
}
extern uint32_t Rt_tick;
uint32_t TickDiff_100ms(uint32_t Tick)
{
  return Rt_tick - Tick;
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    
  /* ## - 1 - Enable LPTIM clock ############################################ */
  __HAL_RCC_LPTIM1_CLK_ENABLE();
  
  /* ## - 4 - Enable and set LPTIM Interrupt to the highest priority ######## */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
	 
}

void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
	  __HAL_RCC_LPTIM1_CLK_DISABLE();
	  HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
}
//单位 ms
void Lptim_time_setting(uint16_t time_num)
{
	HAL_LPTIM_Counter_Stop_IT(&LptimHandle); //停止
	 // 3. 重新配置并启动
  if (HAL_LPTIM_Counter_Start_IT(&LptimHandle, time_num) != HAL_OK)//重新设置
	{
		 printf("Lptim_time_setting error");
//    Error_Handler();
  }
}