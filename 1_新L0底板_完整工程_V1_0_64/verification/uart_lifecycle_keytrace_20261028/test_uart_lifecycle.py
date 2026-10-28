from pathlib import Path
import hashlib,json,re,runpy,subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent
BASE=runpy.run_path(str(HERE/'uart_hal_fixture.py'))
OUT=HERE/'uart-tests'

def main():
    OUT.mkdir(exist_ok=True)
    (OUT/'results.json').write_text(json.dumps(dict(compile_exit=None,test_exit=None,hardware_verified=False)),encoding='utf-8')
    pre=BASE['PRE']
    pre=pre.replace('uint32_t CR1,CR3,ISR,BRR;', 'uint32_t CR1,CR2,CR3,ISR,BRR;')
    pre=pre.replace('uint32_t CR,CCIPR,APB2ENR;', 'uint32_t CR,CCIPR,APB2ENR,CSR;')
    pre=pre.replace('uint32_t ErrorCode,gState,RxState,ReceptionType,RxEventType,Lock;}', 'void (*RxISR)(void *); uint32_t ErrorCode,gState,RxState,ReceptionType,RxEventType,Lock;}')
    pre=pre.replace('(void)h;(void)p;(void)n;rx_arm_calls++;return HAL_OK;', '(void)p;(void)n;rx_arm_calls++; if (arm_fail) return HAL_ERROR; h->RxISR=dummy_rx;h->RxState=HAL_UART_STATE_BUSY_RX; h->Instance->CR1|=USART_CR1_RXNEIE; return HAL_OK;')
    insert=r"""
#define USART_CR1_TCIE (1U<<6)
#define USART_ISR_RXNE (1U<<5)
#define HAL_UART_STATE_BUSY_RX 3U
#define RCC_APB2ENR_USART1EN (1U<<14)
#define RCC_CSR_PORRSTF (1U<<27)
#define RCC_CSR_SFTRSTF (1U<<28)
#define RCC_CSR_IWDGRSTF (1U<<29)
#define UART_CLEAR_NEF (1U<<2)
#define UART_CLEAR_PEF (1U<<0)
#define UART_CLEAR_FEF (1U<<1)
#define UART_CLEAR_IDLEF (1U<<4)
#define UART_CLEAR_TCF (1U<<6)
#define UART_RXDATA_FLUSH_REQUEST 8U
#define USART1_IRQn 27U
#define CLEAR_BIT(reg,bits) ((reg)&=~(bits))
#define __DMB() ((void)0)
static unsigned irq_enabled,irq_pending,irq_calls,irq_context,arm_fail,init_observed_clean;
static uint32_t mock_primask;
static uint32_t __get_PRIMASK(void){return mock_primask;}
static void __disable_irq(void){mock_primask=1U;}
static void __set_PRIMASK(uint32_t v){mock_primask=v;}
static void dummy_rx(void *p){(void)p;}
static uint32_t __get_IPSR(void){return irq_context;}
static void HAL_NVIC_DisableIRQ(unsigned irq){(void)irq;irq_enabled=0;}
static void HAL_NVIC_ClearPendingIRQ(unsigned irq){(void)irq;irq_pending=0;}
static void HAL_NVIC_EnableIRQ(unsigned irq){(void)irq; if(huart1.RxISR==NULL || huart1.RxState!=HAL_UART_STATE_BUSY_RX)abort();irq_enabled=1;}
static void HAL_UART_IRQHandler(UART_HandleTypeDef *h){(void)h;irq_calls++;}
static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h){h->Instance->CR1&=~USART_CR1_RXNEIE;h->RxISR=NULL;h->RxState=HAL_UART_STATE_READY;return HAL_OK;}
#define __HAL_RCC_USART1_CLK_ENABLE() (RCC->APB2ENR|=RCC_APB2ENR_USART1EN)
#define __HAL_UART_SEND_REQ(h,f) ((void)(f),(h)->Instance->ISR&=~USART_ISR_RXNE)
"""
    pos=pre.index('static HAL_StatusTypeDef HAL_UART_Receive_IT(')
    pre=pre[:pos]+insert+pre[pos:]
    pre += '\nvoid UART1_DeInitSafe(void);\n'
    u=BASE['clean']((ROOT/'Core/Src/usart.c').read_bytes().decode('gb18030'))
    h=BASE['clean']((ROOT/'Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c').read_text())
    app=u[u.index('#define UART1_INIT_ACK_TIMEOUT_MS'):u.index('void MX_USART2_UART_Init(void)')]
    fn=BASE['function']
    wait=fn(h,'UART_WaitOnFlagUntilTimeout').replace('HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout','static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout',1)
    idle=fn(h,'UART_CheckIdleState')
    rx='\n'.join(fn(u,n) for n in ['uart1_rx_arm','sensor_rx_start','sensor_rx_ensure_armed'])
    stubs=r"""
static HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h){
 if(irq_enabled || h->Instance->CR1 || h->Instance->CR2 || h->Instance->CR3 || irq_pending)abort();
 init_observed_clean++;
 h->Instance->CR1=USART_CR1_UE|UART_MODE_TX_RX;
 h->Instance->BRR=139U;
 h->Instance->ISR=(init_mode==0U)?(USART_ISR_TEACK|USART_ISR_REACK):((init_mode==2U)?USART_ISR_TEACK:0U);
 h->RxISR=NULL;
 if(init_mode==3U)return HAL_ERROR;
 return UART_CheckIdleState(h);
}
static HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *h){
 if(irq_enabled || !(RCC->APB2ENR&RCC_APB2ENR_USART1EN) || h->Instance->CR1 || h->Instance->CR2 || h->Instance->CR3)abort();
 deinit_calls++;memset(h->Instance,0,sizeof(*h->Instance));h->gState=HAL_UART_STATE_RESET;h->RxState=HAL_UART_STATE_RESET;
 RCC->APB2ENR&=~RCC_APB2ENR_USART1EN;return HAL_OK;
}
"""
    tests=r"""
#undef printf
static void check(int ok,const char *why){if(!ok){fprintf(stderr,"FAIL %s\n",why);exit(9);}}
static void reset_case(void){
 memset(&uart1_regs,0,sizeof(uart1_regs));memset(&uart2_regs,0,sizeof(uart2_regs));
 memset(&huart1,0,sizeof(huart1));memset(&huart2,0,sizeof(huart2));huart2.Instance=USART2;
 memset(&s_uart1_init_diag,0,sizeof(s_uart1_init_diag));memset((void*)&s_uart1_retained,0,sizeof(s_uart1_retained));
 rcc_regs.CR=0x1122U;rcc_regs.CCIPR=0x3344U;rcc_regs.APB2ENR=0x5566U;rcc_regs.CSR=0;
 tick_mode=1;tick_ms=100;Rt_tick=600;flag_reads=flag_after=0;init_mode=0;
 deinit_calls=rx_arm_calls=end_rx_calls=0;s_uart1_init_active=s_uart1_init_ready=0;
 log_used=0;log_text[0]=0;s_fifo_head=s_fifo_tail=s_line_len=7;s_rearm=0;
 irq_enabled=irq_pending=irq_calls=irq_context=arm_fail=init_observed_clean=0;
 mock_primask=0;
 s_uart1_irq_count=s_uart1_orphan_count=0;
}
int main(void){unsigned i;size_t before;
 reset_case();mock_primask=1;uart1_checkpoint(U1D_INIT_ENTER);check(mock_primask==1,"checkpoint preserves masked caller");
 mock_primask=0;uart1_checkpoint(U1D_INIT_CLEAN);check(!mock_primask,"checkpoint restores unmasked caller");
 reset_case();UART1_DeInitSafe();check(huart1.Instance==USART1 && deinit_calls==1,"safe close before first init binds USART1");
 reset_case();uart1_regs.CR1=0xFFFFFFFFU;uart1_regs.CR2=0xFFFFFFFFU;uart1_regs.CR3=0xFFFFFFFFU;
 uart1_regs.ISR=USART_ISR_RXNE|UART_CLEAR_OREF;irq_pending=irq_enabled=1;
 MX_USART1_UART_Init();check(UART1_InitReady() && init_observed_clean==1 && !irq_enabled,"stale state cleaned before init, IRQ still masked");
 sensor_rx_start();check(irq_enabled && huart1.RxISR && huart1.RxState==HAL_UART_STATE_BUSY_RX,"IRQ enabled only after real RX arm");
 check(!s_fifo_head && !s_fifo_tail && !s_line_len,"new session software buffers reset");
 uart1_regs.ISR|=USART_ISR_RXNE|UART_CLEAR_OREF;irq_pending=1;sensor_rx_start();
 check(!(uart1_regs.ISR&(USART_ISR_RXNE|UART_CLEAR_OREF)) && !irq_pending && irq_enabled,"restart discards stale hardware RX/ORE and pending");
 UART1_DeInitSafe();check(!UART1_InitReady() && !irq_enabled && !irq_pending && deinit_calls==1,"clock-valid shutdown closes IRQ and HAL once");
 check(!(RCC->APB2ENR&RCC_APB2ENR_USART1EN),"clock disabled after cleanup");
 puts("PASS production init/shutdown/RX-arm lifecycle and stale RXNE/ORE cleanup");

 for(i=0;i<50;i++){
  MX_USART1_UART_Init();sensor_rx_start();UART1_DeInitSafe();
 }
 check(deinit_calls==51 && init_observed_clean==51,"repeat lifecycle never revives stale interrupt");
 puts("PASS 50 repeated production init/arm/close cycles");

 reset_case();MX_USART1_UART_Init();sensor_rx_start();huart1.RxISR=NULL;
 uart1_regs.ISR|=USART_ISR_RXNE;irq_context=43;before=log_used;
 UART1_IRQHandlerSafe();check(!irq_enabled && !irq_pending && !irq_calls && s_uart1_orphan_count==1,"orphan RX interrupt bounded without HAL null handler");
 check(s_uart1_retained.word[1]==U1D_ORPHAN_IRQ && (s_uart1_retained.word[4]&USART_ISR_RXNE),"orphan evidence captured before flags cleared");
 check(s_uart1_retained.word[12]==43 && log_used==before,"ISR context retained without printf");
 irq_context=0;sensor_rx_ensure_armed();check(irq_enabled && huart1.RxISR && s_rearm==1,"main loop safely rearms after orphan");
 UART1_IRQHandlerSafe();check(irq_calls==1,"healthy IRQ delegates to HAL");
 puts("PASS production orphan IRQ guard, no ISR logging, and main-loop recovery");

 reset_case();MX_USART1_UART_Init();arm_fail=1;sensor_rx_start();check(!irq_enabled,"failed Receive_IT keeps IRQ masked");
 arm_fail=0;sensor_rx_ensure_armed();check(irq_enabled,"failed arm can recover in main loop");
 reset_case();init_mode=1;MX_USART1_UART_Init();check(!UART1_InitReady() && deinit_calls==1 && !irq_enabled,"failed init closes session");
 check(s_uart1_init_diag.stage==1 && s_uart1_init_diag.reason==1,"TEACK bounded by HAL clock");
 reset_case();init_mode=2;MX_USART1_UART_Init();check(s_uart1_init_diag.stage==2 && s_uart1_init_diag.reason==1,"REACK bounded");
 reset_case();init_mode=1;tick_mode=2;MX_USART1_UART_Init();check(s_uart1_init_diag.reason==2,"LPTIM fallback");
 reset_case();init_mode=1;tick_mode=0;MX_USART1_UART_Init();check(s_uart1_init_diag.reason==3 && s_uart1_init_diag.polls==20000,"dual frozen clocks finite poll budget");
 reset_case();init_mode=1;tick_ms=0xFFFFFFF8U;MX_USART1_UART_Init();check(s_uart1_init_diag.reason==1,"tick wrap");
 puts("PASS init/arm failures and original TEACK/REACK/frozen-clock/wrap guards");

 reset_case();huart1.Instance=USART1;uart1_regs.ISR=0xA5;uart1_checkpoint(U1D_WAIT_TE);RCC->CSR=RCC_CSR_IWDGRSTF;
 UART1_DiagBootReport();check(strstr(log_text,"[U1PRE]") && strstr(log_text,"stage=5"),"IWDG prints validated pre-reset checkpoint");
 check(s_uart1_retained.word[1]==U1D_BOOT,"old checkpoint consumed at boot");
 reset_case();uart1_checkpoint(U1D_RX_READY);s_uart1_retained.word[4]^=1U;RCC->CSR=RCC_CSR_IWDGRSTF;UART1_DiagBootReport();check(!log_used,"corrupt record not reported");
 reset_case();uart1_checkpoint(U1D_RX_READY);RCC->CSR=RCC_CSR_IWDGRSTF|RCC_CSR_PORRSTF;UART1_DiagBootReport();check(!log_used,"POR overrides old IWDG flag");
 reset_case();uart1_checkpoint(U1D_RX_READY);RCC->CSR=RCC_CSR_SFTRSTF;UART1_DiagBootReport();check(strstr(log_text,"[U1PRE]")!=NULL,"software reset retains report");
 reset_case();uart1_checkpoint(U1D_RX_READY);RCC->CSR=1U<<26;UART1_DiagBootReport();check(!log_used,"pin-only reset not promoted to retained fault");
 reset_case();UART1_DiagHardFault();check(s_uart1_retained.word[1]==U1D_HARDFAULT && !log_used,"hardfault checkpoint is silent");
 check(sizeof(s_uart1_retained)==64,"retained size exact");
 puts("PASS 64-byte retained record checksum, reset-cause gating, corruption/POR rejection and silent hardfault capture");
 reset_case();s_uart1_init_active=1;tick_ms=1000;s_uart1_init_diag.start_ms=0;
 check(!HAL_UART_InitWaitExpired(&huart2,USART_ISR_TEACK),"USART2 never uses USART1 hook");
 puts("PASS USART2 hook scope; real ARM UNINIT/map and hardware retention require target build/hardware");
 return 0;}
"""
    source=pre+app+'\n'+wait+'\n'+idle+'\n'+rx+'\n'+stubs+'\n'+tests
    (OUT/'harness.c').write_text(source,encoding='utf-8')
    cmd=f'@echo off\ncall "{BASE["VCVARS"]}" >nul\nif errorlevel 1 exit /b %errorlevel%\ncl /nologo /W4 /WX /utf-8 /std:c11 /Od /Fe:harness.exe /Fo:harness.obj harness.c\nexit /b %errorlevel%\n'
    (OUT/'compile.cmd').write_text(cmd,encoding='ascii')
    c=subprocess.run(['cmd.exe','/d','/c','compile.cmd'],cwd=OUT,capture_output=True);(OUT/'compile.log').write_bytes(c.stdout+c.stderr)
    assert not c.returncode,(c.stdout+c.stderr).decode(errors='replace')
    t=subprocess.run([str(OUT/'harness.exe')],cwd=OUT,capture_output=True);(OUT/'test.log').write_bytes(t.stdout+t.stderr)
    print((t.stdout+t.stderr).decode(errors='replace'),end='');assert not t.returncode,t.returncode
    files=['Core/Src/usart.c','Core/Inc/usart.h','Core/Src/main.c','Core/Src/stm32l0xx_it.c','MDK-ARM/MX_02.uvprojx','MDK-ARM/uart_fault_noinit.sct']
    result=dict(compile_exit=c.returncode,test_exit=t.returncode,retained_bytes=64,production_functions=['MX_USART1_UART_Init','UART1_DeInitSafe','UART1_IRQHandlerSafe','uart1_checkpoint','UART1_DiagBootReport','UART1_DiagHardFault','uart1_rx_arm','sensor_rx_start','sensor_rx_ensure_armed','HAL_UART_InitWaitExpired','UART_CheckIdleState','UART_WaitOnFlagUntilTimeout'],source_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},hardware_verified=False)
    (OUT/'results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
if __name__=='__main__':main()
