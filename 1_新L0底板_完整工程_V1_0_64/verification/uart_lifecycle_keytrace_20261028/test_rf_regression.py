"""Compile production recovery/diagnostic functions with hardware mocks.

These tests establish control flow and bounds, never physical RF delivery.
All generated files remain under rf-tests/ in the actual project.
"""
from pathlib import Path
import hashlib
import json
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = HERE / 'rf-tests'
VCVARS = Path(r'D:\visual studio\visual studio2026\VC\Auxiliary\Build\vcvars64.bat')


def clean(s):
    pattern = r'"(?:\\.|[^"\\])*"|/\*.*?\*/|//[^\n]*'
    return re.sub(pattern, lambda m: m[0] if m[0].startswith('"') else ' ', s, flags=re.S)


def function(s, name):
    s = clean(s)
    m = re.search(r'(?m)^(?:static\s+)?[A-Za-z_][\w \t*]*\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', s)
    assert m, name
    depth, end = 1, m.end()
    while depth:
        if s[end] == '{': depth += 1
        elif s[end] == '}': depth -= 1
        end += 1
    return s[m.start():end]


HEAL_PRE = r'''
#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LoraTrace.h"
#define CHECK(c,msg) do{if(!(c)){fprintf(stderr,"FAIL %s line %d\n",msg,__LINE__);exit(13);}}while(0)
enum { Tx_None=0, Tx_oneself=3, Tx_wait_oneself=4, Tx_Forwarding=5, Tx_wait_Forwarding=6 };
enum { RFNET_IDLE=0, RFNET_CYCLEINDEX=1, RFNET_DONE=4 };
enum { Sx1268_Sleep=0 };
typedef enum { NORMAL=0, SPI_READCHECK_WRONG=2 } tSX126xError;
#define LORA_HEAL_FAIL_ROUNDS 10U
#define LORA_HEAL_NO_RX_TICKS 6000U
#define LORA_HEAL_MIN_GAP_TICKS 600U
static uint32_t Rt_tick, hal_tick;
static uint8_t Lora_Tx_state, Lora_Rx_state, g_slp_keep_awake, s_force_rf_exit;
static struct {uint8_t Failure,Forward_flag;uint8_t ForwardBuf[100];uint32_t RfRxTick;} Lo_data;
static struct {uint8_t Setting_flag,lcd_open;} UI;
static struct {uint8_t On_Off_flag,Host_number,partition,transmit_number[3];} Setup;
static struct {uint8_t Status;} RfNet;
static uint8_t s_all_fail_cnt;
static uint32_t s_last_valid_ms;
static uint32_t s_heal_watch_tick, s_heal_last_action;
static uint32_t s_heal_failed_total, s_heal_failed_mark;
static uint8_t s_heal_watch_active, s_heal_action_seen, s_heal_reason;
static uint32_t s_report_seq, s_report_tx_seq, s_self_retry_tick;
static uint8_t s_report_pending, s_self_retry_wait, s_report_data[80], s_report_tx_data[80];
static uint8_t s_rx_state_last;
static uint32_t s_rx_state_tick,s_rx_state_hms;
static uint8_t s_rf_confirm_captured,send_adrr_cnt,s_same_addr_retry,s_cad_busy_cnt,send_flag;
static uint32_t TickLeav,Leav_tick,s_tx_wait_tick,g_lora_rx_stuck_cnt;
#define LORA_RX_STUCK_TICKS 30U
#define LORA_STUCK_MS_AWAKE 60000U
#define LORA_TX_CONFIRM_TICKS 8U
static unsigned init_calls,sleep_calls,trace_calls,radio_calls;
static unsigned operation_count,operation_order[64];
static uint8_t trace_events[16],radio_events[16];
static uint16_t trace_details[16];
static tSX126xError init_result;
static uint32_t HAL_GetTick(void){return hal_tick;}
static tSX126xError SX126x_Lora_init(void){init_calls++;return init_result;}
static void SX126X_WOR_Execute(uint8_t mode){CHECK(mode==Sx1268_Sleep,"only existing sleep after recovery");sleep_calls++;operation_order[operation_count++]=2;Lora_Rx_state=0;}
static void link_radio(uint8_t reason,uint32_t seq){CHECK(seq==(reason==RF_CONFIRM_TIMEOUT?s_report_tx_seq:s_report_seq),"snapshot sequence");CHECK(radio_calls<16,"radio log bound");radio_events[radio_calls++]=reason;operation_order[operation_count++]=1;}
static void link_trace(uint8_t event,uint32_t seq,const uint8_t *frame,uint16_t len,uint16_t detail){
 CHECK(seq==s_report_seq && frame==NULL && len==0,"heal event parameters");
 CHECK(trace_calls<16,"trace log bound");trace_events[trace_calls]=event;trace_details[trace_calls++]=detail;
}
static uint32_t TickDiff_100ms(uint32_t tick){return Rt_tick-tick;}
static void lora_rf_housekeeping(void){}
static void lora_work_state(void){}
static void re_iwdg(void){}
static uint8_t RxData_check(void){return 0;}
'''

HEAL_MAIN = r'''
static void reset(void){
 memset(&Lo_data,0,sizeof(Lo_data));memset(&UI,0,sizeof(UI));memset(&Setup,0,sizeof(Setup));memset(&RfNet,0,sizeof(RfNet));
 Setup.On_Off_flag=1;Setup.Host_number=35;Setup.partition=2;Setup.transmit_number[0]=2;Setup.transmit_number[1]=5;Setup.transmit_number[2]=3;
 Rt_tick=100;hal_tick=71;Lora_Tx_state=0;Lora_Rx_state=0;g_slp_keep_awake=s_force_rf_exit=0;
 s_all_fail_cnt=0;s_last_valid_ms=42;s_heal_watch_tick=s_heal_last_action=0;
 s_heal_failed_total=s_heal_failed_mark=0;s_heal_watch_active=s_heal_action_seen=s_heal_reason=0;
 s_report_seq=19;s_report_tx_seq=0;s_report_pending=1;s_self_retry_wait=1;s_self_retry_tick=83;
 memset(s_report_data,0x35,sizeof(s_report_data));memset(s_report_tx_data,0x5A,sizeof(s_report_tx_data));memset(Lo_data.ForwardBuf,0xA5,sizeof(Lo_data.ForwardBuf));
 s_rx_state_last=7;s_rx_state_tick=3;s_rx_state_hms=4;
 init_calls=sleep_calls=trace_calls=radio_calls=operation_count=0;init_result=NORMAL;
 s_rf_confirm_captured=send_adrr_cnt=s_same_addr_retry=s_cad_busy_cnt=send_flag=0;
 TickLeav=Leav_tick=s_tx_wait_tick=g_lora_rx_stuck_cnt=0;
}
static void fail_rounds(unsigned n){unsigned i;for(i=0;i<n;i++)lora_heal_failed_round();}
static void test_triggers(void){
 unsigned i;reset();Rt_tick=100000;lora_heal_service();CHECK(init_calls==0,"quiet idle must not heal");
 fail_rounds(10);lora_heal_service();CHECK(init_calls==0,"counter alone without actual TX does not heal");
 lora_heal_activity();CHECK(s_heal_watch_tick==Rt_tick,"start watch on activity");lora_heal_service();
 CHECK(init_calls==1 && sleep_calls==1 && s_all_fail_cnt==10,"ten failures accumulate and trigger without false clear");
 CHECK(s_last_valid_ms==42 && s_heal_watch_active==1,"recovery does not fabricate valid receive");
 CHECK(trace_events[0]==LT_HEAL_BEGIN && trace_details[0]==1 && trace_events[1]==LT_HEAL_END && trace_details[1]==0,"explicit attempt/result events");
 CHECK(radio_calls==2 && radio_events[0]==RF_HEAL_BEFORE && radio_events[1]==RF_HEAL_AFTER,"pre/post RF snapshot");
 lora_heal_service();CHECK(init_calls==1,"no repeated recovery without new failures/time");
 for(i=0;i<255;i++)lora_heal_failed_round();CHECK(s_all_fail_cnt==255,"continuous fail count saturates");
 CHECK(s_last_valid_ms==42,"failed rounds never refresh valid time");
 Rt_tick+=600;lora_heal_service();CHECK(init_calls==2,"ten new failed rounds after action can trigger");
 reset();lora_heal_activity();Rt_tick+=5999;lora_heal_service();CHECK(init_calls==0,"not before no-success limit");
 Rt_tick++;lora_heal_service();CHECK(init_calls==1 && trace_details[0]==2,"ten-minute activity watch triggers");
 puts("PASS actual heal triggers, quiet idle, saturation and no fabricated success");
}
static void test_gates(void){
 unsigned gate;for(gate=0;gate<15;gate++){
  reset();lora_heal_activity();fail_rounds(10);
  switch(gate){
   case 0:Lora_Tx_state=Tx_oneself;break;case 1:Lora_Tx_state=Tx_wait_oneself;break;
   case 2:Lora_Tx_state=Tx_Forwarding;break;case 3:Lora_Tx_state=Tx_wait_Forwarding;break;
   case 4:Lora_Rx_state=3;break;case 5:Lora_Rx_state=4;break;case 6:Lora_Rx_state=5;break;case 7:Lora_Rx_state=6;break;
   case 8:Lo_data.Forward_flag=1;break;case 9:g_slp_keep_awake=1;break;case 10:UI.Setting_flag=1;break;
   case 11:Setup.On_Off_flag=0;break;case 12:Setup.Host_number=0;break;case 13:s_force_rf_exit=1;break;case 14:RfNet.Status=RFNET_CYCLEINDEX;break;
  }
  lora_heal_service();CHECK(init_calls==0 && s_heal_reason==1,"unsafe state defers pending maintenance");
  Lora_Tx_state=0;Lora_Rx_state=0;Lo_data.Forward_flag=0;g_slp_keep_awake=0;UI.Setting_flag=0;Setup.On_Off_flag=1;Setup.Host_number=35;s_force_rf_exit=0;RfNet.Status=RFNET_DONE;
  lora_heal_service();CHECK(init_calls==1,"deferred maintenance runs at later safe point");
 }
 puts("PASS actual TX/ACK/RX/forward/collection/menu/power/test/force-exit gates");
}
static void test_preservation_and_failed_init(void){
 unsigned i;uint8_t routes[3],queued[80],inflight[80],forward[100];
 reset();memcpy(routes,Setup.transmit_number,3);memcpy(queued,s_report_data,80);memcpy(inflight,s_report_tx_data,80);memcpy(forward,Lo_data.ForwardBuf,100);
 Lo_data.Failure=201;init_result=SPI_READCHECK_WRONG;lora_heal_activity();fail_rounds(10);lora_heal_service();
 CHECK(trace_details[1]==SPI_READCHECK_WRONG && s_last_valid_ms==42 && Lo_data.Failure==201 && s_all_fail_cnt==10,"failed init is not success or counter clear");
 CHECK(memcmp(routes,Setup.transmit_number,3)==0 && memcmp(queued,s_report_data,80)==0 && memcmp(inflight,s_report_tx_data,80)==0 && memcmp(forward,Lo_data.ForwardBuf,100)==0,"route/payloads retained");
 CHECK(s_report_seq==19 && s_report_tx_seq==0 && s_report_pending==1 && s_self_retry_wait==1 && s_self_retry_tick==83,"queued work and original retry deadline retained");
 CHECK(s_rx_state_last==0 && s_rx_state_tick==Rt_tick && s_rx_state_hms==hal_tick,"RX guard rearmed without touching queued data");
 Rt_tick=201;lora_heal_valid();CHECK(s_last_valid_ms==201 && s_all_fail_cnt==0 && Lo_data.Failure==0 && !s_heal_watch_active && !s_heal_reason,"recognized completion resets watch");
 for(i=0;i<9;i++)lora_heal_failed_round();lora_heal_activity();lora_heal_service();CHECK(init_calls==1,"new window needs ten failures");
 puts("PASS RF failure result, 35 learned routes, snapshots, seq and retry preservation");
}
static void test_wrap_rate(void){
 reset();lora_heal_activity();fail_rounds(10);lora_heal_service();fail_rounds(10);
 Rt_tick=699;lora_heal_service();CHECK(init_calls==1,"60 second rate limit");Rt_tick=700;lora_heal_service();CHECK(init_calls==2,"rate limit expires");
 reset();Rt_tick=0xFFFFFF00U;lora_heal_activity();Rt_tick+=(uint32_t)6000U;lora_heal_service();CHECK(init_calls==1 && trace_details[0]==2,"watch wraps unsigned ticks");
 reset();s_heal_failed_total=0xFFFFFFFBU;s_heal_failed_mark=s_heal_failed_total;lora_heal_activity();fail_rounds(10);lora_heal_service();CHECK(init_calls==1,"failure generation wraps unsigned");
 puts("PASS unsigned tick/generation wrap and recovery rate limit");
}
static void confirmation_state(void){
 reset();Rt_tick=100;Lora_Rx_state=s_rx_state_last=3;s_rx_state_tick=100;s_rx_state_hms=hal_tick;
 Lora_Tx_state=Tx_wait_oneself;Lo_data.RfRxTick=90;s_report_tx_seq=8;
}
static void test_confirmation_capture(void){
 confirmation_state();Lora_reve();
 CHECK(radio_calls==1&&sleep_calls==1&&radio_events[0]==RF_CONFIRM_TIMEOUT,"first confirmation close captured");
 CHECK(operation_count==2&&operation_order[0]==1&&operation_order[1]==2,"capture occurs before sleep clears IRQ");
 confirm_timeout_fallback();CHECK(radio_calls==1&&s_rf_confirm_captured==1,"send timeout fallback does not duplicate capture");
 confirmation_state();s_same_addr_retry=1;Lora_reve();CHECK(radio_calls==0&&sleep_calls==1,"other attempts do not oversample");
 confirmation_state();send_adrr_cnt=1;Lora_reve();CHECK(radio_calls==0&&sleep_calls==1,"other destination does not oversample");
 confirmation_state();Lora_Tx_state=Tx_wait_Forwarding;Lora_reve();CHECK(radio_calls==1&&sleep_calls==1,"forward confirmation close captured");
 confirmation_state();UI.lcd_open=1;Lora_reve();CHECK(!radio_calls&&!sleep_calls,"bright RX window remains open until existing bound");
 confirm_timeout_fallback();confirm_timeout_fallback();CHECK(radio_calls==1&&sleep_calls==0,"fallback captures once while radio still open");
 UI.lcd_open=0;Lora_reve();CHECK(radio_calls==1&&sleep_calls==1,"later RX close does not duplicate fallback");
 puts("PASS real RX-close capture precedes IRQ-clearing sleep, first-attempt sampling and timeout deduplication");
}
int main(void){test_triggers();test_gates();test_preservation_and_failed_init();test_wrap_rate();test_confirmation_capture();return 0;}
'''

DIAG_PRE = r'''
#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "RadioDiag.h"
#define CHECK(c,msg) do{if(!(c)){fprintf(stderr,"FAIL %s line %d\n",msg,__LINE__);exit(13);}}while(0)
enum { MODE_SLEEP=0, MODE_STDBY_RC=1, MODE_STDBY_XOSC=2, MODE_FS=3, MODE_TX=4, MODE_RX=5, MODE_RX_DC=6, MODE_CAD=7, MODE_UNKNOWN=255 };
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_4 16U
#define GPIO_PIN_5 32U
#define GPIOB ((void*)1)
#define GPIOC ((void*)2)
static uint8_t busy,OperatingMode;
#define LORA_BUSY_VAL busy
static uint32_t Rt_tick;
static uint16_t pin_b,pin_c;
static struct {uint32_t IMR,PR;} exti;
#define EXTI (&exti)
volatile RadioDiagIrqCounters g_radio_diag_irq;
volatile uint32_t g_radio_spi_fail_count;
static uint16_t SX126x_BusyTimeoutCnt,SX126x_StandbyTimeoutCnt,Lora_TxWaitTimeoutCnt;
typedef struct {uint8_t Value;} RadioStatus_t;
typedef struct {uint16_t Value;} RadioError_t;
static unsigned calls,operations[4],fail_at,make_busy_at;
typedef uint8_t u8;
static int hspi1;
static uint8_t spi_mock_fail;
#define SX126X_SPI_TIMEOUT_MS 10U
#define HAL_OK 0
static int HAL_SPI_TransmitReceive(int *handle,uint8_t *tx,uint8_t *rx,uint16_t len,uint32_t timeout){
 CHECK(handle==&hspi1&&len==1&&timeout==10U&&*tx==0xA5,"original SPI byte parameters");
 if(spi_mock_fail)return 1;*rx=0x5A;return HAL_OK;
}
static uint32_t HAL_GPIO_ReadPin(void *port,uint16_t mask){return ((port==GPIOB?pin_b:pin_c)&mask)!=0;}
static void operation(unsigned n){operations[calls++]=n;if(fail_at==n)g_radio_spi_fail_count++;if(make_busy_at==n){busy=1;SX126x_BusyTimeoutCnt++;}}
static uint16_t SX126xGetIrqStatus(void){operation(1);return 0x0242;}
static RadioStatus_t SX126xGetStatus(void){RadioStatus_t s;operation(2);s.Value=0x54;return s;}
static RadioError_t SX126xGetDeviceErrors(void){RadioError_t e;operation(3);e.Value=0x20;return e;}
'''

DIAG_MIDDLE = r'''
static char output[32768];static size_t used;
static int capture_printf(const char *fmt,...){int n;va_list ap;va_start(ap,fmt);n=vsnprintf(output+used,sizeof(output)-used,fmt,ap);va_end(ap);if(n<0||(size_t)n>=sizeof(output)-used)exit(90);used+=(size_t)n;return n;}
#define printf capture_printf
#include "production_trace.c"
#undef printf
static void clear_output(void){memset(output,0,sizeof(output));used=0;}
static void reset_capture(void){
 OperatingMode=MODE_RX;busy=0;Rt_tick=123;pin_b=7;pin_c=48;exti.IMR=0xFFFF;exti.PR=0x31;
 g_radio_spi_fail_count=0;SX126x_BusyTimeoutCnt=3;SX126x_StandbyTimeoutCnt=4;Lora_TxWaitTimeoutCnt=5;
 g_radio_diag_irq.dio1=11;g_radio_diag_irq.dio2=12;g_radio_diag_irq.dio3=13;g_radio_diag_irq.cad_done=14;g_radio_diag_irq.cad_activity=15;g_radio_diag_irq.rx_irq=16;
 calls=fail_at=make_busy_at=0;memset(operations,0,sizeof(operations));
}
static void test_capture(void){
 RadioDiagSnapshot s;unsigned i;uint8_t modes[]={MODE_SLEEP,MODE_UNKNOWN,MODE_RX_DC};
 for(i=0;i<sizeof(modes);i++){reset_capture();OperatingMode=modes[i];SX126xCaptureDiagnostic(&s);CHECK(calls==0&&s.read_flags==0&&OperatingMode==modes[i],"never SPI/wake a sleeping/unknown/duty radio");}
 reset_capture();busy=1;SX126xCaptureDiagnostic(&s);CHECK(calls==0&&s.pins==63&&s.read_flags==0,"BUSY skip and physical GPIO pin packing");
 reset_capture();SX126xCaptureDiagnostic(&s);CHECK(calls==3&&operations[0]==1&&operations[1]==2&&operations[2]==3,"IRQ captured before status/errors");
 CHECK(s.rt==123&&s.sw_mode==MODE_RX&&s.status==0x54&&s.irq==0x0242&&s.errors==0x20&&s.read_flags==7,"actual driver fields retained");
 CHECK(s.pins==62&&s.exti_mask==0x31&&s.exti_pending==0x31&&s.busy_to==3&&s.standby_to==4&&s.tx_to==5,"pins/EXTI/timeout counters");
 CHECK(s.dio1==11&&s.dio2==12&&s.dio3==13&&s.cad_done==14&&s.cad_activity==15&&s.rx_irq==16,"ISR counters copied");
 for(i=1;i<=3;i++){reset_capture();fail_at=i;SX126xCaptureDiagnostic(&s);CHECK(s.spi_fail==1,"transport failure counted");CHECK(s.read_flags==(uint8_t)(7U&~(i==1?2U:i==2?1U:4U)),"failed read is explicitly invalid");}
 reset_capture();make_busy_at=1;SX126xCaptureDiagnostic(&s);CHECK(calls==1&&s.read_flags==0&&s.busy_to==4,"new BUSY skips remaining reads and invalidates prior read");
 reset_capture();SX126xCaptureDiagnostic(NULL);CHECK(calls==0,"NULL diagnostic target");
 reset_capture();pin_b=GPIO_PIN_0;pin_c=0;SX126xCaptureDiagnostic(&s);CHECK(s.pins==8,"DIO3 is PB0");
 reset_capture();pin_b=GPIO_PIN_1;pin_c=0;SX126xCaptureDiagnostic(&s);CHECK(s.pins==16,"SW1 is PB1");
 reset_capture();pin_b=GPIO_PIN_2;pin_c=0;SX126xCaptureDiagnostic(&s);CHECK(s.pins==32,"SW2 is PB2");
 reset_capture();pin_b=0;pin_c=GPIO_PIN_4;SX126xCaptureDiagnostic(&s);CHECK(s.pins==2,"DIO1 is PC4");
 reset_capture();pin_b=0;pin_c=GPIO_PIN_5;SX126xCaptureDiagnostic(&s);CHECK(s.pins==4,"DIO2 is PC5");
 puts("PASS real RF capture: no wake/SPI in sleep/BUSY, read order, invalidation, GPIO and IRQ counters");
}
static void test_spi_byte(void){
 reset_capture();spi_mock_fail=0;CHECK(SX126X_ReadWriteByte(0xA5)==0x5A&&g_radio_spi_fail_count==0,"healthy SPI return retained");
 spi_mock_fail=1;CHECK(SX126X_ReadWriteByte(0xA5)==0&&g_radio_spi_fail_count==1,"SPI failure returns initialized byte and increments error counter");
 CHECK(SX126X_ReadWriteByte(0xA5)==0&&g_radio_spi_fail_count==2,"every failed byte counted");
 puts("PASS actual SPI byte healthy result, bounded HAL call and initialized failure result");
}
int main(void){test_capture();test_spi_byte();return 0;}
'''


def run_case(name, code):
    case = OUT / name
    case.mkdir(parents=True, exist_ok=True)
    for f in ['LoraTrace.h', 'RadioDiag.h']:
        (case / f).write_bytes((ROOT / 'User' / f).read_bytes())
    (case / 'production_trace.c').write_bytes((ROOT / 'User/LoraTrace.c').read_bytes())
    (case / 'harness.c').write_text(code, encoding='utf-8')
    script = f'@echo off\ncall "{VCVARS}" >nul\nif errorlevel 1 exit /b %errorlevel%\ncl /nologo /W4 /WX /utf-8 /std:c11 /Od /Fe:harness.exe /Fo:harness.obj harness.c\nexit /b %errorlevel%\n'
    (case / 'compile.cmd').write_text(script, encoding='ascii')
    compiled = subprocess.run(['cmd.exe', '/d', '/c', 'compile.cmd'], cwd=case, capture_output=True)
    (case / 'compile.log').write_bytes(compiled.stdout + compiled.stderr)
    assert compiled.returncode == 0, (name, (compiled.stdout + compiled.stderr).decode(errors='replace'))
    tested = subprocess.run([str(case / 'harness.exe')], cwd=case, capture_output=True)
    (case / 'test.log').write_bytes(tested.stdout + tested.stderr)
    print(name, (tested.stdout + tested.stderr).decode(errors='replace'), sep='\n', end='')
    assert tested.returncode == 0, (name, tested.returncode)
    return {'compile_exit': compiled.returncode, 'test_exit': tested.returncode}


def main():
    OUT.mkdir(exist_ok=True)
    (OUT/'results.json').write_text(json.dumps(dict(verified=False,hardware_verified=False)),encoding='utf-8')
    lora = (ROOT / 'User/LoraPro.c').read_bytes().decode('gb18030')
    driver = (ROOT / 'User/Sx126x_Driver.c').read_bytes().decode('gb18030')
    heal_names = ['lora_heal_activity', 'lora_heal_valid', 'lora_heal_failed_round', 'lora_heal_service', 'lora_rx_window_ticks', 'Lora_reve']
    fallback = re.search(r'if\s*\(send_adrr_cnt == 0U && s_same_addr_retry == 0U && s_rf_confirm_captured == 0U\)\s*\{[^}]*\}',clean(lora))
    assert fallback
    heal = run_case('heal', HEAL_PRE + '\n'.join(function(lora, f) for f in heal_names) + '\nstatic void confirm_timeout_fallback(void){'+fallback[0]+'}\n' + HEAL_MAIN)
    diag = run_case('diag', DIAG_PRE + '\n'.join(function(driver, f) for f in ['SX126X_ReadWriteByte','radio_diag_can_read', 'radio_diag_read_ok', 'SX126xCaptureDiagnostic']) + DIAG_MIDDLE)
    assert hashlib.sha256(function(lora, 'Load_Packaging').encode()).hexdigest() == 'ce01d9343bb16f67387df4be22c81c6f81bade887e11b947e5494720c820ed15', 'Air payload packaging changed'
    assert '// boot_upload_check();' in lora
    h = (ROOT / 'Core/Inc/usart.h').read_bytes().decode('gb18030')
    for name, value, source in [('SLP_READ_PERIOD_SEC','600U',h),('LORA_PREAMBLE_NEIGHBOR','2000U',lora),('LORA_TX_CONFIRM_TICKS','8U',lora)]:
        assert re.search(r'#define\s+'+name+r'\s+'+value,source), name
    assert 'if(Lo_data.Failure>9)' not in clean(lora)
    assert 'GetNewsend_addr();' not in function(lora, 'lora_heal_service')
    assert 'GetNewsend_addr();' not in function(lora, 'Timer_task')
    assert clean(lora).count('lora_heal_valid();') == 6
    assert clean(lora).count('s_last_valid_ms = Rt_tick;') == 1
    assert 'lora_heal_service();' in function(lora,'Lora_task')
    callback = function((ROOT/'Core/Src/stm32l0xx_it.c').read_bytes().decode('gb18030'),'HAL_GPIO_EXTI_Callback')
    assert 'printf(' not in callback and 'SX126xGet' not in callback
    for field in ['dio1','dio2','dio3','cad_done','cad_activity','rx_irq']:
        assert 'g_radio_diag_irq.'+field+'++' in callback
    result = {'heal':heal,'diag':diag,'actual_Load_Packaging_unchanged':True,
        'period_preamble_ack_and_user_boot_comment_preserved':True,
        'hardware_verified':False,'source_sha256':{f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in ['User/LoraPro.c','User/LoraTrace.c','User/LoraTrace.h','User/RadioDiag.h','User/Sx126x_Driver.c','Core/Src/stm32l0xx_it.c']}}
    (OUT/'results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PASS production hook/config/ISR inventory and unchanged wire packaging')


if __name__ == '__main__': main()
