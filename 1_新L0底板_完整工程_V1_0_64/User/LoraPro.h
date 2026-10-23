#ifndef _LORAPRO_H_
#define _LORAPRO_H_

#include "main.h"

#define TestAndAckLen   7

#define Upload          0
#define Forward         1

#define Sx1268_Sleep    0
#define Sx1268_CAD      1

/* ---- sensor data model (moved here from ble_data.h after BT removal) ---- */
typedef enum {
	T_None=0,
	T_MG,   /* anchor */
	T_JG,   /* roadway */
	T_WY2,  /* displacement */
	T_WY4,
	T_WY6,
	T_WY8,
	T_LF,   /* crack */
	T_QJ,   /* inclination */
	T_YL,   /* stress */
	T_YW,   /* liquid level */
	T_WZ,   /* microseism */
	T_DY,   /* geoacoustic */
	T_KK,   /* test */
	T_END,
}Data_tpye;

typedef struct {
	int        Rssi;
	Data_tpye  tpye;
	uint16_t   CH_number;
	uint16_t   CH_data;
	uint16_t   voltage;
	uint8_t    fail_cnt;
	uint32_t   write_time;
}Channel_C;

extern Channel_C Ch_senser[21];

/* ---- networking test (moved here from ui.h) ---- */
typedef enum
{
    RFNET_IDLE,
    RFNET_CYCLEINDEX,
    RFNET_SENDTESTDATA,
    RFNET_WAITACK,
    /* ★★★ V1_0_51：三个地址各 10 次都测完之后**停在这里**，不再回 IDLE。
       原来测完直接置 IDLE，屏端立刻切回"开始组网/返回"选择页，
       三个地址的结果一闪而过、根本无法统计。
       现在停在 RFNET_DONE：屏端继续显示最终结果，底部给出
       「重新组网 / 返回」两项，由用户自己决定。
       取值必须与 CH584 侧 yuying_TFT.h 的 NET_FLAG_DONE 一致（=4）。 */
    RFNET_DONE,
}CRfNetStatus;

typedef struct
{
    uint8_t Part;
    uint8_t Addr;
    uint8_t TestCnt;
    uint8_t FailureCnt;
    uint8_t SuccessCnt;
}CRfRepeater;

typedef struct
{
    CRfNetStatus Status;
    uint32_t Tick;
    uint8_t TestIndex;
    uint8_t RepeterCnt;
    CRfRepeater Repeaters[3];
}CRfNet;

/* host time node type */
typedef enum{
	Node_NONE=0,
	Node_Tail,
	Node_last_but_one,
	Node_head,
	Node_other
}Node_tpye;

typedef enum{
	Woke_None=0,
	Woke_TxRX,
	Woke_Sleep,
	Woke_Init,
}Woke_state;

typedef struct{
	uint32_t  Start_time;
	uint32_t  End_time;
	uint32_t  lock_time;

	uint32_t  work_time;
	uint32_t  sleep_time;
	uint32_t  lock_time_flag;
	int timestamp;
}Time_Node;

typedef struct{
	uint32_t  Start_time;
	uint32_t  End_time;
	uint32_t  lock_time;

	uint32_t  real_time;
	uint8_t   real_flag;
}Lora_time;

typedef enum
{
  RfInit,
  RfIdle,
  Cad,
  CadTimeOut,
  CadDetected,

  RfRxConfig,
  RfRxing,
  RfRxOver,
  RfTimeOut,
  RfTxing,
  RfTxBusy,
  RfTxOver,

  RfError,
}CRfStatus;

typedef enum
{
  Tx_None=0,
  Tx_timestamp,
  Tx_wait_timestamp,
  Tx_oneself,
	Tx_wait_oneself,
  Tx_Forwarding,
	Tx_wait_Forwarding,
	Tx_SLEEP,
	Tx_End
}L_TxStatus;

typedef struct{
	uint32_t Into;
	uint32_t out;
	uint32_t Start;
	uint32_t End;
}Time_c;

typedef enum {
	Send_None=0,
	Send_Start,
	Send_pro,
	Send_end,
}send_tpye;

typedef struct{
	uint8_t   D_addr;
	uint8_t   N_addr;
	send_tpye State;
	CRfStatus Rx_state;

	uint16_t  Tx_data_len;
	uint8_t   Tpye;
	uint8_t   Tx_buf[100];

	uint8_t   Rx_Buf[256];

	uint8_t   ForwardBuf[100];
	uint8_t   Forward_flag;
	uint8_t   Tx_data_flag;

	uint8_t   TestBuf[20];

	uint8_t  	Failure;
	uint32_t  RfRxTick;
	uint32_t  RfTick;

	int Rssi;
}lora_d;

typedef enum
{
  UpLoadIdle,
  WaitUploadAck,
  Forwarding,
  WaitForwardAck,
}CUpLoadStatus;

extern CUpLoadStatus UpLoadStatus;
void Package_data_text(void);
void LoraPro(void);
void RfTRxPro(void);
void RfWaitPro(void);
void UpLoadPro(void);
void RfDataDeal(void);
void CadPro(void);
void Lora_task(void);
/* Main-loop only: queue one completed 0x03 payload (20 channels, 80 bytes). */
uint32_t Lora_SleepCollectionDelayTicks(void);
void Lora_QueueCollectedReport(const uint8_t *channels, uint32_t planned_tick);
void Lora_CancelCollectedReport(void); /* Confirmed binding/identity change only. */
/* ★ V1_0_56：从设置页回到主页 / 绑定流程结束后，把 LoRa 收发状态机拉回已知状态（幂等） */
void Lora_HomeEntry_ReArm(void);
uint8_t RxData_check(void);
void Timer_task(void);
void Lora_reve(void);
void RfNetPro(void);
uint8_t SetCheckSum(uint8_t *buf,uint16_t len);
void Time_check_task(void);
void work_Time_check(void);
void lora_work_state(void);
#endif
