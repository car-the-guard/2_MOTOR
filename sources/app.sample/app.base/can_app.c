/*
 * can_app.c
 * Modified for: RTOS Integration (CRC/Bridge Removed)
 */

#include <sal_api.h>
#include <app_cfg.h>
#include <bsp.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include "can_config.h"
#include "can_reg.h"
#include "can.h"
#include "can_drv.h"
#include "can_porting.h"

// ★ 우리가 만든 헤더
#include "can_app.h"
#include "timestamp.h" 

/* -------------------------------------------------------------------------
   전역 변수 및 핸들 정의
   ------------------------------------------------------------------------- */
#define CAN_TX_QUEUE_SIZE      10
#define CAN_RX_QUEUE_SIZE      10
#define CAN_TX_POOL_SIZE       16
#define CAN_CHANNEL            0

typedef struct {
    CANMessage_t msg;
    CAN_payload_t payload;
} CAN_rx_queue_item_t;

uint32 g_canTxQueueHandle = 0;
uint32 g_canRxQueueHandle = 0;
static uint32 g_canTxQueueCount = 0;

// 에러 관리 (원본 유지)
static uint32 g_canErrorCount = 0;
static uint32 g_canLastErrorType = 0;
static uint32 g_canErrorReportCounter = 0;
#define CAN_ERROR_REPORT_INTERVAL 100 

static CAN_queue_pkt_t g_canTxPool[CAN_TX_POOL_SIZE];
static uint8 g_canTxPoolUsed[CAN_TX_POOL_SIZE];

#define CAN_TASK_STK_SIZE     (2048)
#define CAN_TASK_PRIO         (SAL_PRIO_APP_CFG)

static uint32 g_can_tx_task_id = 0;
static uint32 g_can_tx_task_stk[CAN_TASK_STK_SIZE];
static uint32 g_can_rx_task_id = 0;
static uint32 g_can_rx_task_stk[CAN_TASK_STK_SIZE];

/* -------------------------------------------------------------------------
   내부 함수 선언
   ------------------------------------------------------------------------- */
static void CAN_TxTask_Loop(void *pArg);
static void CAN_RxTask_Loop(void *pArg);
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError);

/* -------------------------------------------------------------------------
   메모리 풀 관리
   ------------------------------------------------------------------------- */
CAN_queue_pkt_t* CAN_AllocPool(void)
{
    SAL_CoreCriticalEnter();
    for (uint8 i = 0; i < CAN_TX_POOL_SIZE; i++) {
        if (g_canTxPoolUsed[i] == 0) {
            g_canTxPoolUsed[i] = 1;
            SAL_CoreCriticalExit();
            return &g_canTxPool[i];
        }
    }
    SAL_CoreCriticalExit();
    return NULL_PTR;
}

void CAN_FreePool(CAN_queue_pkt_t *pPkt)
{
    if (pPkt == NULL_PTR) return;
    SAL_CoreCriticalEnter();
    uint32 index = (uint32)(pPkt - &g_canTxPool[0]);
    if (index < CAN_TX_POOL_SIZE) {
        g_canTxPoolUsed[index] = 0;
        SAL_MemSet(pPkt, 0, sizeof(CAN_queue_pkt_t));
    }
    SAL_CoreCriticalExit();
}

/* -------------------------------------------------------------------------
   CAN 콜백 함수
   ------------------------------------------------------------------------- */
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    (void)ucCh; (void)uiRxIndex; (void)uiRxBufferType;
    
    if (uiError == CAN_ERROR_NONE)
    {
        CANMessage_t sRxMsg;
        CAN_rx_queue_item_t rxItem;
        
        if (CAN_GetNewRxMessage(CAN_CHANNEL, &sRxMsg) == CAN_ERROR_NONE)
        {
            SAL_MemCopy(&rxItem.msg, &sRxMsg, sizeof(CANMessage_t));
            SAL_MemCopy(rxItem.payload.raw, sRxMsg.mData, 8);
            
            // ★ [수정] CRC 체크 제거됨 -> 무조건 통과
            // 바로 큐에 넣습니다.
            SALRetCode_t queueRet = SAL_QueuePut(g_canRxQueueHandle,
                                                 &rxItem,
                                                 sizeof(CAN_rx_queue_item_t),
                                                 0, 
                                                 SAL_OPT_NON_BLOCKING);
            
            if (queueRet != SAL_RET_SUCCESS) {
                // mcu_printf("[CAN] Rx Queue full\r\n");
            }
        }
    }
}

static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }

static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError)
{
    (void)ucCh;
    g_canErrorCount++;
    g_canLastErrorType = uiError;
}

/* -------------------------------------------------------------------------
   1. 초기화
   ------------------------------------------------------------------------- */
void CAN_init(void)
{
    CANErrorType_t ret;
    mcu_printf("[CAN] Initializing...\r\n");
    
    SAL_MemSet(g_canTxPool, 0, sizeof(g_canTxPool));
    SAL_MemSet(g_canTxPoolUsed, 0, sizeof(g_canTxPoolUsed));
    
    SAL_QueueCreate(&g_canTxQueueHandle, (const uint8 *)"CAN_TxQueue", CAN_TX_QUEUE_SIZE, sizeof(CAN_queue_pkt_t*));
    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_RxQueue", CAN_RX_QUEUE_SIZE, sizeof(CAN_rx_queue_item_t));
    
    ret = CAN_Init();
    if (ret != CAN_ERROR_NONE) {
        CAN_Deinit();
        CAN_Init();
    }
    
    CAN_RegisterCallbackFunctionTx(CAN_TxCallback);
    CAN_RegisterCallbackFunctionRx(CAN_RxCallback);
    CAN_RegisterCallbackFunctionError(CAN_ErrorCallback);
    
    mcu_printf("[CAN] Init Done\r\n");
}

/* -------------------------------------------------------------------------
   2. Tx Task
   ------------------------------------------------------------------------- */
static void CAN_TxTask_Loop(void *pArg)
{
    (void)pArg;
    CANMessage_t sTxMsg;
    CAN_queue_pkt_t *rxPacket;
    uint32 uiSizeCopied;
    uint8 ucTxBufferIndex;
    
    SAL_TaskSleep(1000);
    mcu_printf("[CAN] Tx Task Started\r\n");
    
    SAL_MemSet(&sTxMsg, 0, sizeof(CANMessage_t));
    sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mDataLength = 8;
    
    for(;;)
    {
        // 에러 리포팅 (원본 유지)
        if (g_canErrorCount > 0) {
            g_canErrorReportCounter++;
            if (g_canErrorReportCounter >= CAN_ERROR_REPORT_INTERVAL) {
                mcu_printf("[CAN] Error Count: %u (Last: %d)\r\n", (unsigned int)g_canErrorCount, g_canLastErrorType);
                g_canErrorCount = 0;
                g_canErrorReportCounter = 0;
            }
        }
        
        // 큐 대기
        if (SAL_QueueGet(g_canTxQueueHandle, &rxPacket, &uiSizeCopied, 0, 0) == SAL_RET_SUCCESS)
        {
            if (rxPacket != NULL) {
                sTxMsg.mId = rxPacket->id;
                
                // ★ [수정] Timestamp는 유지
                uint32_t timestamp_value = 0;
                TIMESTAMP_get_ms(&timestamp_value);
                rxPacket->body.field.time_ms = (uint16_t)(timestamp_value & 0xFFFFU);

                // ★ [수정] Byte Swapping 및 CRC 계산 제거 -> 데이터 그대로 복사
                SAL_MemCopy(sTxMsg.mData, rxPacket->body.raw, 8);
                
                if (CAN_SendMessage(CAN_CHANNEL, &sTxMsg, &ucTxBufferIndex) != CAN_ERROR_NONE) {
                    mcu_printf("[CAN] Send Fail\r\n");
                }
                
                CAN_FreePool(rxPacket);
            }
        }
    }
}

/* -------------------------------------------------------------------------
   3. Rx Task
   ------------------------------------------------------------------------- */
static void CAN_RxTask_Loop(void *pArg)
{
    (void)pArg;
    CAN_rx_queue_item_t rxItem;
    uint32 uiSizeCopied;
    
    mcu_printf("[CAN] Rx Task Started\r\n");
    
    for(;;)
    {
        if (SAL_QueueGet(g_canRxQueueHandle, &rxItem, &uiSizeCopied, 0, 0) == SAL_RET_SUCCESS)
        {
            // main.c로 데이터 전달
            CAN_consume_rx_message(&rxItem.msg, rxItem.payload);
        }
    }
}

// Start Functions
void CAN_TX_start_task(void) {
    static uint8 init_done = 0;
    if (!init_done) { CAN_init(); init_done = 1; }
    if(g_can_tx_task_id == 0)
        SAL_TaskCreate(&g_can_tx_task_id, (const uint8 *)"CAN Tx", (SALTaskFunc)CAN_TxTask_Loop, &g_can_tx_task_stk[0], CAN_TASK_STK_SIZE, CAN_TASK_PRIO, NULL);
}

void CAN_RX_start_task(void) {
    static uint8 init_done = 0;
    if (!init_done) { CAN_init(); init_done = 1; }
    if(g_can_rx_task_id == 0)
        SAL_TaskCreate(&g_can_rx_task_id, (const uint8 *)"CAN Rx", (SALTaskFunc)CAN_RxTask_Loop, &g_can_rx_task_stk[0], CAN_TASK_STK_SIZE, CAN_TASK_PRIO, NULL);
}

void CAN_start_task(void) {
    CAN_TX_start_task();
    CAN_RX_start_task();
}