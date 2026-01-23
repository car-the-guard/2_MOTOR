// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c
* Description : VCP-G RTOS Integration (Main Control with Motor Task)
***************************************************************************************************
*/

#define MCU_BSP_SUPPORT_APP_BASE 1

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <uart.h>
#include <gpio.h>
#include <pdm.h>
#include <ictc.h>
#include <string.h>

// ★ 우리가 만든 CAN 통신용 헤더 포함
#include "can_app.h" 

// =========================================================
// [0] 설정
// =========================================================
#define UART_DEBUG_CH       (0)
#define UART_BAUDRATE       (115200)

// ID 설정 (0x24: 센서, 0x28: 내 속도)
#define CAN_ID_RX_SENSOR    (0x24) 
#define CAN_ID_TX_SPEED     (0x28)

// 모터 핀
#define MOTOR_L_PWM_CH      (0)             
#define MOTOR_L_IN1         GPIO_GPB(23)     
#define MOTOR_L_IN2         GPIO_GPB(21)     
#define MOTOR_R_PWM_CH      (8)             
#define MOTOR_R_IN1         GPIO_GPB(24)     
#define MOTOR_R_IN2         GPIO_GPB(22)     

// 엔코더
#define ENCODER_L_ICTC_CH   (0)         
#define ENCODER_GPIO_PIN    GPIO_GPC(4)
#define ENCODER_PIN_INDEX   (64UL) 

// PWM & 물리상수
#define PWM_PERIOD_NS       (1000000UL)     
#define MAX_DUTY_SCALE      (1000UL) 
#define GPIO_PERICH_CH0     (0UL)
#define USE_PMM_MONITOR     (0UL)
#define COUNTS_PER_REV      (1600.0f) 
#define WHEEL_DIA_M         (0.065f)  
#define PI                  (3.141592f)

// Task 설정
#define MOTOR_TASK_STK_SIZE (1024)
#define MOTOR_TASK_PRIO     (SAL_PRIO_APP_CFG + 1) // CAN보다 우선순위 낮거나 같게

// =========================================================
// [1] 전역 변수
// =========================================================
volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   

typedef struct {
    int32_t counter;       
    int32_t last_counter;  
    float speed_mps;       
} Encoder_t;

Encoder_t Enc1;
static uint32 g_MotorTaskID;
static uint32 g_MotorTaskStk[MOTOR_TASK_STK_SIZE];

// =========================================================
// [2] 하드웨어 초기화
// =========================================================
static void System_HW_Init(void) {
    UartParam_t uartParam;
    UART_Close(UART_DEBUG_CH);
    uartParam.sCh = UART_DEBUG_CH;
    uartParam.sPriority = 10U;
    uartParam.sBaudrate = UART_BAUDRATE;
    uartParam.sMode = UART_POLLING_MODE;
    uartParam.sCtsRts = UART_CTSRTS_OFF;
    uartParam.sPortCfg = 0; 
    uartParam.sWordLength = WORD_LEN_8;
    uartParam.sFIFO = ENABLE_FIFO;
    uartParam.s2StopBit = TWO_STOP_BIT_OFF;
    uartParam.sParity = 0;
    uartParam.sFnCallback = NULL;
    UART_Open(&uartParam);
}

static void Motor_HW_Init(void) {
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    PDM_Init(); 
}

static void Encoder_HW_Init(void) {
    // 핀 설정만 간단히 (상세 ICTC 설정은 필요시 추가)
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL)));
}

// =========================================================
// [3] ★ 핵심 연결고리: CAN 수신 데이터 처리 함수
// =========================================================
// 이 함수는 can_app.c 의 Rx Task 안에서 호출됩니다.
void CAN_consume_rx_message(CANMessage_t *pMsg, CAN_payload_t payload)
{
    // [ID 확인] 0x24 (센서 데이터) 인가?
    if (pMsg->mId == CAN_ID_RX_SENSOR) 
    {
        // 데이터 파싱 (0번 바이트: 직진, 1번 바이트: 회전)
        int8_t cmd_fwd = (int8_t)payload.field.data[0];
        int8_t cmd_rot = (int8_t)payload.field.data[1];

        // 전역 변수 업데이트 (Motor Task가 이걸 보고 움직임)
        g_TargetF = (int32)cmd_fwd;
        g_TargetR = (int32)cmd_rot;
        
        // 로그 출력 (너무 자주 찍히면 주석 처리)
        // mcu_printf("[RX] ID 0x24 -> Target Updated: F=%d R=%d\n", g_TargetF, g_TargetR);
    }
}

// =========================================================
// [4] 모터 제어 태스크
// =========================================================
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale) {
    PDMModeConfig_t pdmConfig;
    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber      = GPIO_PERICH_CH0; 
    pdmConfig.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl      = 1; 
    pdmConfig.mcPeriodNanoSec1  = PWM_PERIOD_NS; 
    pdmConfig.mcDutyNanoSec1    = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);
    PDM_SetConfig(channel, &pdmConfig);
    PDM_Enable(channel, USE_PMM_MONITOR);
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed) {
    uint32 duty = 0;
    float abs_speed = 0.0f;
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed; }
    else if (speed < 0) { GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed; }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0; }
    
    if(abs_speed > 100.0f) abs_speed = 100.0f;
    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

// 모터 태스크 루프
static void Motor_Task_Loop(void *pArg)
{
    (void)pArg;
    mcu_printf("[TASK] Motor Control Loop Started!\n");

    for (;;)
    {
        // 1. 주기적 실행 (10ms Sleep) - 다른 태스크(CAN)에게 CPU 양보
        SAL_TaskSleep(10); 

        // 2. 전역 변수 읽기 (CAN Rx Task가 업데이트해준 값)
        float F = (float)g_TargetF;
        float R = (float)g_TargetR;

        // 3. 모터 구동
        Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, F);
        Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, F);
        
        // 4. 상태 로그 (100ms 마다 한 번씩)
        static int log_cnt = 0;
        if(++log_cnt >= 10) {
            log_cnt = 0;
            mcu_printf("Motor Status: F=%d R=%d\n", (int)F, (int)R);
        }
    }
}

// =========================================================
// [5] 메인 진입점
// =========================================================
void cmain(void)
{
    (void)SAL_Init(); // RTOS 커널 시작
    BSP_PreInit();
    BSP_Init(); 

    System_HW_Init();
    mcu_printf("\n[SYSTEM] Booting RTOS Mode...\n");

    // 1. 하드웨어 초기화
    Motor_HW_Init();
    Encoder_HW_Init();

    // 2. CAN 태스크 시작 (can_app.c에 있는 함수 호출)
    // -> 내부적으로 CAN_Init, 큐 생성, Rx/Tx 태스크 생성을 다 해줍니다.
    CAN_start_task(); 

    // 3. 모터 태스크 시작
    SAL_TaskCreate(&g_MotorTaskID, 
                   (const uint8 *)"Motor Task", 
                   (SALTaskFunc)Motor_Task_Loop, 
                   &g_MotorTaskStk[0], 
                   MOTOR_TASK_STK_SIZE, 
                   MOTOR_TASK_PRIO, 
                   NULL);

    mcu_printf("[SYSTEM] All Tasks Created. Running...\n");
    
    // RTOS 스케줄러가 알아서 태스크를 돌리므로 cmain은 여기서 끝납니다.
}

static void AppTaskCreate(void) {}
static void DisplayAliveLog(void) {}
static void DisplayOTPInfo(void) {}

#endif