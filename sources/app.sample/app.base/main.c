// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>

#include <uart.h>
#include <gpio.h>
#include <pdm.h>
#include <string.h>

static void Delay_Loop(volatile uint32 count);
static int32 Simple_Atoi(char *str);
static float Clamp_Val(float x, float lo, float hi);

static void Motor_HW_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Parse_Command(char *cmd);

// SDK 내부 함수들도 미리 선언해야 "static conflicts" 에러가 안 납니다.
static void AppTaskCreate(void); 
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void); // 로그에 보여서 추가함

//bluetooth set ch0_A28(18),29(19)
#define BLUETOOTH_UART_CH       (0)             // UART Ch0 (보드 핀맵 확인!)
#define HC06_BAUDRATE           (9600)        
#define CMD_BUF_SIZE            (64)  

// [공식 예제 매크로 참고]
// #define PMM_ON                  (1UL) 
// #define PMM_OFF                 (0UL)

#define USE_PMM_MONITOR         (0UL) // PMM_OFF
// Port Number는 보통 0이 CH0(Port A 등)에 해당합니다. 
#define GPIO_PERICH_CH0         (0UL)

// left motor - PDM Ch0
#define MOTOR_L_PWM_CH          (0)             
#define MOTOR_L_IN1             GPIO_GPB(23)     // TODO: 실제 핀 번호로 수정 (예: GPIO_GPC(3))
#define MOTOR_L_IN2             GPIO_GPB(21)     

// right motor - PDM Ch1
#define MOTOR_R_PWM_CH          (1)             
#define MOTOR_R_IN1             GPIO_GPB(2)     
#define MOTOR_R_IN2             GPIO_GPB(3)     

// parameter set
#define PWM_PERIOD_NS           (1000000UL)     
#define MAX_DUTY_SCALE          (1000UL) // 사용자 제어 해상도 (0~1000)
#define TURN_SENSITIVITY        (1.0f)          // 회전 민감도

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;

volatile int32 g_TargetF = 0;   // Forward : -100 ~ 100
volatile int32 g_TargetR = 0;   // Rotation: -100 ~ 100

static uint32 g_LastDutyL = 0xFFFF; // 초기값은 불가능한 값으로 설정
static uint32 g_LastDutyR = 0xFFFF;
/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

//3 Utility Function
void Delay_Loop(volatile uint32 count) { while (count--) { __asm("nop"); }}

int32 Simple_Atoi(char *str)
{
    int32 res = 0;
    int32 i = 0;
    while (str[i] >= '0' && str[i] <= '9')
    {
        res = res * 10 + (str[i] - '0');
        i++;
    }
    return res;
}

static float Clamp_Val(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// 4 Motor Init, Control Function
    // 4-1 Motor Init
void Motor_HW_Init(void)
{
    // GPIO 설정
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);

    // [FIX] PDM 초기화 (인자 없음)
    PDM_Init(); 

    // 초기 상태: 0속도 설정 및 Enable
    g_LastDutyL = 0xFFFF; 
    g_LastDutyR = 0xFFFF;
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale)
{
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0; // [추가] 타임아웃 카운터

    // [최적화] 값이 바뀌지 않았다면 아무것도 안 함 (매우 중요)
    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    // [디버그] 로그 출력
    if (duty_1000_scale > 0) {
        mcu_printf("[PWM] Ch:%d Duty:%d (Update)\n", channel, duty_1000_scale);
    }

    // 설정값 준비
    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber      = GPIO_PERICH_CH0; 
    pdmConfig.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl      = 1; 
    pdmConfig.mcClockDivide     = 0; 
    pdmConfig.mcPeriodNanoSec1  = PWM_PERIOD_NS; 
    
    if(duty_1000_scale > MAX_DUTY_SCALE) duty_1000_scale = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1    = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    // 1. 채널 끄기
    PDM_Disable(channel, USE_PMM_MONITOR);

    // [★핵심 해결책] 하드웨어가 실제로 꺼질 때까지 기다림
    // 이 부분이 없어서 설정이 무시되었던 것임
    wait_cnt = 0;
    while(PDM_GetChannelStatus(channel) == PDM_STATE_BUSY) 
    { 
        wait_cnt++;
        if(wait_cnt > 10000) break; // 무한루프 방지 (안전장치)
        // 아주 짧은 지연
        __asm("nop"); 
    }

    // 2. 설정 적용 (이제 안전하게 적용됨)
    PDM_SetConfig(channel, &pdmConfig);

    // 3. 다시 켜기
    if (duty_1000_scale > 0)
    {
        PDM_Enable(channel, USE_PMM_MONITOR);
    }
}
    // 4-2 Motor Control Function
void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed)
{
    uint32 duty = 0;
    float abs_speed = 0.0f;

    // Control
    if (speed > 0)          // 전진
    {
        GPIO_Set(p1, 1);
        GPIO_Set(p2, 0);
        abs_speed = speed;
    }
    else if (speed < 0)     // 후진
    {
        GPIO_Set(p1, 0);
        GPIO_Set(p2, 1);
        abs_speed = -speed; // 절대값으로 변환
    }
    else                    // 정지
    {
        GPIO_Set(p1, 0);
        GPIO_Set(p2, 0);
        abs_speed = 0;
    }

    // ARR scailing (0~1000)
    duty = (uint32)(abs_speed * 10.0f);

    // Safty
    //if (duty > MAX_DUTY) duty = MAX_DUTY;

    // Set PWM
    Update_PWM_Duty(channel, duty);
}

// 5 Skid Steering
void Car_Update_Logic(void)
{
    float F = (float)g_TargetF;
    float R = (float)g_TargetR;

    // Mixing
    float left_val  = F + (R * TURN_SENSITIVITY);
    float right_val = F - (R * TURN_SENSITIVITY);

    // Clamping (-100 ~ 100)
    left_val  = Clamp_Val(left_val,  -100.0f, 100.0f);
    right_val = Clamp_Val(right_val, -100.0f, 100.0f);

    // Motor Update
    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

// 6 Command Parsing
void Parse_Command(char *cmd)
{
    int32 i = 0;
    int32 val = 0;

    while (cmd[i] != '\0')
    {
        if (cmd[i] == 'F') { // 전진
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetF = val;            
        }
        else if (cmd[i] == 'B') { // 후진
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetF = -val;           
        }
        else if (cmd[i] == 'R') { // 우회전
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetR = val;            
        }
        else if (cmd[i] == 'L') { // 좌회전
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetR = -val;           
        }
        i++;
    }
}


/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/

void cmain(void)
{
    UartParam_t uartParam;
    static uint8 rx_char;
    static char cmd_buf[64];
    static uint8 cmd_len = 0;

    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init(); 

    // 모터 초기화
    Motor_HW_Init();

    // UART 초기화
    UART_Close(BLUETOOTH_UART_CH);
    uartParam.sCh           = BLUETOOTH_UART_CH;
    uartParam.sPriority     = 10U;
    uartParam.sBaudrate     = HC06_BAUDRATE;
    uartParam.sMode         = UART_POLLING_MODE;
    uartParam.sCtsRts       = UART_CTSRTS_OFF;
    uartParam.sPortCfg      = 0; 
    uartParam.sWordLength   = WORD_LEN_8;
    uartParam.sFIFO         = ENABLE_FIFO;
    uartParam.s2StopBit     = TWO_STOP_BIT_OFF;
    uartParam.sParity       = 0;
    uartParam.sFnCallback   = NULL;
    UART_Open(&uartParam);

    mcu_printf("\n[2VCP3] RC Car Ready! (Scale: 0~1000)\n");


    while (1)
    {
        if (UART_Read(BLUETOOTH_UART_CH, &rx_char, 1) > 0)
        {
            if (rx_char == '\n' || rx_char == '\r')
            {
                if (cmd_len > 0) 
                {
                    cmd_buf[cmd_len] = '\0';
                    mcu_printf("\n[BT RX] %s\n", cmd_buf);
                    Parse_Command(cmd_buf);
                    cmd_len = 0;
                }
            }
            else
            {
                if (cmd_len < 63) cmd_buf[cmd_len++] = (char)rx_char;
                else cmd_len = 0; 
            }
        }

        Car_Update_Logic();
        Delay_Loop(1000); 
    }
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )