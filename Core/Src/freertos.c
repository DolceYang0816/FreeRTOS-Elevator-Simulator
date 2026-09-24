/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for FreeRTOS applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
 
#include "stm3210x_lcd.h"
 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    REQ_INTERNAL = 0,       /* 轿厢内部按键：1~4 */
    REQ_EXTERNAL_UP,        /* 外部上行呼叫：1U、2U、3U */
    REQ_EXTERNAL_DOWN       /* 外部下行呼叫：2D、3D、4D */
} RequestType_t;

typedef enum
{
    DIR_DOWN = -1,
    DIR_IDLE = 0,
    DIR_UP = 1
} ElevatorDir_t;

typedef enum
{
    DOOR_CLOSED = 0,
    DOOR_OPEN
} DoorState_t;

typedef struct
{
    uint8_t floor;          /* 1~4 */
    RequestType_t type;
} RequestMsg_t;

typedef struct
{
    uint8_t currentFloor;   /* 当前楼层：1~4 */
    ElevatorDir_t dir;      /* 当前方向 */
    DoorState_t door;       /* 门状态 */

    uint8_t insideReq[5];   /* 内部目标楼层请求 */
    uint8_t upReq[5];       /* 外部上行请求 */
    uint8_t downReq[5];     /* 外部下行请求 */

    uint32_t servedCount;   /* 已服务次数 */
} ElevatorState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REQUEST_QUEUE_LEN      16
#define UART_RX_QUEUE_LEN      32

#define ELEVATOR_MOVE_TIME_MS  3000
#define DOOR_OPEN_TIME_MS      4000
#define DISPLAY_PERIOD_MS      2000
#define KEY_DEBOUNCE_MS        20
#define KEY_RELEASE_MS         20

/* LCD UI RGB565 colors */
#define UI_BG          ((uint16_t)0x0841)   /* dark blue-gray */
#define UI_HEADER      ((uint16_t)0x001F)   /* blue */
#define UI_PANEL       ((uint16_t)0x2104)   /* dark panel */
#define UI_PANEL2      ((uint16_t)0x3186)
#define UI_WHITE       ((uint16_t)0xFFFF)
#define UI_BLACK       ((uint16_t)0x0000)
#define UI_GREEN       ((uint16_t)0x07E0)
#define UI_RED         ((uint16_t)0xF800)
#define UI_YELLOW      ((uint16_t)0xFFE0)
#define UI_CYAN        ((uint16_t)0x07FF)
#define UI_ORANGE      ((uint16_t)0xFD20)
#define UI_GRAY        ((uint16_t)0x8410)
#define UI_PURPLE      ((uint16_t)0x780F)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static TaskHandle_t elevatorTaskHandle  = NULL;
static TaskHandle_t displayTaskHandle   = NULL;
static TaskHandle_t keyTaskHandle       = NULL;
static TaskHandle_t uartParseTaskHandle = NULL;

static QueueHandle_t requestQueueHandle = NULL;  /* 按键/串口解析后的电梯请求 */
static QueueHandle_t uartRxQueueHandle  = NULL;  /* USART1 中断收到的原始字节 */

static SemaphoreHandle_t stateMutexHandle = NULL;
static SemaphoreHandle_t uartMutexHandle  = NULL;

static ElevatorState_t gElevator =
{
    .currentFloor = 1,
    .dir = DIR_IDLE,
    .door = DOOR_CLOSED,
    .insideReq = {0},
    .upReq = {0},
    .downReq = {0},
    .servedCount = 0
};

static uint8_t uartRxByte = 0;
static volatile uint8_t keyScanning = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void ElevatorTask(void *argument);
static void DisplayTask(void *argument);
static void KeyTask(void *argument);
static void UartParseTask(void *argument);

static void UartPrintf(const char *fmt, ...);

static void Keypad_AllColumnsHigh(void);
static void Keypad_AllColumnsLow(void);
static uint8_t Keypad_Scan(void);
static uint8_t Keypad_AnyRowHigh(void);

static void Elevator_ProcessNewRequests(TickType_t waitTicks);
static void Elevator_AddRequest(const RequestMsg_t *msg);
static void Elevator_DelayWithInput(uint32_t ms);
static void Elevator_OpenDoorAndServe(void);
static void Elevator_MoveOneFloor(ElevatorDir_t dir);

static uint8_t Elevator_HasAnyRequestLocked(void);
static uint8_t Elevator_HasRequestAboveLocked(void);
static uint8_t Elevator_HasRequestBelowLocked(void);
static uint8_t Elevator_FloorHasAnyRequestLocked(uint8_t floor);
static uint8_t Elevator_NeedStopAtCurrentFloorLocked(void);
static void Elevator_DecideDirectionLocked(void);

static const char *DirToStr(ElevatorDir_t dir);
static const char *DoorToStr(DoorState_t door);
static const char *ReqTypeToStr(RequestType_t type);
static void Display_PrintState(void);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
	

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

    BaseType_t ret;

    requestQueueHandle = xQueueCreate(REQUEST_QUEUE_LEN, sizeof(RequestMsg_t));
    uartRxQueueHandle  = xQueueCreate(UART_RX_QUEUE_LEN, sizeof(uint8_t));

    stateMutexHandle = xSemaphoreCreateMutex();
    uartMutexHandle  = xSemaphoreCreateMutex();

    if (requestQueueHandle == NULL)
        printf("requestQueue create FAIL\r\n");
    else
        printf("requestQueue create OK\r\n");

    if (uartRxQueueHandle == NULL)
        printf("uartRxQueue create FAIL\r\n");
    else
        printf("uartRxQueue create OK\r\n");

    if (stateMutexHandle == NULL)
        printf("stateMutex create FAIL\r\n");
    else
        printf("stateMutex create OK\r\n");

    if (uartMutexHandle == NULL)
        printf("uartMutex create FAIL\r\n");
    else
        printf("uartMutex create OK\r\n");

    Keypad_AllColumnsHigh();

    ret = xTaskCreate(ElevatorTask,
                      "Elevator",
                      384,
                      NULL,
                      tskIDLE_PRIORITY + 3,
                      &elevatorTaskHandle);

    if (ret == pdPASS)
        printf("ElevatorTask create OK\r\n");
    else
        printf("ElevatorTask create FAIL\r\n");

    ret = xTaskCreate(UartParseTask,
                      "UartParse",
                      256,
                      NULL,
                      tskIDLE_PRIORITY + 2,
                      &uartParseTaskHandle);

    if (ret == pdPASS)
        printf("UartParseTask create OK\r\n");
    else
        printf("UartParseTask create FAIL\r\n");

    ret = xTaskCreate(KeyTask,
                      "KeyInput",
                      256,
                      NULL,
                      tskIDLE_PRIORITY + 2,
                      &keyTaskHandle);

    if (ret == pdPASS)
        printf("KeyTask create OK\r\n");
    else
        printf("KeyTask create FAIL\r\n");

    ret = xTaskCreate(DisplayTask,
                      "Display",
                      512,
                      NULL,
                      tskIDLE_PRIORITY + 1,
                      &displayTaskHandle);

    if (ret == pdPASS)
        printf("DisplayTask create OK\r\n");
    else
        printf("DisplayTask create FAIL\r\n");

    if (HAL_UART_Receive_IT(&huart1, &uartRxByte, 1) == HAL_OK)
    {
        printf("USART1 RX interrupt start OK\r\n");
    }
    else
    {
        printf("USART1 RX interrupt start ERROR\r\n");
    }

  /* USER CODE END Init */
}

/* USER CODE BEGIN Application */

/* ========================= 基础打印函数 ========================= */

static void UartPrintf(const char *fmt, ...)
{
    va_list args;

    if (uartMutexHandle != NULL)
    {
        xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
    }

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    if (uartMutexHandle != NULL)
    {
        xSemaphoreGive(uartMutexHandle);
    }
}

static const char *DirToStr(ElevatorDir_t dir)
{
    switch (dir)
    {
        case DIR_UP:   return "UP";
        case DIR_DOWN: return "DOWN";
        default:       return "IDLE";
    }
}

static const char *DoorToStr(DoorState_t door)
{
    return (door == DOOR_OPEN) ? "OPEN" : "CLOSED";
}

static const char *ReqTypeToStr(RequestType_t type)
{
    switch (type)
    {
        case REQ_INTERNAL:      return "IN";
        case REQ_EXTERNAL_UP:   return "UP";
        case REQ_EXTERNAL_DOWN: return "DOWN";
        default:                return "UNKNOWN";
    }
}

/* ========================= 矩阵键盘部分 ========================= */

static void Keypad_AllColumnsHigh(void)
{
    HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
}

static void Keypad_AllColumnsLow(void)
{
    HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_RESET);
}

static void Keypad_SetColumn(uint8_t col, GPIO_PinState state)
{
    switch (col)
    {
        case 0: HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, state); break;
        case 1: HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, state); break;
        case 2: HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, state); break;
        case 3: HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, state); break;
        default: break;
    }
}

static uint8_t Keypad_ReadRow(uint8_t row)
{
    switch (row)
    {
        case 0: return (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET);
        case 1: return (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET);
        case 2: return (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET);
        case 3: return (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET);
        default: return 0;
    }
}

static uint8_t Keypad_AnyRowHigh(void)
{
    if (HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET) return 1;
    if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET) return 1;
    if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET) return 1;
    if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET) return 1;
    return 0;
}

static uint8_t Keypad_Scan(void)
{
    static const uint8_t keyMap[4][4] =
    {
        {1, 2, 3, 10},
        {4, 5, 6, 11},
        {7, 8, 9, 12},
        {14, 0, 15, 13}
    };

    uint8_t row;
    uint8_t col;
    uint8_t key = 0;

    keyScanning = 1;

    Keypad_AllColumnsLow();

    for (col = 0; col < 4; col++)
    {
        Keypad_SetColumn(col, GPIO_PIN_SET);

        for (volatile uint32_t d = 0; d < 2000; d++)
        {
            __NOP();
        }

        for (row = 0; row < 4; row++)
        {
            if (Keypad_ReadRow(row))
            {
                key = keyMap[row][col];
                goto scan_end;
            }
        }

        Keypad_SetColumn(col, GPIO_PIN_RESET);
    }

scan_end:
    Keypad_AllColumnsHigh();
    keyScanning = 0;

    return key;
}

/* ========================= 请求处理部分 ========================= */

static void Elevator_AddRequest(const RequestMsg_t *msg)
{
    uint8_t valid = 1;

    if (msg == NULL)
    {
        return;
    }

    if (msg->floor < 1 || msg->floor > 4)
    {
        valid = 0;
    }

    if (msg->type == REQ_EXTERNAL_DOWN && msg->floor == 1)
    {
        valid = 0;
    }

    if (msg->type == REQ_EXTERNAL_UP && msg->floor == 4)
    {
        valid = 0;
    }

    if (valid == 0)
    {
        UartPrintf("[INVALID] floor=%d type=%s\r\n",
                   msg->floor,
                   ReqTypeToStr(msg->type));
        return;
    }

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);

    if (msg->type == REQ_INTERNAL)
    {
        gElevator.insideReq[msg->floor] = 1;
    }
    else if (msg->type == REQ_EXTERNAL_UP)
    {
        gElevator.upReq[msg->floor] = 1;
    }
    else if (msg->type == REQ_EXTERNAL_DOWN)
    {
        gElevator.downReq[msg->floor] = 1;
    }

    xSemaphoreGive(stateMutexHandle);

    UartPrintf("[REQUEST] floor=%d type=%s\r\n",
               msg->floor,
               ReqTypeToStr(msg->type));
}

static void Elevator_ProcessNewRequests(TickType_t waitTicks)
{
    RequestMsg_t msg;

    if (requestQueueHandle == NULL)
    {
        return;
    }

    if (xQueueReceive(requestQueueHandle, &msg, waitTicks) == pdPASS)
    {
        Elevator_AddRequest(&msg);

        while (xQueueReceive(requestQueueHandle, &msg, 0) == pdPASS)
        {
            Elevator_AddRequest(&msg);
        }
    }
}

static uint8_t Elevator_FloorHasAnyRequestLocked(uint8_t floor)
{
    if (floor < 1 || floor > 4)
    {
        return 0;
    }

    if (gElevator.insideReq[floor]) return 1;
    if (gElevator.upReq[floor])     return 1;
    if (gElevator.downReq[floor])   return 1;

    return 0;
}

static uint8_t Elevator_HasAnyRequestLocked(void)
{
    uint8_t i;

    for (i = 1; i <= 4; i++)
    {
        if (Elevator_FloorHasAnyRequestLocked(i))
        {
            return 1;
        }
    }

    return 0;
}

static uint8_t Elevator_HasRequestAboveLocked(void)
{
    uint8_t i;

    for (i = gElevator.currentFloor + 1; i <= 4; i++)
    {
        if (Elevator_FloorHasAnyRequestLocked(i))
        {
            return 1;
        }
    }

    return 0;
}

static uint8_t Elevator_HasRequestBelowLocked(void)
{
    int8_t i;

    for (i = (int8_t)gElevator.currentFloor - 1; i >= 1; i--)
    {
        if (Elevator_FloorHasAnyRequestLocked((uint8_t)i))
        {
            return 1;
        }
    }

    return 0;
}

static uint8_t Elevator_NeedStopAtCurrentFloorLocked(void)
{
    uint8_t f = gElevator.currentFloor;

    if (gElevator.insideReq[f])
    {
        return 1;
    }

    if (gElevator.dir == DIR_IDLE)
    {
        return Elevator_FloorHasAnyRequestLocked(f);
    }

    if (gElevator.dir == DIR_UP)
    {
        if (gElevator.upReq[f])
        {
            return 1;
        }

        if (!Elevator_HasRequestAboveLocked() && gElevator.downReq[f])
        {
            return 1;
        }
    }

    if (gElevator.dir == DIR_DOWN)
    {
        if (gElevator.downReq[f])
        {
            return 1;
        }

        if (!Elevator_HasRequestBelowLocked() && gElevator.upReq[f])
        {
            return 1;
        }
    }

    return 0;
}

static void Elevator_DecideDirectionLocked(void)
{
    if (!Elevator_HasAnyRequestLocked())
    {
        gElevator.dir = DIR_IDLE;
        return;
    }

    if (gElevator.dir == DIR_IDLE)
    {
        if (Elevator_FloorHasAnyRequestLocked(gElevator.currentFloor))
        {
            gElevator.dir = DIR_IDLE;
        }
        else if (Elevator_HasRequestAboveLocked())
        {
            gElevator.dir = DIR_UP;
        }
        else if (Elevator_HasRequestBelowLocked())
        {
            gElevator.dir = DIR_DOWN;
        }
        return;
    }

    if (gElevator.dir == DIR_UP)
    {
        if (Elevator_HasRequestAboveLocked())
        {
            gElevator.dir = DIR_UP;
        }
        else if (Elevator_FloorHasAnyRequestLocked(gElevator.currentFloor))
        {
            gElevator.dir = DIR_UP;
        }
        else if (Elevator_HasRequestBelowLocked())
        {
            gElevator.dir = DIR_DOWN;
        }
        else
        {
            gElevator.dir = DIR_IDLE;
        }
        return;
    }

    if (gElevator.dir == DIR_DOWN)
    {
        if (Elevator_HasRequestBelowLocked())
        {
            gElevator.dir = DIR_DOWN;
        }
        else if (Elevator_FloorHasAnyRequestLocked(gElevator.currentFloor))
        {
            gElevator.dir = DIR_DOWN;
        }
        else if (Elevator_HasRequestAboveLocked())
        {
            gElevator.dir = DIR_UP;
        }
        else
        {
            gElevator.dir = DIR_IDLE;
        }
        return;
    }
}

static void Elevator_DelayWithInput(uint32_t ms)
{
    uint32_t elapsed = 0;
    const uint32_t step = 50;

    while (elapsed < ms)
    {
        uint32_t delay = step;

        if ((ms - elapsed) < step)
        {
            delay = ms - elapsed;
        }

        vTaskDelay(pdMS_TO_TICKS(delay));
        elapsed += delay;

        Elevator_ProcessNewRequests(0);
    }
}

static void Elevator_OpenDoorAndServe(void)
{
    uint8_t f;

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);

    f = gElevator.currentFloor;
    gElevator.door = DOOR_OPEN;

    gElevator.insideReq[f] = 0;
    gElevator.upReq[f] = 0;
    gElevator.downReq[f] = 0;

    gElevator.servedCount++;

    xSemaphoreGive(stateMutexHandle);

    UartPrintf("[ARRIVE] floor=%d, door open 2s\r\n", f);

    Elevator_DelayWithInput(DOOR_OPEN_TIME_MS);

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);
    gElevator.door = DOOR_CLOSED;
    xSemaphoreGive(stateMutexHandle);

    UartPrintf("[DOOR] floor=%d, door closed\r\n", f);
}

static void Elevator_MoveOneFloor(ElevatorDir_t dir)
{
    uint8_t fromFloor;
    uint8_t toFloor;

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);

    fromFloor = gElevator.currentFloor;

    if (dir == DIR_UP && gElevator.currentFloor < 4)
    {
        toFloor = gElevator.currentFloor + 1;
        gElevator.dir = DIR_UP;
    }
    else if (dir == DIR_DOWN && gElevator.currentFloor > 1)
    {
        toFloor = gElevator.currentFloor - 1;
        gElevator.dir = DIR_DOWN;
    }
    else
    {
        gElevator.dir = DIR_IDLE;
        xSemaphoreGive(stateMutexHandle);
        return;
    }

    xSemaphoreGive(stateMutexHandle);

    UartPrintf("[MOVE] %dF -> %dF, dir=%s\r\n",
               fromFloor,
               toFloor,
               DirToStr(dir));

    Elevator_DelayWithInput(ELEVATOR_MOVE_TIME_MS);

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);
    gElevator.currentFloor = toFloor;
    xSemaphoreGive(stateMutexHandle);
}

/* ========================= 任务函数 ========================= */

static void ElevatorTask(void *argument)
{
    ElevatorDir_t dir;
    uint8_t hasAny;

    UartPrintf("\r\n========== Elevator FreeRTOS Demo ==========\r\n");
    UartPrintf("Internal key: press 1~4 on keypad\r\n");
    UartPrintf("External call: send ASCII 1U, 2U, 2D, 3U, 3D, 4D\r\n");
    UartPrintf("USART: 115200, 8N1, no HEX mode\r\n");
    UartPrintf("Start at 1F, door closed\r\n");
    UartPrintf("===========================================\r\n\r\n");

    for (;;)
    {
        Elevator_ProcessNewRequests(0);

        xSemaphoreTake(stateMutexHandle, portMAX_DELAY);

        if (Elevator_NeedStopAtCurrentFloorLocked())
        {
            xSemaphoreGive(stateMutexHandle);
            Elevator_OpenDoorAndServe();
            continue;
        }

        Elevator_DecideDirectionLocked();

        dir = gElevator.dir;
        hasAny = Elevator_HasAnyRequestLocked();

        xSemaphoreGive(stateMutexHandle);

        if (!hasAny || dir == DIR_IDLE)
        {
            Elevator_ProcessNewRequests(pdMS_TO_TICKS(100));
            continue;
        }

        Elevator_MoveOneFloor(dir);
    }
}

static void DisplayTask(void *argument)
{
    for (;;)
    {
        Display_PrintState();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

static void KeyTask(void *argument)
{
    uint8_t key;
    RequestMsg_t msg;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(KEY_DEBOUNCE_MS));

        key = Keypad_Scan();

        if (key >= 1 && key <= 4)
        {
            msg.floor = key;
            msg.type = REQ_INTERNAL;

            xQueueSend(requestQueueHandle, &msg, portMAX_DELAY);
        }

        while (Keypad_AnyRowHigh())
        {
            vTaskDelay(pdMS_TO_TICKS(KEY_RELEASE_MS));
        }

        while (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            ;
        }
    }
}

static void UartParseTask(void *argument)
{
    uint8_t ch;
    uint8_t floor = 0;
    RequestMsg_t msg;

    for (;;)
    {
        if (xQueueReceive(uartRxQueueHandle, &ch, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        if (ch >= 'a' && ch <= 'z')
        {
            ch = ch - 'a' + 'A';
        }

        if (ch == '\r' || ch == '\n' || ch == ' ')
        {
            continue;
        }

        if (floor == 0)
        {
            if (ch >= '1' && ch <= '4')
            {
                floor = ch - '0';
            }
            else
            {
                UartPrintf("[UART] invalid floor char: %c\r\n", ch);
            }
        }
        else
        {
            if (ch == 'U')
            {
                msg.floor = floor;
                msg.type = REQ_EXTERNAL_UP;
                xQueueSend(requestQueueHandle, &msg, portMAX_DELAY);
            }
            else if (ch == 'D')
            {
                msg.floor = floor;
                msg.type = REQ_EXTERNAL_DOWN;
                xQueueSend(requestQueueHandle, &msg, portMAX_DELAY);
            }
            else
            {
                UartPrintf("[UART] invalid direction char: %c\r\n", ch);
            }

            floor = 0;
        }
    }
}

/* ========================= 状态显示 ========================= */

static void PrintReqArray(const char *name, const uint8_t req[5])
{
    uint8_t i;
    uint8_t empty = 1;

    printf("%s:", name);

    for (i = 1; i <= 4; i++)
    {
        if (req[i])
        {
            printf(" %dF", i);
            empty = 0;
        }
    }

    if (empty)
    {
        printf(" none");
    }
}

static void LCD_FillArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    BSP_LCD_SetTextColor(color);
    BSP_LCD_FillRect(x, y, w, h);
}

static void LCD_ShowText(uint16_t x,
                         uint16_t y,
                         sFONT *font,
                         uint16_t textColor,
                         uint16_t backColor,
                         const char *text)
{
    BSP_LCD_SetFont(font);
    BSP_LCD_SetTextColor(textColor);
    BSP_LCD_SetBackColor(backColor);
    BSP_LCD_DisplayStringAt(x, y, (uint8_t *)text, LEFT_MODE);
}

static void LCD_ShowTextCenter(uint16_t x,
                               uint16_t y,
                               uint16_t w,
                               sFONT *font,
                               uint16_t textColor,
                               uint16_t backColor,
                               const char *text)
{
    uint16_t textWidth;
    uint16_t xpos;

    BSP_LCD_SetFont(font);

    textWidth = strlen(text) * font->Width;

    if (w > textWidth)
        xpos = x + (w - textWidth) / 2;
    else
        xpos = x;

    BSP_LCD_SetTextColor(textColor);
    BSP_LCD_SetBackColor(backColor);
    BSP_LCD_DisplayStringAt(xpos, y, (uint8_t *)text, LEFT_MODE);
}

static void LCD_DrawBox(uint16_t x,
                        uint16_t y,
                        uint16_t w,
                        uint16_t h,
                        uint16_t bgColor,
                        uint16_t borderColor)
{
    LCD_FillArea(x, y, w, h, bgColor);

    BSP_LCD_SetTextColor(borderColor);
    BSP_LCD_DrawRect(x, y, w, h);
}

static void LCD_DrawBaseUI(void)
{
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(UI_BG);
    BSP_LCD_SetTextColor(UI_WHITE);
    BSP_LCD_Clear(UI_BG);

    /* Header */
    LCD_FillArea(0, 0, 320, 30, UI_HEADER);
    LCD_ShowTextCenter(0, 7, 320, &Font16, UI_WHITE, UI_HEADER, "Elevator Control System");

    /* Section title */
    LCD_ShowText(10, 112, &Font16, UI_YELLOW, UI_BG, "Pending Requests");

    /* Bottom bar */
    LCD_FillArea(0, 222, 320, 18, UI_PANEL);
}

static uint16_t LCD_DirColor(ElevatorDir_t dir)
{
    if (dir == DIR_UP)
        return UI_GREEN;
    else if (dir == DIR_DOWN)
        return UI_ORANGE;
    else
        return UI_GRAY;
}

static uint16_t LCD_DoorColor(DoorState_t door)
{
    if (door == DOOR_OPEN)
        return UI_GREEN;
    else
        return UI_RED;
}

static void LCD_DrawInfoCard(uint16_t x,
                             uint16_t y,
                             uint16_t w,
                             uint16_t h,
                             const char *title,
                             const char *value,
                             uint16_t valueColor,
                             uint8_t bigFont)
{
    LCD_DrawBox(x, y, w, h, UI_PANEL2, UI_CYAN);

    LCD_ShowTextCenter(x, y + 6, w, &Font12, UI_WHITE, UI_PANEL2, title);

    if (bigFont)
    {
        LCD_ShowTextCenter(x, y + 28, w, &Font24, valueColor, UI_PANEL2, value);
    }
    else
    {
        LCD_ShowTextCenter(x, y + 35, w, &Font16, valueColor, UI_PANEL2, value);
    }
}

static void MakeReqValue(char *buf, uint16_t len, const uint8_t req[5])
{
    uint8_t i;
    uint8_t empty = 1;
    int n = 0;

    if (len == 0)
        return;

    buf[0] = '\0';

    for (i = 1; i <= 4; i++)
    {
        if (req[i])
        {
            n += snprintf(&buf[n], len - n, "%dF ", i);
            empty = 0;

            if (n >= len)
                break;
        }
    }

    if (empty)
    {
        snprintf(buf, len, "none");
    }
}

static void LCD_DrawRequestRow(uint16_t y,
                               const char *label,
                               uint16_t labelColor,
                               const char *value)
{
    uint16_t labelTextColor;

    LCD_FillArea(8, y, 304, 25, UI_PANEL);

    LCD_FillArea(8, y, 58, 25, labelColor);

    if (labelColor == UI_YELLOW || labelColor == UI_GREEN || labelColor == UI_CYAN)
        labelTextColor = UI_BLACK;
    else
        labelTextColor = UI_WHITE;

    LCD_ShowTextCenter(8, y + 5, 58, &Font16, labelTextColor, labelColor, label);
    LCD_ShowText(78, y + 6, &Font12, UI_WHITE, UI_PANEL, value);

    BSP_LCD_SetTextColor(UI_GRAY);
    BSP_LCD_DrawRect(8, y, 304, 25);
}

static void LCD_ShowElevatorState(const ElevatorState_t *s)
{
    static uint8_t uiInited = 0;
    char line[40];
    char reqLine[40];

    if (uiInited == 0)
    {
        LCD_DrawBaseUI();
        uiInited = 1;
    }

    /* Top three info cards */
    snprintf(line, sizeof(line), "%dF", s->currentFloor);
    LCD_DrawInfoCard(8, 38, 96, 66, "FLOOR", line, UI_CYAN, 1);

    snprintf(line, sizeof(line), "%s", DirToStr(s->dir));
    LCD_DrawInfoCard(112, 38, 96, 66, "DIR", line, LCD_DirColor(s->dir), 0);

    snprintf(line, sizeof(line), "%s", DoorToStr(s->door));
    LCD_DrawInfoCard(216, 38, 96, 66, "DOOR", line, LCD_DoorColor(s->door), 0);

    /* Pending request rows */
    MakeReqValue(reqLine, sizeof(reqLine), s->insideReq);
    LCD_DrawRequestRow(134, "IN", UI_PURPLE, reqLine);

    MakeReqValue(reqLine, sizeof(reqLine), s->upReq);
    LCD_DrawRequestRow(164, "UP", UI_GREEN, reqLine);

    MakeReqValue(reqLine, sizeof(reqLine), s->downReq);
    LCD_DrawRequestRow(194, "DOWN", UI_ORANGE, reqLine);

    /* Bottom information */
    LCD_FillArea(0, 222, 320, 18, UI_PANEL);
    snprintf(line, sizeof(line), "Served:%lu   USART:115200",
             (unsigned long)s->servedCount);
    LCD_ShowText(10, 225, &Font12, UI_YELLOW, UI_PANEL, line);
}

static void Display_PrintState(void)
{
    ElevatorState_t s;

    xSemaphoreTake(stateMutexHandle, portMAX_DELAY);
    memcpy(&s, &gElevator, sizeof(ElevatorState_t));
    xSemaphoreGive(stateMutexHandle);

    xSemaphoreTake(uartMutexHandle, portMAX_DELAY);

    printf("\r\n[STATE] floor=%dF, dir=%s, door=%s, served=%lu\r\n",
       s.currentFloor,
       DirToStr(s.dir),
       DoorToStr(s.door),
       (unsigned long)s.servedCount);

    printf("        ");
    PrintReqArray("IN", s.insideReq);
    printf(" | ");
    PrintReqArray("UP", s.upReq);
    printf(" | ");
    PrintReqArray("DOWN", s.downReq);
    printf("\r\n");

    xSemaphoreGive(uartMutexHandle);

    LCD_ShowElevatorState(&s);
}
/* ========================= HAL 中断回调 ========================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t data;

    if (huart->Instance == USART1)
    {
        data = uartRxByte;

        /*
         * 调试用：收到什么就回显什么。
         * 如果不想回显，可以把这几行注释掉。
         */
        if ((USART1->SR & USART_SR_TXE) != 0)
        {
            USART1->DR = data;
        }

        /*
         * 把收到的字节发送到队列，由 UartParseTask 解析。
         */
        if (uartRxQueueHandle != NULL)
        {
            xQueueSendFromISR(uartRxQueueHandle,
                              &data,
                              &xHigherPriorityTaskWoken);
        }

        /*
         * 重新开启下一次 1 字节接收中断。
         * 这句必须有，否则只能接收一次。
         */
        HAL_UART_Receive_IT(&huart1, &uartRxByte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * USART 错误回调：
 * 防止串口溢出或异常后不再接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &uartRxByte, 1);
    }
}

/*
 * 矩阵键盘行中断回调：
 * EXTI 中断里不扫描键盘，只通知 KeyTask。
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == ROW0_Pin ||
        GPIO_Pin == ROW1_Pin ||
        GPIO_Pin == ROW2_Pin ||
        GPIO_Pin == ROW3_Pin)
    {
        if (!keyScanning && keyTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(keyTaskHandle,
                                   &xHigherPriorityTaskWoken);

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* USER CODE END Application */
