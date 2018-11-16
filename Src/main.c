/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "dorobo32.h"
#include "wifi.h"
#include "adc.h"
#include "trace.h"
#include "fft.h"
#include "motor.h"
#include "Movement.h"
#include "ObstacleSensor.H"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;
int8_t mov_velocity = 0;
enum RobotMovement robotMovDirection = Mov_Stop;
Bumpers bumperSensors;
DistanceSensor distanceSensors;
SemaphoreHandle_t xSemaphore;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Behaviour(void const * argument);
static void ObstacleAvoidanceSensors(void const * argument);
static void IRSensorTest(void const * argument);
static void MotorControl(void const * argument);
static void DefaultIdle(void const * argument);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    dorobo_init();
    wifi_init();
    adc_init();

    motor_init();

    SetUpSensors();

    InitMotorControl();

    // Digital pins for target sensors
    digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_NOPULL);

    digital_configure_pin(DD_PIN_PC8, DD_CFG_INPUT_NOPULL);

    ft_start_sampling(DD_PIN_PD14);

    //Here HIGH actually means low...

    xSemaphore = NULL;

    xSemaphore = xSemaphoreCreateMutex();

    /* Create the thread(s) */
    /* definition and creation of defaultTask */

    xTaskCreate((TaskFunction_t) Behaviour, "Behaviour", 128, NULL, 1, NULL);
    xTaskCreate((TaskFunction_t) ObstacleAvoidanceSensors, "SensorReading", 128, NULL, 2, NULL);
    //xTaskCreate((TaskFunction_t) IRSensorTest, "IRSensorReadings", 128, NULL, 1, NULL);
    //xTaskCreate((TaskFunction_t) MotorControl, "MotorControl", 128, NULL, 4, NULL);
    //xTaskCreate((TaskFunction_t) DefaultIdle, "Idle", 64, NULL, 0, NULL);

    /* Start scheduler */
    //osKernelStart();
    vTaskStartScheduler();

}

/* StartDefaultTask function */
static void Behaviour(void const * argument)
{

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 300 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        tracef("[Measured Distance]: Right Sensor  %d   Left Sensor %d \r\n", distanceSensors.Right,
                                distanceSensors.Left);
        tracef("Behaviour \r\n");

    }

}

static void MotorControl(void const * argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    MovementControl _motorControl;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xSemaphore != NULL)
        {
            if (xSemaphoreTake(xSemaphore, ( TickType_t ) 10 ) == pdTRUE)
            {
                _motorControl = ReadMovementToExecute();
                led_red_toggle();
                switch (_motorControl.direction)
                {
                    case Mov_Straight:
                        mov_velocity = MoveStraight(mov_velocity);
                        break;
                    case Mov_Back:
                        mov_velocity = MoveBack(mov_velocity);
                        break;
                    case Mov_Rot_Left:
                        mov_velocity = RotateLeft(mov_velocity);
                        break;
                    case Mov_Rot_Right:
                        mov_velocity = RotateRight(mov_velocity);
                        break;
                    case Mov_Stop:
                    default:
                        mov_velocity = StopMovement();
                        break;
                }
                xSemaphoreGive(xSemaphore);
            }
            else
            {
                tracef("\r\n [Motor Control] Semaphore taken by another Task \r\n");
            }

        }
        else
        {
            tracef("\r\n [Motor Control] Mutex NULL \r\n");
        }

        tracef("MotorControl \r\n");
        //osDelay(250);
    }
}

static void IRSensorTest(void const * argument)
{
    uint32_t sensorValue = 0;
    float voltageRead = 0;

    while (1)
    {
        uint8_t levelPin = 0;
        uint8_t levelPin2 = 0;

        /* tracef(" [Target Sensor] Pin Level : %d,  PIN 2   %d\r\n", levelPin,
         levelPin2);*/

        if (ft_is_sampling_finished())
        {
            uint16_t freq = ft_get_transform(DFT_FREQ100);
            for (uint8_t i = 0; i < 100; i++)
            {
                levelPin += digital_get_pin(DD_PIN_PD14);
                levelPin2 += digital_get_pin(DD_PIN_PC8);
            }

            levelPin /= 100;
            levelPin2 /= 100;

            tracef(" [Target Sensor] Pin Level : %d,  Freq Read: %d PIN 2   %d\r\n", levelPin, freq, levelPin2);

            //TODO Filter maybe?
            freq > 1500 ? led_red(DD_LEVEL_LOW) : led_red(DD_LEVEL_HIGH);

            //led_red_toggle();
            ft_start_sampling(DD_PIN_PD14);

        }
        uint8_t switchLevel = digital_get_pin(DD_PIN_PD15);

        tracef(" [Switch] Switch Level : %d \r\n", switchLevel);
        tracef("IRSensorTest \r\n");

        vTaskDelay(500);
    }
}

static void ObstacleAvoidanceSensors(void const * argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 300 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bumperSensors = ReadBumperSensors();

        distanceSensors = ReadDistanceSensors();

        //tracef("[Bumpers] left Bumper:  %d, right Bumper:  %d \r\n ", bumperSensors.Left, bumperSensors.Right);

        tracef("Sensor Value Right RAW:   %d   Sensor Left %d \r\n", distanceSensors.RightRawValue,
                distanceSensors.LeftRawValue);


    }
}

static void DefaultIdle(void const * argument)
{
    while (1)
    {
        tracef("DefaultIdle \r\n");
        vTaskDelay(100);
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
