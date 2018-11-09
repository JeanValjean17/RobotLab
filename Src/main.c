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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;

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
static void DistanceSensorReading(void const * argument);
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

    digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_NOPULL);
    digital_configure_pin(DD_PIN_PC8, DD_CFG_INPUT_NOPULL);
    digital_configure_pin(DD_PIN_PC13, DD_CFG_OUTPUT);
    digital_configure_pin(DD_PIN_PD15, DD_CFG_INPUT_PULLUP);

    digital_set_pin(DD_PIN_PC13, DD_LEVEL_HIGH);

    ft_start_sampling(DD_PIN_PD14);

    //Here HIGH actually means low...
    led_red(DD_LEVEL_HIGH);
    led_green(DD_LEVEL_HIGH);

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */

    //xTaskCreate((TaskFunction_t) Behaviour, "Behaviour", 128, NULL, 1, NULL);
    xTaskCreate((TaskFunction_t) DistanceSensorReading, "SensorReading", 128,
    NULL, 4, NULL);
    xTaskCreate((TaskFunction_t) IRSensorTest, "IRSensorReadings", 128, NULL, 3,
    NULL);
    xTaskCreate((TaskFunction_t) MotorControl, "MotorControl", 128, NULL, 4,
            NULL);
    xTaskCreate((TaskFunction_t) DefaultIdle, "Idle", 64, NULL, 0, NULL);

    //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Start scheduler */
    //osKernelStart();
    vTaskStartScheduler();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /* USER CODE END 3 */

}

/* StartDefaultTask function */
static void Behaviour(void const * argument)
{

    /* USER CODE BEGIN 5 */

    /* Infinite loop */
    for (;;)
    {
        //sensorValue = adc_get_value(DA_ADC_CHANNEL0);
        //led_green_toggle();
        //traces("toogle led\r\n");   //print debug message
        //sensorValue = adc_get_value(DA_ADC_CHANNEL0);
        tracef("Behaviour \r\n");
        vTaskDelay(20);       //delay the task for 20 ticks (1 ticks = 50 ms)
    }
    /* USER CODE END 5 */
}

/// -30,30,0 goes in straight line but third wheel lags. When third wheel is
/// not zero it'll just take a turn.
///
/// @param argument
static void MotorControl(void const * argument)
{
    while (1)
    {
        if (digital_get_pin(DD_PIN_PD15))
        {
            motor_set(DM_MOTOR0, 0);
            motor_set(DM_MOTOR1, 0);
            motor_set(DM_MOTOR2, 0);
        }
        else
        {
            motor_set(DM_MOTOR0, -30);
            motor_set(DM_MOTOR1, 30);
            motor_set(DM_MOTOR2, -20);
        }

        /*motor_set(DM_MOTOR0, 20);
         motor_set(DM_MOTOR1, 20);
         motor_set(DM_MOTOR2, 20);*/
        //uint8_t motorSpeed = motor_get_speed(DM_MOTOR0);
        //tracef("Motor Speed :  %d\r\n", motorSpeed);
        tracef("MotorControl \r\n");
        vTaskDelay(100);
    }
}

static void IRSensorTest(void const * argument)
{
    uint32_t sensorValue = 0;
    float voltageRead = 0;

    while (1)
    {
        uint8_t levelPin = digital_get_pin(DD_PIN_PD14);
        uint8_t levelPin2 = digital_get_pin(DD_PIN_PC8);

       /* tracef(" [Target Sensor] Pin Level : %d,  PIN 2   %d\r\n", levelPin,
                levelPin2);*/

        if (ft_is_sampling_finished())
        {
            uint16_t freq = ft_get_transform(DFT_FREQ100);
            uint8_t levelPin = digital_get_pin(DD_PIN_PD14);
            uint8_t levelPin2 = digital_get_pin(DD_PIN_PC8);
            tracef( " [Target Sensor] Pin Level : %d,  Freq Read: %d PIN 2   %d\r\n",
                   levelPin, freq, levelPin2);

            //TODO Filter maybe?
            freq > 1500 ? led_red(DD_LEVEL_LOW) : led_red(DD_LEVEL_HIGH);

            //led_red_toggle();
            ft_start_sampling(DD_PIN_PD14);

        }
        uint8_t switchLevel = digital_get_pin(DD_PIN_PD15);

        tracef(" [Switch] Switch Level : %d \r\n", switchLevel);
        tracef("IRSensorTest \r\n");

        vTaskDelay(50);
    }
}

static void DistanceSensorReading(void const * argument)
{
    uint32_t sensorValue = 0;
    float voltageRead = 0;

    while (1)
    {

        sensorValue = 0;
        for (uint8_t i = 0; i < 10; i++)
        {
            sensorValue += adc_get_value(DA_ADC_CHANNEL0);
        }

        sensorValue /= 10;

        voltageRead = (sensorValue * 3.6) / 4095;
        tracef("Voltage: %g, Sensor : %d\r\n", voltageRead, sensorValue);
        tracef("DistanceSensorReading \r\n");
        vTaskDelay(100);       //delay the task for 20 ticks (1 ticks = 50 ms)
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
