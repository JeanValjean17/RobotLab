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
#include "IRSensor.h"

/* Global variables access by tasks ---------------------------------------------------------*/
int8_t mov_velocity = 0;
enum RobotMovement robotMovDirection = Mov_Stop;
Bumpers bumperSensors;
DistanceSensor distanceSensors;
SemaphoreHandle_t xSemaphore;
bool directionBothDistanceDetection = false;
IRSensors irSensors;
uint8_t counterCorner = 0;

/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void Behaviour(void const * argument);
static void ObstacleAvoidanceSensors(void const * argument);
static void IRSensor(void const * argument);
static void MotorControl(void const * argument);

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    dorobo_init();
    wifi_init();
    adc_init();

    motor_init();

    irSensors.leftValue = 1;
    irSensors.oldLeftValue = 1;
    irSensors.rightValue = 1;
    irSensors.oldRightValue = 1;

    SetUpSensors();

    InitMotorControl();

    // Digital pins for target sensors
    digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_NOPULL);

    digital_configure_pin(DD_PIN_PC8, DD_CFG_INPUT_NOPULL);

    ft_start_sampling(DD_PIN_PD14);

    // Creating mutexes

    xSemaphore = NULL;

    xSemaphore = xSemaphoreCreateMutex();

    /* Create the thread(s) */

    xTaskCreate((TaskFunction_t) Behaviour, "Behaviour", 128, NULL, 2, NULL);
    xTaskCreate((TaskFunction_t) ObstacleAvoidanceSensors, "SensorReading", 128, NULL, 1, NULL);
    xTaskCreate((TaskFunction_t) IRSensor, "IRSensorReadings", 128, NULL, 1, NULL);
    xTaskCreate((TaskFunction_t) MotorControl, "MotorControl", 128, NULL, 3, NULL);

    /* Start scheduler */
    vTaskStartScheduler();

}

/**
 * Task executing arpox. each 300 ms. This taks reads sensors information and writes accordingly behaviour to
 * the motor task for the movement.
 * @param argument
 */
static void Behaviour(void const * argument)
{

    // Frequency control so the task can execute each 300 ms.
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xSemaphore != NULL)
        {
            // Asking if mutex is free, otherwise it won't execute until it is free.
            if (xSemaphoreTake(xSemaphore, ( TickType_t ) 10 ) == pdTRUE)
            {
                // Bumpers not detecting. Logic is inverted
                if (bumperSensors.Left && bumperSensors.Right)
                {
                    // Left sensor detecting
                    if (counterCorner > 5)
                    {
                        WriteMovement(Mov_Back, 4);
                    }
                    else if (distanceSensors.LeftRawValue > 1100 && distanceSensors.RightRawValue < 1300)
                    {
                        tracef("\r\nMov_Rot_Right");
                        WriteMovement(Mov_Rot_Right, 1);
                        counterCorner++;
                    }
                    //Right sensor detecting
                    else if (distanceSensors.RightRawValue > 1300 && distanceSensors.LeftRawValue < 1100)
                    {
                        tracef("\r\nMov_Rot_Left");
                        WriteMovement(Mov_Rot_Left, 1);
                        counterCorner++;
                    }
                    //Both sensors detecting
                    else if (distanceSensors.LeftRawValue > 1100 && distanceSensors.RightRawValue > 1300)
                    {
                        tracef("\r\nBoth detecting MOVE LEFT");
                        WriteMovement(Mov_Rot_Left, 1);
                        counterCorner++;
                    }
                    else
                    {
                        if (!irSensors.leftValue && irSensors.rightValue)
                        {
                            tracef("\r\n Move LEft Target LEFT \r\n");
                            WriteMovement(Mov_Rot_Left, 1);
                        }
                        else if (irSensors.leftValue && !irSensors.rightValue)
                        {
                            tracef("\r\n Move LEft Target RIGHT \r\n");
                            WriteMovement(Mov_Rot_Right, 1);
                        }
                        else
                        {
                            tracef("\r\nMov_Straight Nothing Detected\r\n");
                            WriteMovement(Mov_Straight, 1);
                            counterCorner = 0;
                        }
                    }
                }
                else if ((!bumperSensors.Left && !bumperSensors.Right) || (bumperSensors.Left && !bumperSensors.Right))
                {
                    WriteMovement(Mov_Back, 3);
                    counterCorner = 0;
                }
                else if (!bumperSensors.Left && bumperSensors.Right)
                {
                    WriteMovement(Mov_BackRight, 3);
                    counterCorner = 0;
                }
                // Frees up mutex for other tasks to use it
                xSemaphoreGive(xSemaphore);
            }
        }
        //tracef("[Bumper sensors]: Right Sensor  %d   Left Sensor %d \r\n", bumperSensors.Right, bumperSensors.Left);

        //tracef("[Raw Value]: Right Sensor  %d   Left Sensor %d \r\n", distanceSensors.RightRawValue,
        //distanceSensors.LeftRawValue);

        tracef("\r\n leftValue:  %d, rightValue: %d\r\n", irSensors.leftValue, irSensors.rightValue);

        //tracef("\r\n[IR Sensor Freq Value]:  %d \r\n", irSensors.freq);
    }
}

/**
 * Reads the movement to executed based on the results from the Behaviour task and sends it to the motors.
 * @param argument
 */
static void MotorControl(void const * argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 200 / portTICK_PERIOD_MS; //500
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
                    case Mov_BackRight:
                        mov_velocity = BackAndRight(mov_velocity);
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
    }
}

/**
 *
 * @param argument
 */
static void IRSensor(void const * argument)
{
    uint32_t sensorValue = 0;
    float voltageRead = 0;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool leftSensor = 0;
        bool rightSensor = 0;

        if (ft_is_sampling_finished())
        {
            // Freq only works in one pin. Right now set up to work on PD14 pin.

            uint16_t freq = ft_get_transform(DFT_FREQ100);

            leftSensor = digital_get_pin(DD_PIN_PD14);
            rightSensor = digital_get_pin(DD_PIN_PC8);

            /*tracef("\r\n digPinLeft:  %d, digPinRight:  %d oldValueLeft:  %d oldRightValue:  %d\r\n", leftSensor,
             rightSensor, irSensors.oldLeftValue, irSensors.oldRightValue);*/

            irSensors.leftValue = leftSensor;
            irSensors.rightValue = rightSensor;
            irSensors.freq = freq;

            /*tracef(" [Target Sensor] Freq Read: %d Left Sensor %d Right Sensor %d \r\n", freq, irSensors.leftDetection,
             irSensors.rightDetection);*/

            //TODO Filter maybe?
            freq > 1500 ? led_red(DD_LEVEL_LOW) : led_red(DD_LEVEL_HIGH);

        }

        tracef("\r\n IRSensor \r\n");

    }
}

/**
 * Reads the obstacle sensors, namely; bumper and distance sensors.
 * @param argument
 */
static void ObstacleAvoidanceSensors(void const * argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bumperSensors = ReadBumperSensors();

        distanceSensors = ReadDistanceSensors();

        //tracef("[Bumpers] left Bumper:  %d, right Bumper:  %d \r\n ", bumperSensors.Left, bumperSensors.Right);

        //tracef("Sensor Value Right RAW:   %d   Sensor Left %d \r\n", distanceSensors.RightRawValue,
        // distanceSensors.LeftRawValue);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
