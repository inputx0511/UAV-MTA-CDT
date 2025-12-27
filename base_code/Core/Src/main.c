/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "String.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart8;
DMA_HandleTypeDef hdma_uart8_rx;

/* USER CODE BEGIN PV */
#define QUEUE_SIZE 100 // Tăng kích thước nếu cần cập nhật liên tục
#define RINGBUF_SIZE 40 // Kích thước bộ đệm vòng (có thể điều chỉnh)
#define NUM_CHANNELS 3 // Số kênh ADC

//kiểu dữ liệu bool
typedef enum {
    false = 0,
    true = 1
} bool;

//Cấu trúc hàng đợi CAN
typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
} CAN_TxMessageTypeDef;
CAN_TxMessageTypeDef canQueue[QUEUE_SIZE];
uint8_t queueHead = 0, queueTail = 0, queueCount = 0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef Rxmsg;
CAN_FilterTypeDef sFilterConfig;
uint8_t Rxdata[8];
uint8_t Txdata[8];
uint32_t TxMailbox;
//Arduino Nano
uint8_t Rxdata8[32];
uint8_t Rxbuffer8[34];
uint8_t data8;
uint8_t ind8 = 0;

uint8_t display_mode = 0;
uint32_t lastUpdate = 0;
//GAMEPAD
#define GAMEPAD_ADDRESS 0x55<<1
int dir = 0;
int angle;
//=========================Cau truc tay cam==============//
typedef struct {
	  uint8_t isConnected;
	  uint8_t dpad;
	  int32_t aLx;
	  int32_t aLy;
	  int32_t aRx;
	  int32_t aRy;
	  uint16_t buttons;
} Gamepad;
Gamepad gamepad;
int32_t aLx = 0;
int32_t aLy = 0;
int32_t aRx = 0;

// Khai báo biến toàn cục samples
int32_t samples[NUM_CHANNELS][RINGBUF_SIZE] = {0}; // Mảng 2 chiều cho 4 kênh, khởi tạo

const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float T_MIN = -15.0f;
const float T_MAX = 15.0f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float Kp_MIN = 0;
const float Kp_MAX = 500.0f;
const float Kd_MIN = 0;
const float Kd_MAX = 5.0f;
const float W_ratio = 0.2f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */
static inline int32_t read32bitI2C(uint8_t);
static inline int16_t read16bitI2C(uint8_t);
void gamepad_update();
int32_t Read_ADC_Medium(uint8_t, int32_t);
int float_to_uint(float, float, float, unsigned int);
void EnterMode(int);
void ExitMode(int);
void Zero(int);
void base_speed(float, float, float, float);
void motor_speed(int, float);
void motor_position(int, int, int, int);
void Begin();
void Restart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Thêm tin nhắn vào hàng đợi
void push_to_queue(CAN_TxHeaderTypeDef *header, uint8_t *data) {
    if (queueCount >= QUEUE_SIZE) {
        queueTail = (queueTail + 1) % QUEUE_SIZE;
        queueCount--;
    }
    canQueue[queueHead].TxHeader = *header;
    memcpy(canQueue[queueHead].TxData, data, 8);
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount++;
}
//Lấy tin nhắn từ hàng đợi
bool pop_from_queue(CAN_TxMessageTypeDef *msg) {
    if (queueCount == 0) {
        return false; // Hàng đợi rỗng
    }
    *msg = canQueue[queueTail];
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount--;
    return true;
}
// Hàm xử lý hàng đợi
void process_can_queue(void) {
    CAN_TxMessageTypeDef msg;
    if (pop_from_queue(&msg)) {
        if(HAL_CAN_AddTxMessage(&hcan1, &msg.TxHeader, msg.TxData, &TxMailbox) != HAL_OK){
        	push_to_queue(&msg.TxHeader, msg.TxData); // Đẩy lại nếu lỗi
        }
    }
}
static inline int32_t read32bitI2C(uint8_t firstByte){
	return Rxdata8[firstByte] << 24 | Rxdata8[firstByte + 1] << 16 | Rxdata8[firstByte + 2] << 8 | Rxdata8[firstByte + 3];
}
static inline int16_t read16bitI2C(uint8_t firstByte){
	return Rxdata8[firstByte] << 8 | Rxdata8[firstByte + 1];
}

//Cap nhat trang thai tay cam
void gamepad_update(){
    gamepad.isConnected = Rxdata8[0];
    gamepad.buttons = read16bitI2C(26);
    gamepad.aLx = (read32bitI2C(2) - 4)*435/512;
    gamepad.aLy = (-read32bitI2C(6) + 4)*435/512;
    gamepad.aRx = (read32bitI2C(10) - 4)*435/512;

    aLx = Read_ADC_Medium(0,gamepad.aLx);
    aLy = Read_ADC_Medium(1,gamepad.aLy);
    aRx = Read_ADC_Medium(2,gamepad.aRx);

    //Base
	base_speed(aLx - aLy - W_ratio*aRx, aLx + aLy - W_ratio*aRx, -aLx + aLy  - W_ratio*aRx, -aLx - aLy - W_ratio*aRx);
}
//Lấy giá trị trung bình ADC
int32_t Read_ADC_Medium(uint8_t channel, int32_t value) {
    int32_t sum = 0;
    int32_t read_adc_rough = value;
    	for(int i = 0; i < (RINGBUF_SIZE-1); i++){
    		samples[channel][i] = samples[channel][i+1];
    		sum += samples[channel][i];
    	}
    	samples[channel][RINGBUF_SIZE-1] = read_adc_rough;
    	return (int32_t)(sum/(RINGBUF_SIZE-1));
    }
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max-x_min;
    float _offset = x_min;
    int pgg = 0;
    if(bits == 12){
    	pgg = (unsigned int) ((x-_offset)*4095.0/span);
    }
    if(bits == 16){
        pgg = (unsigned int) ((x-_offset)*65535.0/span);
    	}
    return pgg;
}

//Khoi dong dong co Cubemars
void EnterMode(int id){
    Txdata[0] = 0xFF;
    Txdata[1] = 0xFF;
    Txdata[2] = 0xFF;
    Txdata[3] = 0xFF;
    Txdata[4] = 0xFF;
    Txdata[5] = 0xFF;
    Txdata[6] = 0xFF;
    Txdata[7] = 0xFC;
    TxHeader.StdId = id;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txdata, &TxMailbox);
    HAL_Delay(5);
 }
 //Dung dong co Cubemars
 void ExitMode(int id){
    Txdata[0] = 0xFF;
    Txdata[1] = 0xFF;
    Txdata[2] = 0xFF;
    Txdata[3] = 0xFF;
    Txdata[4] = 0xFF;
    Txdata[5] = 0xFF;
    Txdata[6] = 0xFF;
    Txdata[7] = 0xFD;
    TxHeader.StdId = id;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txdata, &TxMailbox);
    HAL_Delay(5);
}
//Quy 0 dong co Cubemars
void Zero(int id){
    HAL_Delay(3);
    Txdata[0] = 0xFF;
    Txdata[1] = 0xFF;
    Txdata[2] = 0xFF;
    Txdata[3] = 0xFF;
    Txdata[4] = 0xFF;
    Txdata[5] = 0xFF;
    Txdata[6] = 0xFF;
    Txdata[7] = 0xFE;
    TxHeader.StdId = id;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txdata, &TxMailbox);
    HAL_Delay(5);
}
//Dieu khien toc do robot
void base_speed(float v1, float v2, float v3, float v4){
    motor_speed(1,v1);
   	motor_speed(2,v2);
   	motor_speed(3,v3);
   	motor_speed(4,v4);
}
//Dieu khien toc do dong co
void motor_speed(int id, float velocity){
    //-429 <= velocity <= 429
    if(velocity >= 100){
    	velocity = 100;
    }
    else if(velocity <= -100){
    	velocity = -100;
    }

    float v = velocity*M_PI/30; //v (rad/s) , velocity(rpm)

    int p_int = float_to_uint(0, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int Kp_int = float_to_uint(0, Kp_MIN, Kp_MAX, 12);
    int Kd_int = float_to_uint(0.1, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(0, T_MIN, T_MAX, 12);

    Txdata[0] = p_int>>8;
    Txdata[1] = p_int & 0xFF;
    Txdata[2] = v_int>>4;
    Txdata[3] = ((v_int&0xF)<<4)|(Kp_int>>8);
    Txdata[4] = Kp_int&0xFF;
    Txdata[5] = Kd_int>>4;
    Txdata[6] = ((Kd_int&0xF)<<4)|(t_int>>8);
    Txdata[7] = t_int & 0xFF;
    TxHeader.StdId = id;
    push_to_queue(&TxHeader, Txdata);
}


//Ham khoi dong
void Begin(){
	HAL_Delay(3000);
    HAL_CAN_Start(&hcan1);
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    HAL_UART_Receive_DMA(&huart8, &data8, 1);
    HAL_Delay(2500);
    //Quy 0 dong co Cubemars
    for(int i = 1; i < 6; i++){
    	Zero(i);
    		HAL_Delay(100);
    }
    	//Khoi dong dong co de Cubemars
    	for(int i = 1; i < 6; i++){
    	  EnterMode(i);
    	  HAL_Delay(100);
    	}
    }

void Restart(){
    	HAL_CAN_Start(&hcan1);
    	TxHeader.IDE = CAN_ID_STD;
    	TxHeader.RTR = CAN_RTR_DATA;
    	TxHeader.DLC = 8;
    	TxHeader.TransmitGlobalTime = DISABLE;

 }
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    	{
    	if(huart -> Instance == UART8){ //Arduino Nano
    			if(ind8 == 0 && data8 == 0xAA) { // Kiểm tra header
    			  Rxbuffer8[ind8++] = data8;
    			  Rxbuffer8[33] = 0x00;
    			}
    			else if (ind8 > 0 && ind8 < 33) { // Nhận 32 byte dữ liệu
    				Rxbuffer8[ind8++] = data8;
    				Rxbuffer8[33] += data8;
    			}
    			else if(ind8 == 33 && data8 == Rxbuffer8[33]){ //so sánh CRC
    				memcpy(Rxdata8, &Rxbuffer8[1], 32);
    				ind8 = 0; //Reset nếu nhận đủ
    			}
    			else {
    				  ind8 = 0; // Reset nếu header sai
    			}
    			HAL_UART_Receive_DMA(&huart8, &data8, 1);
    		}
    	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  Begin();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  process_can_queue();
	  if(HAL_GetTick() - lastUpdate >= 5){
	 		  if(Rxdata8[0] == 1){
	 			  gamepad_update();
	 		  }
	 		  else{
	 			  base_speed(0,0,0,0);
	 		  }
	 		  lastUpdate = HAL_GetTick();
	 	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 57600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
