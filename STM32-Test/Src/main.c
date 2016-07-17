/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//! Makro for the delay times
#define CAN_TRANSMIT_DELAY (100)
#define LED_TICKS (250)

//! Struktur auf alle Timer-Ticks Variablen  (maximal 255 Variablen)
typedef volatile struct timeoutticks_st {
    uint32_t canTransmitDelay;          // Verzögerungszeit bis nächste Tx-Message gesendet wird
    uint32_t ledTicks;                  // ticks für led blinken
} timeoutticks_t;

//! Status Flags from STM32-CAN device
typedef volatile struct stm32Status_st {
    interface_flags_t interface_flags;
    vu16 TimeStampCounter;
  
    uint32_t usbConnected ;              // set to 1 then stm32 is connected to usb and initialized
    
} stm32Status_t;


static timeoutticks_t TimeoutTicks = {     
    CAN_TRANSMIT_DELAY          // this have to be always the first entry
,   LED_TICKS   
};

static timeoutticks_t * const pTicks = &TimeoutTicks;  // I like pointers, so I use pointers :-)

static stm32Status_t stm32Status = {
    { 0,0,0,0 }     //flags
,   0               // timestamp counter        
,   0               // usb connected, not used    
};

static stm32Status_t * const pStatus = &stm32Status;


static CanTxMsgTypeDef myTxMessage;
static CanRxMsgTypeDef myRxMessage;
static CAN_FilterConfTypeDef myFilter;

unsigned char usbHelloMsg[]= "\nGreetings from USB device";

unsigned char usbLine[50];


volatile unsigned char myLastLine[MAX_USB_RECEIVED_BUFFER][MAX_USB_RECEIVED_BUFFER_LEN+1];   // Ablage Commandas Strings die per USB empfangen werden (+1 für die abschliesende 0 für Strings)
volatile unsigned char strReceivedCommando[MAX_USB_RECEIVED_BUFFER_LEN+1];                   // Buffer für eine komplette Zeile
volatile unsigned char strSendCommando[MAX_USB_RECEIVED_BUFFER_LEN+1];                       // eigneer Sendebuffer zum Test da eventuell die gemeinsame Nutzung von strReceivedCommando nicht funktioniert

volatile unsigned char myLastLinePosIdx = 0;     ;  // Spaltenindex von USB STring

// aa test
volatile unsigned char strStatusCommand[5];

volatile unsigned char myWriteIndexLastLine = 0  ;  //SchreibIndesx für den USB STring, wenn beide indexe gleich sind, dann liegt keine neue Nachricht im Buffer
volatile unsigned char myReadIndexLastLine  = 0  ;  // Lese Index für den USB




volatile enBitrate enCanBaudrate = _500_kbit;   // Merker des Indexes auf die Baudrate 0=500 1 250 ...usw.

//! Baudraten für den Sx Laviel-Protokoll setting Baudrate befehl   (Nummer von S ist der Index fürs Array)
const enBitrate usbBaudSettings[_LAST_BAUDRATE_]= {
    _10_kbit    // S0
,   _20_kbit    // S1
,   _50_kbit    // ...
,   _100_kbit
,   _125_kbit
,   _250_kbit
,   _500_kbit
,   _33_kbit
,   _95_kbit
,   _83_kbit   // S9
,   _47_kbit   // S10   not compatible to CAN-Hacker    
 };

const Bitrate_st stBitrate[COUNT_BITRATE]= {  // Reihenfolge ergibt sich von der Enumaration von  enum bitrate_t
//    PRESCal     BS1  BS2        // todo für stm
     { 6    , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 500kbit   S6  diese Werte gelten für einen Systemtakt von 72Mhz  PLCK1=32 MHz                         
,    { 12   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 250kbit   S5                            
,    { 24   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 125kbit   S4                            
,    { 30   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 100kbit   S3                            
,    { 42   , CAN_BS1_6TQ, CAN_BS2_2TQ}  // 95kbit                               
,    { 36   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 83kbit    S9                              
,    { 60   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 50kbit  S2                             
,    { 90   , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 33kbit                               
,    { 150  , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 20kbit  S1 
,    { 300  , CAN_BS1_8TQ, CAN_BS2_3TQ}  // 10kbit  S0   
,    { 42   , CAN_BS1_12TQ, CAN_BS2_5TQ}  // 47,619kbit  S10   
};

 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_WWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_SYSTICK_Callback(void);  // handle own additional systicks
static void USER_CANx_Init(CAN_HandleTypeDef *hCANx,enBitrate baud, uint32_t canMode);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    uint32_t i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_WWDG_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  
    USER_CANx_Init(&hcan1,enCanBaudrate,CAN_MODE_NORMAL);   // overwrite the cubeMX initialization
    USER_CANx_Init(&hcan2,enCanBaudrate,CAN_MODE_NORMAL);   // first init it with 100kbit   todo: remvoe onyl init from lawicel opoen commands

    HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
    
    HAL_GPIO_WritePin(CAN_RS_GPIO_Port,CAN_RS_Pin,GPIO_PIN_RESET);  // enable 82C250  HS


   
    
    hcan1.pTxMsg = &myTxMessage;
    
    myTxMessage.DLC = 4;
    myTxMessage.StdId = 0x234;
    
    myTxMessage.IDE = CAN_ID_STD;
    
    
    HAL_CAN_Transmit_IT(&hcan1);
    
    myFilter.FilterNumber           = 0;
    myFilter.FilterMode             = CAN_FILTERMODE_IDMASK;
    myFilter.FilterScale            = CAN_FILTERSCALE_32BIT;
    myFilter.FilterIdHigh           = 0x0000;
    myFilter.FilterIdLow            = 0x0000;
    myFilter.FilterMaskIdHigh       = 0x0000;
    myFilter.FilterMaskIdLow        = 0x0000;
    myFilter.FilterFIFOAssignment   = 0;
    myFilter.FilterActivation       = ENABLE;
    
    HAL_CAN_ConfigFilter(&hcan1,&myFilter);
    

    hcan1.pRxMsg=  &myRxMessage;
    if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) {
        /* Reception Error */
        HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);
    }    
    

    myTxMessage.DLC = 3;
    myTxMessage.ExtId = 0x14F00500;
    
    myTxMessage.IDE = CAN_ID_EXT;
    myTxMessage.Data[0] = 0xAA;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      
    if(!pTicks->ledTicks) {
        pTicks->ledTicks = LED_TICKS;

        if(!pStatus->interface_flags.CanChannelOnOff) {  // If the CAN channel is closed, the red LED is toggled.
            HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
        }
        else {
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        }
    }

    if(!pTicks->canTransmitDelay) { // test for sending a cylic can message   , todo aw: remove it later
        pTicks->canTransmitDelay = CAN_TRANSMIT_DELAY;
        
        HAL_CAN_Transmit_IT(&hcan1);

        // CDC_Transmit_FS(usbHelloMsg,strlen((const char *)usbHelloMsg));

    }
    

    if(myWriteIndexLastLine!=myReadIndexLastLine) {  // new lawicel comand string received?

        i= sprintf((char*)strReceivedCommando,"%s",myLastLine[myReadIndexLastLine]);  // yes,  copy command from buffer as string

        // for first usb testing, send a pong 
        #ifdef PING_PONG
            CDC_Transmit_FS((unsigned char *)strReceivedCommando,i);   // CAN-Hacker don't want echos :-)
        #endif

        switch (strReceivedCommando[0]) {           // execute Lawicel commands

            case 'V':
                CDC_Transmit_FS((unsigned char *) STM32_CAN_SW_VERSION, sizeof(STM32_CAN_SW_VERSION)-1 );
            break;
        
            case 'v':
                CDC_Transmit_FS((unsigned char *) STM32_CAN_HW_VERSION, sizeof(STM32_CAN_HW_VERSION)-1 );
            break;

            /* Place for the other Lawicel commands :-)
            
            
            */
            
            case 'S':  // set Baudrat S0..S9  , only allowed the CAN channel is closed
                if ( (strReceivedCommando[1] >='0') &&  (strReceivedCommando[1] <='9') && (!pStatus->interface_flags.CanChannelOnOff)  ) {  // parmeter okay? and channel closed?
                    SetCanBaudrate(usbBaudSettings[strReceivedCommando[1]-0x30]);
                    pStatus->interface_flags.BaudSettingsReceived=1;

                    strReceivedCommando[0]=0x0D;             
                    CDC_Transmit_FS((unsigned char *)strReceivedCommando,1);
                }
                else {                         // wrong Parameter  ==> 0x07 senden
                    strReceivedCommando[0]=0x07;             
                    CDC_Transmit_FS((unsigned char *)strReceivedCommando,1);
                    
                    pStatus->interface_flags.BaudSettingsReceived=0;  // wegen falscher Baudrate zurücksetzen
                }                    
            break;
                
            case 's':  //Benutzerdefinierete Baudrate setzen  (erstmal nur s8333 für 83.33kbit und s3333 für 33,33kbit)
                if (    (strReceivedCommando[1] =='8') && (strReceivedCommando[2] =='3') \
                    &&  (strReceivedCommando[3] =='3') && (strReceivedCommando[4] =='3') \
                    && (!pStatus->interface_flags.CanChannelOnOff)  ) {  // valid parameter and channel closed?
                    SetCanBaudrate(usbBaudSettings[_83_kbit]);
                    pStatus->interface_flags.BaudSettingsReceived=1;

                    strReceivedCommando[0]=0x0D;             
                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
                else {
                    if ( (strReceivedCommando[1] =='3') &&  (strReceivedCommando[2] =='3') \
                     &&  (strReceivedCommando[3] =='3') &&  (strReceivedCommando[4] =='3') \
                     &&  (!pStatus->interface_flags.CanChannelOnOff)  ) {  // gültige Zahl und Kanal geschlossen?
                        SetCanBaudrate(usbBaudSettings[_33_kbit]);
                        pStatus->interface_flags.BaudSettingsReceived=1;

                        strReceivedCommando[0]=0x0D;             
                        CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                    }
                    else {
                        if ( (strReceivedCommando[1] =='9') &&  (strReceivedCommando[2] =='5') \
                         &&  (strReceivedCommando[3] =='0') &&  (strReceivedCommando[4] =='0') \
                         &&  (!pStatus->interface_flags.CanChannelOnOff)  ) {  // gültige Zahl und Kanal geschlossen?
                            SetCanBaudrate(usbBaudSettings[_95_kbit]);
                            pStatus->interface_flags.BaudSettingsReceived=1;

                            strReceivedCommando[0]=0x0D;             
                            CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                        }
                        else {
                           // wrong parameter ==> send 0x07 
                            strReceivedCommando[0]=0x07;             
                            CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                            
                            // todo flag setzen, dass fehlerhafte Baudrate empfangen wurde und somit der OpenBefehl nicht ausgefürht werden darf,
                            // solange nicht eine korrekte benutzerdefinierte Baudreate bzw. ein S1..S9 empfangen wurde
                            pStatus->interface_flags.BaudSettingsReceived=0;  // because of wrong baudrate received don't execute "Channel Open"
                        }
                    }                    
                }
            break;
                
                
            case 'O':    // open CAN channel ( channel has to be closed)
                pStatus->interface_flags.CanSilentModeOnOff = 0x00;  // normaler Modus
                pStatus->TimeStampCounter=0;
                if((!pStatus->interface_flags.CanChannelOnOff) && (pStatus->interface_flags.BaudSettingsReceived)) { // Kanal geschlossen und Baudrate schon empfangen?

                    USER_CANx_Init(&hcan1,enCanBaudrate,CAN_MODE_NORMAL); 
                    USER_CANx_Init(&hcan2,enCanBaudrate,CAN_MODE_NORMAL); 

                    pStatus->interface_flags.CanChannelOnOff=1;  // set flag to status "opened"
                    strReceivedCommando[0]=0x0D;             
                    
                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
                else {  // already opened
                    strReceivedCommando[0]=0x07;  

                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
            break;
    

            case 'C':       // close CAN channel    (channel has to be opened, befor it can be closed)
                if(pStatus->interface_flags.CanChannelOnOff) { // channel opened?

                    // todo aw: use also a macro to select which CANs are used, see above for the old version without HAL
                    HAL_CAN_DeInit(&hcan1);  // Deinit both CANs
                    HAL_CAN_DeInit(&hcan2);
    
                    pStatus->interface_flags.CanChannelOnOff=0;  // set flag to status "closed"
                    
                    strReceivedCommando[0]=0x0D;             
                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 

                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
                }
                else {  // already closed
                    strReceivedCommando[0]=0x07;             
                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
            break;
                
                
            case 'L':  //listen only mode , only allowed to set when the CAN channel is closed
                    
                pStatus->interface_flags.CanSilentModeOnOff = 0x01;
                pStatus->TimeStampCounter=0;
            
                if((!pStatus->interface_flags.CanChannelOnOff) && (pStatus->interface_flags.BaudSettingsReceived)) { // Kanal geschlossen und Baudrate schon empfangen?
                    // todo aw : use macros for the available CAN channels
                    USER_CANx_Init(&hcan1,enCanBaudrate,CAN_MODE_SILENT); 
                    USER_CANx_Init(&hcan2,enCanBaudrate,CAN_MODE_SILENT); 

                    pStatus->interface_flags.CanChannelOnOff=1;  
                    strReceivedCommando[0]=0x0D;             
                    
                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
                else {  // already opened
                    strReceivedCommando[0]=0x07;  

                    CDC_Transmit_FS((u8 *)strReceivedCommando,1); 
                }
            break;
                
            case 'Z':  // set timestamp status
                strReceivedCommando[0]=0x0D;  
                if(strReceivedCommando[1]=='1') {
                    pStatus->interface_flags.TimeStampOnOff=1;
                }
                else {
                    if (strReceivedCommando[1]=='0') {
                        pStatus->interface_flags.TimeStampOnOff=0;
                    }
                    else {  // unknown status, set error code for the response
                        strReceivedCommando[0]=0x07;  
                    }
                }
                CDC_Transmit_FS((u8 *)strReceivedCommando,1); 

            break;
                
            case 0x07:                              // Lawicel: error received?  also set by receiving more than 20 chars without [CR]
                CDC_Transmit_FS((unsigned char *)strReceivedCommando,1);
            
            default:
                strReceivedCommando[0]=0x07;        // send error back by receiving unknown command
                CDC_Transmit_FS((unsigned char *)strReceivedCommando,1);
            
            break;
         }   // end switch lawicel commands

        if(myReadIndexLastLine < (MAX_USB_RECEIVED_BUFFER - 1)) {   
            myReadIndexLastLine += 1;                                  
        }
        else {
            myReadIndexLastLine = 0;
        }
    }  // end new command string received
  }
  
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_8TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_8TQ;
  hcan2.Init.BS2 = CAN_BS2_3TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);

}

/* WWDG init function */
void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 0x40;
  hwwdg.Init.Counter = 0x40;
  HAL_WWDG_Init(&hwwdg);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : HS_LS_IN_Pin */
  GPIO_InitStruct.Pin = HS_LS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HS_LS_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S2_IN_Pin */
  GPIO_InitStruct.Pin = S2_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(S2_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nCAN2_STB_Pin CAN_RS_Pin */
  GPIO_InitStruct.Pin = nCAN2_STB_Pin|CAN_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN2_ERR_IN_Pin */
  GPIO_InitStruct.Pin = CAN2_ERR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAN2_ERR_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BLUE_Pin LED_RED_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_RED_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nCAN2_STB_Pin|CAN_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin|LED_RED_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
    // valid CAN-Message received
    
    // todo managed received data
    
    
    // when release fifo0 ISR Bit
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);  // achtung kein check auf welchen Buffer, hier fest 0
    
}

/**
  * @brief  Own-Systik Handler 
  *         
  * This function is called every 1ms. This function is used to handle the won pTicks structurel.<br>
  * maximal 255 tick variables are allowed 
  *
  */
void HAL_SYSTICK_Callback(void){  // handle own additional systicks
    uint32_t i;
    volatile uint32_t *pTick = &TimeoutTicks.canTransmitDelay ;    // this have to be the first variable in the timeticks

    // decrement all tick varriable 
    for (i=0; i < ( sizeof(timeoutticks_t) / sizeof(uint32_t));i++) {
        if (*pTick) {
            (*pTick)--;
        }
        pTick++;
    }
    
    
}

/**
  * @brief  CAN Baudrate setzen
  *         Baudrate für die CAN-Interfaces auf einen festen Wert setzen, <br>so das keine Autobauderkennung erforderlich ist.
  * @param  None
  * @retval None
  */
void SetCanBaudrate(enBitrate enBaud) {  //
      enCanBaudrate=enBaud; 
}


/**
  * @brief  User CAN initialization 
  *         Setup the CAN port with the Lawicel baudrate
  * @param  CAN_HandleTypeDef *hCANx
  * @todo   impelentation and testing
  * @note   the basic initializing is done by the cubeMX, here only baudratesettings is don
  * @retval None
  */

static void USER_CANx_Init(CAN_HandleTypeDef *hCANx,enBitrate baud, uint32_t canMode){

  
  
  //   
  
  hCANx->Init.Prescaler = stBitrate[baud].Prescaler;
  hCANx->Init.Mode = canMode;
  hCANx->Init.SJW =  CAN_SJW_1TQ; // always zero , which means the Synchronization Jump Width is 1 tq
  hCANx->Init.BS1 = stBitrate[baud].BS1;
  hCANx->Init.BS2 = stBitrate[baud].BS2;

  // since here should be set ,   
/*
  hCANx->Init.TTCM = DISABLE;
  hCANx->Init.ABOM = ENABLE;
  hCANx->Init.AWUM = DISABLE;
  hCANx->Init.NART = DISABLE;
  hCANx->Init.RFLM = DISABLE;
  hCANx->Init.TXFP = DISABLE;
*/
    
  HAL_CAN_Init(hCANx);





    
    
    
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
