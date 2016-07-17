/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN__H__
#define __MAIN__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "stm32f1xx.h"   

// ######## user settings #############################################
     
//! Größe des CAN Empfangsbuffers
#define MYRX_SIZE (100)

//######### end user settings ##################################

// #define PING_PONG   // send back the received line , Attention: CAN-Hacker don't like this :-)
     
#define STM32_CAN_SW_VERSION "V0001\xD"         // attention CAN-Hacker don't like : V0000
#define STM32_CAN_HW_VERSION "AT72HW30SW2\xD"   // This version is required for recognizing the device by my CAN-Analyzer 


#define STM32_L_RESPONSE "L\xD"
     
     
#define STATUS_ERROR_CHANGED_DELAY_TIME (1000)  // 1s Verzögerung für das wiederholte senden des Fehlerstatus bei einer Änderung
#define END_VALUE_TIMESTAMP_COUNTER (0xEA5F)    // Timestamp Zähler zählt bis zu diesen Wert 59999ms 
     
     
     
#define MAX_USB_RECEIVED_BUFFER (10)
#define MAX_USB_RECEIVED_BUFFER_LEN (30)     
     
     
     
/** @addtogroup Exported_types
  * @{
  */  

/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */



/*!< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT
#define HSE_Value            HSE_VALUE
#define HSI_Value            HSI_VALUE
/**
  * @}
  */

   
typedef struct myRxMessage_st {
    CanRxMsgTypeDef LastRxMessage;     // Empfangene Message  
    CAN_TypeDef  *pCAN;         // Zeiger auf die Empfangsschnittstelle
    u16 timeStamp;              // Timestamp  
}myRxMessage_t;




 
 
//! Typedef für die allgemeine Interface-Statusflags
typedef struct {                                        // diverse status flags for Lawicel protocoll
    u8 CanChannelOnOff:                   1;              
    u8 BaudSettingsReceived:              1;              
    u8 TimeStampOnOff:                    1;              
    u8 CanSilentModeOnOff:                1;              
} interface_flags_t;
 
typedef enum bitrate_ty {
    _500_kbit       
,   _250_kbit
,   _125_kbit
,   _100_kbit
,    _95_kbit
,    _83_kbit
,    _50_kbit
,    _33_kbit
,    _20_kbit
,    _10_kbit
,    _47_kbit    

,	_LAST_BAUDRATE_
} enBitrate;

#define COUNT_BITRATE (_LAST_BAUDRATE_)


typedef struct Bitrate_t {      // 
    unsigned int Prescaler;
    uint32_t BS1;
    uint32_t BS2;
} Bitrate_st ;

//! Type for holding interrupt status 
typedef uint32_t InterruptStatus_t;


extern volatile unsigned char  myLastLine[MAX_USB_RECEIVED_BUFFER][MAX_USB_RECEIVED_BUFFER_LEN+1];    // Ablage Commandas Strings die per USB empfangen werden
extern volatile unsigned char  myWriteIndexLastLine ;   //SchreibIndesx für den USB STring, wenn beide indexe gleich sind, dann liegt keine neue Nachricht im Buffer
extern volatile unsigned char  myReadIndexLastLine;     // Lese Index für den USB
extern volatile unsigned char  myLastLinePosIdx;        // Spaltenindex von USB STring


extern const enBitrate usbBaudSettings[_LAST_BAUDRATE_];
extern struct myRxMessage_st stLastRxMessage[];


void SetCanBaudrate(enBitrate enBaud);     


// Globally enables interrupts if they were enabled 
   
__STATIC_INLINE void Interrupt_restore(InterruptStatus_t status){
    __set_PRIMASK(status & 0x01);
}

// Globally disables interrupts if they are not already disabled 
__STATIC_INLINE InterruptStatus_t Interrupt_saveAndDisable(void){
	InterruptStatus_t status;

    status = __get_PRIMASK();
    __set_PRIMASK(status | 0x01);

    return status;
}



#ifdef __cplusplus
}
#endif

#endif //__MAIN__H__
