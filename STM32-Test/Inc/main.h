/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN__H__
#define __MAIN__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


#define PING_PONG   // send back the received line , Attention: CAN-Hacker don't like this :-)
     
#define STM32_CAN_SW_VERSION "V0000\xD"
#define STM32_CAN_HW_VERSION "AT72HW30SW2\xD"   // This version is required for recognizing the device by my CAN-Analyzer 


#define STM32_L_RESPONSE "L\xD"
     
     
     
     
     
#define MAX_USB_RECEIVED_BUFFER (10)
#define MAX_USB_RECEIVED_BUFFER_LEN (30)     
 


extern volatile unsigned char  myLastLine[MAX_USB_RECEIVED_BUFFER][MAX_USB_RECEIVED_BUFFER_LEN+1];    // Ablage Commandas Strings die per USB empfangen werden
extern volatile unsigned char  myWriteIndexLastLine ;   //SchreibIndesx für den USB STring, wenn beide indexe gleich sind, dann liegt keine neue Nachricht im Buffer
extern volatile unsigned char  myReadIndexLastLine;     // Lese Index für den USB
extern volatile unsigned char  myLastLinePosIdx;        // Spaltenindex von USB STring


     
     

#ifdef __cplusplus
}
#endif

#endif //__MAIN__H__
