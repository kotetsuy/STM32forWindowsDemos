/**
  ******************************************************************************
  * @file    STM32L1xx_IAP/inc/flash_if.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    24-January-2012
  * @brief   This file provides all the headers of the flash_if functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define FLASH_PAGE_SIZE               0x20000     /* 128k Bytes */

/* define the address from where user application will be loaded,
   the application address should be a start sector address */
#define APPLICATION_ADDRESS   (uint32_t)0x08020000

/* define the address from where IAP will be loaded, 0x08000000:BANK1 or 
   0x08030000:BANK2 */
#define FLASH_START_ADDRESS   (uint32_t)0x08000000

/* Get the number of sectors from where the user program will be loaded */
#define FLASH_SECTOR_NUMBER  5

#define USER_FLASH_LAST_PAGE_ADDRESS  0x08060000
#define USER_FLASH_END_ADDRESS 0x0807ffff

/* define the user application size */
#define USER_FLASH_SIZE   (USER_FLASH_END_ADDRESS - APPLICATION_ADDRESS + 1)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartSector);
uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data, uint16_t DataLength);
//uint32_t FLASH_If_DisableWriteProtection(void);
//uint32_t FLASH_If_GetWriteProtectionStatus(void);
//FLASH_Status FLASH_If_WriteProtectionConfig(void);

#endif  /* __FLASH_IF_H */

/*******************(C)COPYRIGHT 2012 STMicroelectronics *****END OF FILE******/
