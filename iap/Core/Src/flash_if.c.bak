/**
  ******************************************************************************
  * @file    STM32L1xx_IAP/src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    24-January-2012
  * @brief   This file provides all the memory related operation functions.
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

/** @addtogroup STM32L1xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

#if 0
/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 
  /* Unlock the Program memory */
  FLASH_Unlock();

  /* Clear all FLASH flags */  
  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);   
}
#endif

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint32_t FLASH_If_Erase(uint32_t StartSector)
{
  FLASH_EraseInitTypeDef EraseInit = {
		  FLASH_TYPEERASE_SECTORS,
		  FLASH_BANK_1,
		  FLASH_SECTOR_5,
		  3, // 5,6,7
		  FLASH_VOLTAGE_RANGE_3
  };
  uint32_t SectorError;
  
  if (HAL_FLASHEx_Erase(&EraseInit, &SectorError) == HAL_OK) {
	  return 0;
  }
  return 1;
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data ,uint16_t DataLength)
{
	uint32_t addr = (uint32_t)FlashAddress;

	while (DataLength > 0) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *Data) != HAL_OK) {
			return 1;
		}
		addr += 4;
		Data++;
		DataLength--;
	}
	return 0;
}

#if 0
/**
  * @brief  Disables the write protection of user flash area.
  * @param  None
  * @retval 0: Write Protection successfully disabled
  *         1: Error: Flash write unprotection failed
  *         2: Flash memory is not write protected
  */
uint32_t FLASH_If_DisableWriteProtection(void)
{
  FLASH_Status status = FLASH_BUSY;

  /* Clear all FLASH flags */  
  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
  
  /* Test if user memory is write protected */
  if (FLASH_If_GetWriteProtectionStatus() != 0x00)
  {
    /* Unlock the Option Bytes */  
    FLASH_OB_Unlock();

    /* Disable the write protection of user application pages */ 
    status = FLASH_If_WriteProtectionConfig();
    if (status == FLASH_COMPLETE)
    {
      /* Write Protection successfully disabled */
      return (0);
    }
    else
    {
      /* Error: Flash write unprotection failed */
      return (1);
    }
  }
  else
  {
     /* Flash memory is not write protected */
     return(2);
  }
}

/**
  * @brief  Returns the write protection status of user flash area.
  * @param  None
  * @retval If the sector is write-protected, the corresponding bit in returned
  *         value is set.
  *         If the sector isn't write-protected, the corresponding bit in returned
  *         value is reset.
  *         e.g. if only sector 3 is write-protected, returned value is 0x00000008
  */
uint32_t FLASH_If_GetWriteProtectionStatus(void)
{
#ifdef STM32L1XX_MD 
  return(FLASH_OB_GetWRP() & FLASH_PROTECTED_SECTORS);
#elif defined STM32L1XX_HD
  uint32_t WRP_Status = 0, WRP1_Status = 0, WRP2_Status = 0, sectornumber = 0;
  
  sectornumber=FLASH_SECTOR_NUMBER;
  /* Test on the start address in Bank1 */
  if (FLASH_START_ADDRESS <= 0x0802FF00)
  {
    /* Test on the user application to be programmed in Bank1 */
    if (APPLICATION_ADDRESS <= 0x0802FF00)
    {
      WRP2_Status = FLASH_OB_GetWRP2();
      
      if(sectornumber < 32)
      {
        WRP_Status = FLASH_OB_GetWRP() & ((uint32_t)~((1<<sectornumber) - 1));
        WRP1_Status = FLASH_OB_GetWRP1();
      }
      else
      {
        WRP1_Status = (FLASH_OB_GetWRP1() & ((uint32_t)~((1<<(sectornumber-32)) - 1)));
      }
    }
    else if (APPLICATION_ADDRESS >= 0x08030000) /* Test on the user application to be programmed in Bank2 */
    {
      sectornumber = (uint32_t)((APPLICATION_ADDRESS - 0x08030000)>>12);
      if (sectornumber < 16)
      {
        WRP1_Status = FLASH_OB_GetWRP1() & (uint32_t)~((1 << (sectornumber +16) - 1));
        WRP2_Status = FLASH_OB_GetWRP2();
      }
      else
      {
        WRP2_Status = FLASH_OB_GetWRP2() & (uint32_t)~((1 << (sectornumber-16) - 1));
      }
    }
  }
  else if (FLASH_START_ADDRESS >= 0x08030000) /* Test on the start address in Bank2 */
  {  
    if (APPLICATION_ADDRESS <= 0x0802FF00)  /* Test on the user application to be programmed in Bank1 */
    {
      sectornumber = (uint32_t)(APPLICATION_ADDRESS - 0x08000000)>>12;
      if(sectornumber < 32)
      {
        WRP_Status = FLASH_OB_GetWRP() & (uint32_t)~((1<<sectornumber - 1));
        WRP1_Status = FLASH_OB_GetWRP1()&0x0000FFFF;
      }
      else
      {
        WRP1_Status = (FLASH_OB_GetWRP1() & (uint32_t)~((1 << (sectornumber-32) - 1)))&0x0000FFFF;    
      }
    }
    else  /* Test on the user application to be programmed in Bank2 */
    {
      if(sectornumber < 16)
      {
        WRP1_Status = FLASH_OB_GetWRP1() & ((uint32_t)~(1 << (sectornumber + 16) - 1));
        WRP2_Status = FLASH_OB_GetWRP2();
      }
      else
      {
        WRP2_Status = (FLASH_OB_GetWRP2() & (uint32_t)~(1 <<( sectornumber - 16) - 1));
      }      
    }
  }
  if ((WRP_Status!=0)||(WRP1_Status!=0)||(WRP2_Status!=0))
    return 1;
  else
    return 0;

#endif   
}

/**
  * @brief  Disable the write protection status of user flash area.
  * @param  None
  * @retval If the sector is write-protected, the corresponding bit in returned
  *         value is set.
  *         If the sector isn't write-protected, the corresponding bit in returned
  *         value is reset.
  *         e.g. if only sector 3 is write-protected, returned value is 0x00000008
  */
FLASH_Status FLASH_If_WriteProtectionConfig(void)
{
  FLASH_Status state = FLASH_COMPLETE;
#ifdef STM32L1XX_MD 
  
  /* Disable the write protection of user application pages */
  state = FLASH_OB_WRPConfig(FLASH_PROTECTED_SECTORS, DISABLE); 
#elif defined STM32L1XX_HD
  FLASH_Status WRPstatus = FLASH_COMPLETE, WRP1status = FLASH_COMPLETE, WRP2status = FLASH_COMPLETE ;
  uint32_t sectornumber = 0;
  
  sectornumber=FLASH_SECTOR_NUMBER;
  
  /* Test on the start address on Bank1 */
  if (FLASH_START_ADDRESS <= 0x0802FF00)
  {
    if (sectornumber < 32)
    {
      WRPstatus = FLASH_OB_WRPConfig(((uint32_t)~((1 << sectornumber) - 1)), DISABLE);
      WRP1status = FLASH_OB_WRP1Config(OB_WRP1_AllPages, DISABLE);
      WRP2status = FLASH_OB_WRP2Config(OB_WRP2_AllPages, DISABLE);
    }
    else if ((sectornumber >= 32)&& (sectornumber < 64))
    {
      WRP1status = FLASH_OB_WRP1Config(((uint32_t)~((1 << (sectornumber - 32)) - 1)), DISABLE);
      WRP2status = FLASH_OB_WRP2Config(OB_WRP2_AllPages, DISABLE);
    }
    else 
      WRP2status = FLASH_OB_WRP2Config(((uint32_t)~((1 << (sectornumber - 64)) - 1)), DISABLE);
  }
  else  if (FLASH_START_ADDRESS >= 0x08030000)/* Test on the start address on Bank2 */
  {
    if( APPLICATION_ADDRESS <=0x0802FF00)/* User application in BANK1 */
    {
      sectornumber = (uint32_t)((APPLICATION_ADDRESS-0x08000000)>>12);
      if(sectornumber < 32)
      {
        WRPstatus = FLASH_OB_WRPConfig((uint32_t)~((1 << sectornumber) - 1), DISABLE);
        WRP1status = FLASH_OB_WRP1Config(0x0000FFFF,DISABLE);
      }
      else
      { 
        WRP1status = FLASH_OB_WRP1Config((uint32_t)~((1 << (sectornumber-32)) - 1), DISABLE);
      }
    }
    else /* User application Bank2 */
    {
      if(sectornumber < 16)
      {
        WRP1status = FLASH_OB_WRP1Config((uint32_t)~((1 << (sectornumber+16)) - 1), DISABLE); 
        WRP2status = FLASH_OB_WRP2Config(OB_WRP2_AllPages, DISABLE);
      }
      else
      {
        WRP2status = FLASH_OB_WRP2Config((uint32_t)~((1 << (sectornumber -16 )) - 1), DISABLE);
      }
    }

    if ((WRPstatus!=FLASH_COMPLETE)||(WRP1status!=FLASH_COMPLETE)||(WRP2status!=FLASH_COMPLETE))
      state = FLASH_ERROR_WRP;
  }
#endif   
  return state;
}
#endif
/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
