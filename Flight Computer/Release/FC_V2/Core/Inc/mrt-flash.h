#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_flash_ex.h"
#include "stm32f3xx_hal_flash.h"

HAL_StatusTypeDef MRT_EraseFlashPage(uint32_t address);
HAL_StatusTypeDef MRT_WriteDataToFlash(uint32_t TypeProgram, uint32_t address,
		uint64_t *data, uint32_t nbBytesToWrite);
