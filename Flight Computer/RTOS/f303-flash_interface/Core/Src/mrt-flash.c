#include "mrt-flash.h"

HAL_StatusTypeDef MRT_EraseFlashPage(uint32_t address) {
	if (HAL_FLASH_Unlock() != HAL_OK) {
		myprintf("Flash unlock failed, could not erase page\n");
		return HAL_ERROR;
	}
	uint32_t reg = 0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = address;
	EraseInitStruct.NbPages = 1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &reg);

	if ((reg | 0xFFFFFFFF) == 0xFFFFFFFF) {

	} else {
		myprintf("Page erase failed.\n");
		return HAL_ERROR;
	}
	HAL_FLASH_Lock();
	return HAL_OK;
}

/*
 * Does not append, use with precaution.
 */
HAL_StatusTypeDef MRT_WriteDataToFlash(uint32_t TypeProgram, uint32_t address,
		uint64_t *data, uint32_t nbBytesToWrite) {
	/*
	 if (*((uint32_t *) address) != -1) // Un-comment this 'if' whenever you use this code to avoid overwriting data UNTESTED CODE THO OOPS
	 {
	 myprintf("Flash unlock failed, could not erase page\n");
	 return HAL_ERROR;
	 }
	 */

	uint32_t currAddress = address;
	uint32_t currIncrement = 0;

	switch (TypeProgram) {
	case FLASH_TYPEPROGRAM_DOUBLEWORD:
		currIncrement += 4;
	case FLASH_TYPEPROGRAM_WORD:
		currIncrement += 2;
	case FLASH_TYPEPROGRAM_HALFWORD:
		currIncrement += 2;
		break;
	default:
		myprintf("Invalid TypeProgram\n");
		return HAL_ERROR;
		break;
	}

	if (MRT_EraseFlashPage(currAddress - ((currAddress - 0x08000000U) % 0x800U)) != HAL_OK) {
		return HAL_ERROR;
	}

	for (int i = 0; i < nbBytesToWrite; i++) {
		if (HAL_FLASH_Unlock() != HAL_OK) {
			myprintf("Flash unlock failed, could not write to memory\n");
			return HAL_ERROR;
		}

		if (HAL_FLASH_Program(TypeProgram, currAddress, data[i]) != HAL_OK) {
			myprintf("Could not write\n");
			return HAL_ERROR;
		}
		HAL_FLASH_Lock();

		currAddress += currIncrement;

		if ((currAddress - 0x08000000U) % 0x800U == 0) {

			if (MRT_EraseFlashPage(currAddress) != HAL_OK) {
				return HAL_ERROR;
			}
		}
	}

}
