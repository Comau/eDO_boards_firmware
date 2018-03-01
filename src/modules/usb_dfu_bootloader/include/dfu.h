#pragma once

#define USE_BACKUP_REGISTER false

#define DFU_MAGIC 0xB0BAFE77
extern const uint32_t __ram0_end__[];
extern const uint32_t user_address_bottom[];
static const uint32_t PROGRAM_FLASH_FROM = (uint32_t)(user_address_bottom);

uint32_t getMagic() {
#if USE_BACKUP_REGISTER
	return RTC->BKP0R;
#else
	return *((uint32_t*)(__ram0_end__));
#endif
}

void setMagic(uint32_t magic) {
#if USE_BACKUP_REGISTER
#if 0
	  /* Backup domain access enabled and left open.*/
	  PWR->CR |= PWR_CR_DBP;

	  /* Reset BKP domain if different clock source selected.*/
	  if ((RCC->BDCR & STM32_RTCSEL_MASK) != STM32_RTCSEL){
	    /* Backup domain reset.*/
	    RCC->BDCR = RCC_BDCR_BDRST;
	    RCC->BDCR = 0;
	  }

	//PWR->CR |= PWR_CR_DBP;
    //RCC->BDCR |= RCC_BDCR_RTCEN;
    __NOP();
#endif
	RTC->BKP0R = magic;
#else
	*((uint32_t*)(__ram0_end__)) = magic;
#endif
}

typedef void (* pFunction)(
    void
);

int32_t
jumptoapp(
    uint32_t addr
)
{
    pFunction JumpToApp;
    uint32_t  JumpAddress;

    // The second entry of the vector table contains the reset_handler function
    JumpAddress = *(uint32_t*)(addr + 4);

    // Assign the function pointer
    JumpToApp = (pFunction)JumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(uint32_t*)addr);

    chSysDisable();

    // Jump!
    JumpToApp();

    return 0;
} // jumptoapp
