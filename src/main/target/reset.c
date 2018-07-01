#include "platform.h"
#include "drivers/system.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/
void Reset_Handler(void)
{
#ifdef STM32F40_41xxx
    // Enable CCM
    RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
#endif

    checkForBootLoaderRequest();

    // Initialize stack
    extern uint8_t _heap_stack_begin, _heap_stack_end;
    memset(&_heap_stack_begin, 0xa5, &_heap_stack_end - &_heap_stack_begin);

    // Zero out trivially initialized SRAM variables
    extern uint8_t _sbss, _ebss;
    memset(&_sbss, 0, &_ebss - &_sbss);

    // Copy static duration variable initializers
    extern uint8_t _sidata, _sdata, _edata;
    memcpy(&_sdata, &_sidata, &_edata - &_sdata);

#ifdef USE_FAST_RAM
    // Zero out trivially initialized CCM/DTCM variables
    extern uint8_t __fastram_bss_start__, __fastram_bss_end__;
    memset(&__fastram_bss_start__, 0, &__fastram_bss_end__ - &__fastram_bss_start__);

    // Load FAST_RAM variable intializers into DTCM RAM
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, &_efastram_data - &_sfastram_data);
#endif

#ifdef USE_ITCM_RAM
    // Load functions into ITCM RAM
    extern uint8_t tcm_code_start;
    extern uint8_t tcm_code_end;
    extern uint8_t tcm_code;
    memcpy(&tcm_code_start, &tcm_code, &tcm_code_end - &tcm_code_start);
#endif

    SystemInit();

    extern int main(void);
    main();

    while (true) {}
}
