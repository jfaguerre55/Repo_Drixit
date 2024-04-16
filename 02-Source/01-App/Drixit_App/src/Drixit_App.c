/*
===============================================================================
 Name        : Drixit_App.c
 Author      : Juan Aguerre
 Version     : 0
 Copyright   : -
 Description : main definition
===============================================================================
*/

#include "Drixit_App_common.h"

bool hw_sys_err = false;
bool os_sys_err = false;

int main(void) {

	// System clock post-boot update
    SystemCoreClockUpdate();

    // Hardware/Board initialization
    hw_sys_err = System_Hardware_Init();

    // OS initialization
    os_sys_err = System_OS_Init();

	// Check for correct system init
	assert( !hw_sys_err || !os_sys_err );
	while( hw_sys_err||os_sys_err );

    // Start FreeRTOS kernel
    vTaskStartScheduler();

    while(true);

    return 0 ;
}







