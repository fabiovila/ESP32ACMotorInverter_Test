#include <stdio.h>
#include <math.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"

extern int  Bluetooth_Init(void);
extern void Motor_Init(void);


void app_main()
{
    
    Motor_Init();
    Bluetooth_Init();


}



