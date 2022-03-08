/**
 * @file tasks.c
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief general RTOS system tasks
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO15:  input, pulled up, interrupt from falling edge
 * GPIO25:  input, pulled up, interrupt from falling edge.
 */

// define listening pin as input, interrupt flag
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     25
#define GPIO_INPUT_IO_1     15
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

// The GPIO queue, gets filled when event is detected
static QueueHandle_t gpio_evt_queue = NULL;

// Raise Interrupt, send to queue
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// do some task after interrupt was raised to queue
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // TODO: which ADS GPIO raised it ? buffer 1 or 2 ? ->readData()
            
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void interrupt_setup(){

   //zero-initialize the config structure.
    gpio_config_t io_conf = {}; 
    //interrupt Trigger when DRDY goes LOW
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO15/25 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task: method, name, stack size, parameters, priority, handle
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 2, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}
