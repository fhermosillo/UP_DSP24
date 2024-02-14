/* Private Includes -------------------------------------------*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* Private Defines --------------------------------------------*/
#define INPUT_PIN GPIO_NUM_15
#define LED_PIN   GPIO_NUM_2

/* Private Variables ------------------------------------------*/
int state = 0;
xQueueHandle xExtiQueue;


/* Interrupt Handlers -----------------------------------------*/
static void IRAM_ATTR EXTI15_IRQHandler(void *args)
{
  int gpio_num = (int)args;
  xQueueSendFromISR(xExtiQueue, &gpio_num, NULL);
}


/* Tasks ------------------------------------------------------*/
void vLEDControlTask(void *params)
{
  // Local variable definition
  uint32_t gpio_pin, count = 0;

  // A task should never finish
  while (true)
  {
    // Block task until a queue is received
    if (xQueueReceive(xExtiQueue, &gpio_pin, portMAX_DELAY))
    {
      gpio_set_level(LED_PIN, gpio_get_level(INPUT_PIN));
    }
  }
}


/* Arduino Framework ----------------------------------------------------------------*/
void setup(void)
{
  // Setup LED gpio
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  // Setup button gpio
  gpio_reset_pin(INPUT_PIN);
  gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
  gpio_pulldown_en(INPUT_PIN); // GPIO input is always on LOW level
  gpio_pullup_dis(INPUT_PIN);
  gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE); // Set EXTI type (LOW to HIGH)

  // Queue
  xExtiQueue = xQueueCreate(10, sizeof(uint32_t)); // 10 items of type "int"

  // Task creation
  xTaskCreate(vLEDControlTask,  // Task pointer
              "led",            // Task name (for debug only)
              2000,             // Stack size (for local variables, registers, etc.)
              NULL,             // Input argument (Null)
              1,                // Priority (Higher = 25, Lowest = 0 )
              NULL);            // Task handler (Null)
  
  // Install EXTI ISR service routine
  gpio_install_isr_service(0);
  // Register IRQHandler
  gpio_isr_handler_add( INPUT_PIN,          // GPIO Pin
                        EXTI15_IRQHandler,  // IRQHandler
                        (void *)INPUT_PIN); // Input argument
}

/* FreeRTOS Main Task ----------------------------------------------------------*/
// This function runs in a while of the main task of the RTOS
void loop(void) {
  vTaskDelay(1000);
}
