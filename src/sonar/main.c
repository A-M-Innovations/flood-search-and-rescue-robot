#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gptimer.h"
#include "esp_log.h"   // For logging (like printf, but better)
#include <esp_timer.h> // For microsecond delays
#include <freertos/queue.h>


//Macro for the GPIO pin to set the trig input pin on the sonar module
#define TRIG 32
//Macro for the GPIO pin to read the echo output pin on the sonar module
#define ECHO 33
#define TIMER_DIVIDER 80
static QueueHandle_t distance_queue; // Queue to send distance from ISR to task
int distance = 0;
static volatile bool echo_active = false;
static const char *TAG = "Distance";
static gptimer_handle_t gptimer = NULL;


//resets the TRIG GPIO pin and sets its as an output
void gpio_trig_config(){    
    gpio_reset_pin(TRIG);
    gpio_set_direction(TRIG, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG, 0);
}

//Initializes the echo gpio pin configuration parameters and resets the pin
void gpio_echo_config(){

    gpio_reset_pin(ECHO);
    // gpio_set_direction(TRIG, GPIO_MODE_OUTPUT);
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << ECHO),     // Select the pin using a bit mask
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&config);  // Apply this configuration
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);  // Initialize the ISR service (0 = default flags)
    gpio_isr_handler_add(ECHO, echo_isr_handler, NULL);  // Attach the ISR to the pin

}

void init_timer() {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Default clock (usually APB clock, ~80 MHz)
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, //1MHz = 1 tick per microsecond
    };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
}



void init_alarm(){
gptimer_alarm_config_t alarm_config = {
        .alarm_count = 38000, // 38 ms = 38000 ticks at 1 MHz
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
}

void IRAM_ATTR echo_isr_handler(void *arg)
{
    if (gpio_get_level(ECHO)){
        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0x00000000ULL));
        ESP_ERROR_CHECK(gptimer_start(gptimer));        
        echo_active = true;
    }
    else if (echo_active)
    {
        echo_active = false;
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
        uint64_t timer_value = 0;
        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &timer_value));
        distance =  (0.034 * timer_value) / 2;
        xQueueSendFromISR(distance_queue, &distance, NULL);
    }
}

static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    bool need_yield = false;
    if(echo_active){
        // Clears the interrupt flag so the ISR can run again next time.
        ESP_ERROR_CHECK(gptimer_stop(timer));
        ESP_ERROR_CHECK(gptimer_set_raw_count(timer, 0x00000000ULL));
        distance =  -1;
        xQueueSendFromISR(distance_queue, &distance, NULL);
        echo_active = false;
        need_yield  =true;    
    }
    return need_yield;
}


//Sets the trig pin high or low for 10us
void trig()
{
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10); //CPU actively waits so no latency to set trig pin low
    gpio_set_level(TRIG, 0);
}



void distance_task(void *arg)
{
    while(1)
    {
        if (xQueueReceive(distance_queue,&distance, portMAX_DELAY))
        {
            if (distance == -1){
                ESP_LOGI(TAG,"Echo wasn't received");
                
            }
            else
            {
                ESP_LOGI(TAG,"distance: %d cm", distance);
            }
            
        }
    }
}

void app_main() {
    gpio_trig_config();
    gpio_echo_config();
    init_timer();
    init_alarm();

    distance_queue = xQueueCreate(10, sizeof(int));
    xTaskCreate(distance_task, "distance_task", 2048, NULL, 5, NULL);
    while(1)
    {        
        trig();
        vTaskDelay(pdMS_TO_TICKS(100));    
    }


}


