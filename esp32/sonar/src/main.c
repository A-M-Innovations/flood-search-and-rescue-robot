  /*
  ******************************************************************************
  * This program will send sonar data back to the host via wifi. The distance 
  * will be in units of feet. You can also view the distance in the serial
  * monitor.
  ******************************************************************************
*/

#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gptimer.h"
#include "esp_log.h"   
#include <esp_timer.h> 
#include <freertos/queue.h>


#define TRIG 32
#define ECHO 33
#define TIMER_DIVIDER 80 //Used to set clock rate 1MHz in gptimer intialziation
static QueueHandle_t distance_queue; //Queue to send distance from ISR to distance task
int distance = 0; 
static volatile bool echo_active = false; //indicates whether sonar module transmitted an echo
static const char *TAG = "Distance";
static gptimer_handle_t gptimer = NULL;


//Handles the rising and falling edge of the echo pin
//Starts the timer if echo pin goes high and stops,
//calculates the distance based on elapsed time and
//stores in in a queue for the distance task to print
void IRAM_ATTR echo_isr_handler(void *arg)
{
    if (gpio_get_level(ECHO)){
        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0x00000000ULL));
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        echo_active = true; // to enable distance calculation when gpio interrupt occurs
    }
    else if (echo_active)
    {
        echo_active = false;
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
        uint64_t timer_value = 0;
        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &timer_value));
        distance =  (0.034 * timer_value) / 2; //uses the distance-speed-time equation to calcuate distance
        xQueueSendFromISR(distance_queue, &distance, NULL); //distance is stored in a queue to later be used in the distance task
    }
}

//Alarm callback that is used when the echo is not received back after 38ms. 
//Sends a distance value of -1 to distance task to indicate no echo and
//print no echo message.
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    bool need_yield = false;
    if(echo_active){
        ESP_ERROR_CHECK(gptimer_stop(timer));
        ESP_ERROR_CHECK(gptimer_set_raw_count(timer, 0x00000000ULL));
        distance =  -1; //stored in queue so that distance task can be unblocked
        xQueueSendFromISR(distance_queue, &distance, NULL); //distance is stored in a queue to later be used in the distance task
        echo_active = false;
        need_yield  =true; //since true, switch to another task immediately when return is executed 
    }
    return need_yield;
}

//resets the TRIG GPIO pin and sets it as an output
void gpio_trig_config(){    
    gpio_reset_pin(TRIG);
    gpio_set_direction(TRIG, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG, 0);
}

//Initializes the echo gpio pin configuration parameters resets the pin and enables an intr.
void gpio_echo_config(){
    gpio_reset_pin(ECHO);
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

//initalize the alarm and registers the timer_alarm_cb callback
void init_alarm(){
gptimer_alarm_config_t alarm_config = {
        .alarm_count = 38000, // 38 ms = 38000 ticks at 1 MHz. Users clock rate of timer.
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer)); //enable timer after registering the callback
}

//Sets the trig pin high or low for 10us
void trig()
{
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10); //CPU actively waits so no latency to set trig pin low
    gpio_set_level(TRIG, 0);
}


//Prints the distance to a serial monitor
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

    distance_queue = xQueueCreate(10, sizeof(int)); //creates a queue to store the distance to be used in the distance task
    xTaskCreate(distance_task, "distance_task", 2048, NULL, 5, NULL); //task is created to print the distance to serial monitor
    while(1)
    {        
        trig();
        vTaskDelay(pdMS_TO_TICKS(100)); //wait so IDLE task can be ran and WDT doesnt expire
    }
}


