//imports
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <stdio.h>

//Defining Pins
//LEDS
#define GREEN_LED GPIO_NUM_8
#define RED_LED GPIO_NUM_9
#define LOW_BEAMS GPIO_NUM_16
//BUTTONS
#define DRIVER_OCC GPIO_NUM_4
#define DRIVER_BELT GPIO_NUM_7
#define PASS_OCC GPIO_NUM_17
#define PASS_BELT GPIO_NUM_3
#define IGNITION GPIO_NUM_10
//BUZZER
#define BUZZER GPIO_NUM_14
//LIGHT SENSOR
#define POT_ADC_CHAN_2 ADC_CHANNEL_5
//POTENTIOMETER
#define POT_ADC_CHAN_1 ADC_CHANNEL_0

//ADC
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define BITWIDTH                ADC_BITWIDTH_12
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (19)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits

//Set the PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 

//min, max, and true full max angles for wipers
#define WIPERS_MIN              (245) //0 degrees
#define WIPERS_MAX              (620) //90 degrees
#define WIPERS_FULL_MAX         (965) //180 degrees

//Setting windshield wiper select thresholds
//for mode select
#define OFF_THRESH              3170
#define INT_THRESH              2377
#define LOW_THRESH              1585
#define HIGH_THRESH             793
//for speed select
#define SHORT_THRESH            0
#define MED_THRESH              1057
#define LONG_THRESH             2113
#define MAX_THRESH              3170

//DELAY
#define LOOP_DELAY_MS           25 // Loop delay
#define BUZZER_TIME             500 //time buzzer is on

//wiper variables
volatile bool wipe_off;
volatile bool wipe_int;
volatile bool wipe_low;
volatile bool wipe_high;
volatile bool short_del;
volatile bool med_del;
volatile bool long_del;

int short_del_ms = 1000;
int med_del_ms = 3000;
int long_del_ms = 5000;

//defining steps for low and high speed for wipers
int low_steps = 65;
int high_steps = 30;

//defining variables for wiper task
int stack_depth = 4096;
int wiper_task_priority = 5;

//function declarations
void perform_wipe_cycle(int steps);
void wiper_control_task(void *pvParameters);
static void ledc_init(void);
static void buttons_init(void);
static void leds_init(void);
static void buzzer_init(void);


void app_main(void) {

    //initializing components
    ledc_init();
    leds_init();
    buttons_init();
    buzzer_init();

    //ADC CONFIG
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle
   
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    }; 
    
    adc_oneshot_config_channel                          // Configure the Potentiometer Channel
    (adc1_handle, POT_ADC_CHAN_1, &config);
    adc_oneshot_config_channel                          // Configure the Light Sensor Channel
    (adc1_handle, POT_ADC_CHAN_2, &config);

    adc_cali_curve_fitting_config_t pot_cali_config_1 = { //configuring curve fitting for the potentiometer
        .unit_id = ADC_UNIT_1,
        .chan = POT_ADC_CHAN_1,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };

    adc_cali_curve_fitting_config_t pot_cali_config_2 = { //configuring curve fitting for the light sensor
        .unit_id = ADC_UNIT_1,
        .chan = POT_ADC_CHAN_2,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_cali_handle_t adc1_cali_chan_handle;            // Calibration handle
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&pot_cali_config_2, &adc1_cali_chan_handle);
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&pot_cali_config_1, &adc1_cali_chan_handle);

    //LCD Config
    hd44780_t lcd =
    {
        .write_cb = NULL,
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = GPIO_NUM_39,
            .e  = GPIO_NUM_37,
            .d4 = GPIO_NUM_36,
            .d5 = GPIO_NUM_35,
            .d6 = GPIO_NUM_48,
            .d7 = GPIO_NUM_47,
            .bl = HD44780_NOT_USED
        }
    };
    ESP_ERROR_CHECK(hd44780_init(&lcd));

    //creates wiper task
    xTaskCreate(wiper_control_task, "Wiper_Task", stack_depth, NULL, wiper_task_priority, NULL);

    //defining variables
    bool driver_occupied_prev = false;
    bool driver_occupied = false;
    bool driver_belt = false;
    bool pass_occupied = false;
    bool pass_belt = false;
    bool ignition_button_prev = false;
    bool ignition_button = false;
    bool ignition = false;
    bool can_start = false;
    
    
    //==========   MAIN LOOP   ==========

    while(true) {
        int pot_2_bits;                                   // light sensor ADC reading (bits)
        adc_oneshot_read
        (adc1_handle, POT_ADC_CHAN_2, &pot_2_bits);          // Read ADC bits
       
        int speed_select;                                     // ADC reading (mV)
        adc_cali_raw_to_voltage
        (adc1_cali_chan_handle, pot_2_bits, &speed_select);

        int pot_1_bits;                                   // ADC reading (bits)
        adc_oneshot_read
        (adc1_handle, POT_ADC_CHAN_1, &pot_1_bits);          // Read ADC bits
       
        int mode_select;                                     // ADC reading (mV)
        adc_cali_raw_to_voltage
        (adc1_cali_chan_handle, pot_1_bits, &mode_select);

        //update button states
        driver_occupied_prev = driver_occupied;
        driver_occupied = gpio_get_level(DRIVER_OCC) == 0;
        driver_belt = gpio_get_level(DRIVER_BELT) == 0;
        pass_occupied = gpio_get_level(PASS_OCC) == 0;
        pass_belt = gpio_get_level(PASS_BELT) == 0;
        ignition_button_prev = ignition_button;
        ignition_button = gpio_get_level(IGNITION) == 0;

        //update wiper mode
        wipe_off = mode_select <= OFF_THRESH && mode_select > INT_THRESH;
        wipe_int = mode_select <= INT_THRESH && mode_select > LOW_THRESH && ignition;
        wipe_low = mode_select <= LOW_THRESH && mode_select > HIGH_THRESH && ignition;
        wipe_high = mode_select <= HIGH_THRESH && mode_select >= 0 && ignition;
        //update wiper delay
        short_del = speed_select >= SHORT_THRESH && speed_select < MED_THRESH;
        med_del = speed_select >= MED_THRESH && speed_select < LONG_THRESH;
        long_del = speed_select >= LONG_THRESH && speed_select <= MAX_THRESH;

        //upate can_start variable based on current button states
        can_start = driver_occupied && driver_belt && pass_occupied && pass_belt;

        //welcome message
        if (driver_occupied && !driver_occupied_prev) { //if driver seat is occupied
            printf("Welcome to enhanced alarm system model 218-W25\n"); //print welcome message
        }

        //green LED
        if (can_start && !ignition) { //if conditions to start the engine are met
            gpio_set_level(GREEN_LED, 1); //turn green LED on
        }
        else {      //if conditions to start the engine are not met
            gpio_set_level(GREEN_LED, 0); //turn green LED off
        }

        if (ignition_button && !ignition_button_prev) {
            if(can_start && !ignition) { //if engine is not on and ignition conditions are met
                gpio_set_level(GREEN_LED, 0); //switch LEDS and print ignition message
                gpio_set_level(RED_LED, 1);
                printf("Engine Started\n");
                ignition = true;
            }
            else if (ignition) { //if engine is on, turn engine off
                ignition = false;
                gpio_set_level(RED_LED, 0);
                hd44780_clear(&lcd);

            }
            else {      //if the conditions for the engine starting are not met
                gpio_set_level(BUZZER, 1);      //activate the buzzer
                printf("Ignition Inhibited\n"); //print ignition failure message
                if (!driver_occupied) { 
                printf("Driver seat not occupied\n");
                }
                if (!driver_belt) {
                    printf("Driver seatbelt not fastened\n");
                }
                if (!pass_occupied) {
                    printf("Passenger seat not occupied\n");
                }
                if (!pass_belt) {
                    printf("Passenger seatbelt not fastened\n");
                }
                vTaskDelay(BUZZER_TIME / portTICK_PERIOD_MS);
                gpio_set_level(BUZZER, 0); 
            }
        }
       
        if(ignition) {
            hd44780_clear(&lcd);
            //displaying mode
            hd44780_gotoxy(&lcd, 0, 0);
            if(wipe_off) {
                hd44780_puts(&lcd, "Off");
            }
            else if(wipe_int) {
                hd44780_puts(&lcd, "Intermittent");
                hd44780_gotoxy(&lcd, 0, 1);
                if(short_del){
                    hd44780_puts(&lcd, "Short");
                }
                else if(med_del) {
                    hd44780_puts(&lcd, "Medium");
                }
                else if(long_del) {
                    hd44780_puts(&lcd, "Long");
                }
            }
            else if (wipe_low) {
                hd44780_puts(&lcd, "Low");
            }
            else if (wipe_high) {
                hd44780_puts(&lcd, "High");
            }
        }
        
        vTaskDelay(  LOOP_DELAY_MS / portTICK_PERIOD_MS); //loop delay to prevent bouncy inputs
    }
}
//wiper task handles the wipers, sets them to the proper mode
void wiper_control_task(void *pvParameters) { 
    while(1) {
        if(wipe_high) {
            perform_wipe_cycle(high_steps);
        }
        else if(wipe_low) {
            perform_wipe_cycle(low_steps);
        }
        else if(wipe_int) {
            perform_wipe_cycle(low_steps);

            int delay_ms = short_del_ms;
            if(short_del) delay_ms = short_del_ms;
            if(med_del) delay_ms = med_del_ms;
            if(long_del) delay_ms = long_del_ms;

            vTaskDelay(delay_ms/portTICK_PERIOD_MS);
        }
        else {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, WIPERS_MIN);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS); //MIGHT CAUSE PROBLEMS, CHECK WITH BOARD
        }
    }
}
//init functions to configure pins

static void ledc_init(void) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void buttons_init(void) {
    //Configuring DRIVER_OCC
    gpio_config_t driver_occ_io_conf = {
        .pin_bit_mask = (1ULL << DRIVER_OCC), 
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&driver_occ_io_conf));

    //Configuring DRIVER_BELT
    gpio_config_t driver_belt_io_conf = {
        .pin_bit_mask = (1ULL << DRIVER_BELT), 
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&driver_belt_io_conf));

    //Configuring PASS_OCC
    gpio_config_t pass_occ_io_conf = {
        .pin_bit_mask = (1ULL << PASS_OCC), 
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&pass_occ_io_conf));

    //Configuring PASS_BELT
    gpio_config_t pass_belt_io_conf = {
        .pin_bit_mask = (1ULL << PASS_BELT), 
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&pass_belt_io_conf));

    //Configuring IGNITION
    gpio_config_t ignition_io_conf = {
        .pin_bit_mask = (1ULL << IGNITION), 
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&ignition_io_conf));
}

static void leds_init(void) {
    //Configuring GREEN_LED
    gpio_config_t green_led_io_conf = {
        .pin_bit_mask = (1ULL << GREEN_LED), 
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&green_led_io_conf));
    
    //Configuring RED_LED
    gpio_config_t red_led_io_conf = {
        .pin_bit_mask = (1ULL << RED_LED), 
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&red_led_io_conf));
    
    //Configuring LOW_BEAMS
    gpio_config_t low_beams_led_io_conf = {
        .pin_bit_mask = (1ULL << LOW_BEAMS), 
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&low_beams_led_io_conf));
}

static void buzzer_init(void) {
    gpio_config_t buzzer_io_conf = {
        .pin_bit_mask = (1ULL << BUZZER), 
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE    
    };
    ESP_ERROR_CHECK(gpio_config(&buzzer_io_conf));
}

//performs a wipe with the given number of steps
void perform_wipe_cycle(int steps) {
    
    int angle = WIPERS_MIN;
    int interval = (WIPERS_MAX - WIPERS_MIN) / steps;
    while(angle < WIPERS_MAX) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        angle += interval;
        vTaskDelay(LOOP_DELAY_MS/portTICK_PERIOD_MS);
    }
    angle -= interval;
    while(angle > WIPERS_MIN) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        angle -= interval;
        vTaskDelay(LOOP_DELAY_MS/portTICK_PERIOD_MS);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, WIPERS_MIN);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}