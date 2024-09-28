#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define LED_DELAY_MS 250

void blink();
void pwm();
// Initialize the GPIO for the LED


void pico_led_init(void) {
#ifdef PICO_DEFAULT_LED_PIN
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
#endif
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(25, led_on);
#endif
}

void blink(void)
{
    pico_led_init();
    while (true) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        printf("blink is ready\n");
        return;
    };
    return;
}

void pwm(void)
{
    gpio_init(17); //gpio 17 is the D Auido Amp notShutDown Pin
    gpio_set_dir(17, GPIO_OUT);
    gpio_init(15);  //gpio 15 is the pwm1 25khz or 39khz signal for the ultrasonic generator
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(17, true);
    while(true)
    {
         gpio_put(15, true);
         sleep_us(40);
         gpio_put(15, false);
         sleep_us(40);
         //return;
    }
    return;
}

/*void user_inp(char x)
{
    //char x = 0;
    scanf("%s", &x);
    printf("entered value = %s\n", x);
    sleep_ms(1000);
    return;
}
void user_inp(char x);
*/
int main() {
    stdio_init_all();
    
    char state = 'i'; //statemachine variable
    uint8_t inp = 0;
    //blink();
    while(1){
        blink();
        inp = getchar(); //scanf("%s", &inp);
        //user_inp(inp);

        if (inp == 's')
        {
            state = 's';
        }
        else
        {
            state = 'i';
        }
        //state machine starts here
        switch(state)
        {
            case 'i':
                //idle so do something               
                    if (inp == 's')
                    {
                        state = 's';
                    }
                
                printf("in idle mode\n");
                /*while(true)
                {
                    printf("still in idle mode\n");
                    sleep_ms(1000);
                    return;
                }*/
                break;
            case 's':
                //start outputing pwm
                printf("starting mic\n");
                pwm();
                if (inp == 'i')
                    {
                        gpio_put(17, false);
                        state = 'i';
                    }  
                break;
        };
        //return 0;
     };
     //return 0;
}
