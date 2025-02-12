#include <pico/stdlib.h>
#include <stdio.h>
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <pico/time.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/structs/systick.h>
#include <hardware/exception.h>

#define FLAG_CORE1 1234

//time
volatile uint32_t period = 0;
volatile uint64_t top = 0;

uint32_t counter64bit = 0;
uint32_t moyenne = 0;

void core1();

//Interruption signal A (rising edge)
void InterruptSignalA(uint gpio, uint32_t events);

//Interrupt when counter24 bits reach 0
void SysTickINT();

uint64_t GetTime64();

int main()  {
  //Setup
  stdio_init_all();

  set_sys_clock_khz(125000, 0);   //clk_sys à 125MHz

  systick_hw->csr |= 0x00000007;  //Enable timer with interrupt
  systick_hw->rvr = 0x00ffffff;         //Set the max counter value (when the timer reach 0, it's set to this value)
  exception_set_exclusive_handler(SYSTICK_EXCEPTION, SysTickINT);	//Interrupt

  //test LED
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  
  //Interrupt on GPIO 21
  gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_EDGE_RISE, true, InterruptSignalA);

  clkFrequence = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000; //clk_cpu en hz;

  //Setup core1 :
  multicore_launch_core1(core1);
  multicore_fifo_pop_blocking(); //Attend le lancement du core1

  //LED Test :
  gpio_put(25, 1);


  //================== Core0 loop ====================
  while(1)  {
    
    //Communication with core1()
    if (multicore_fifo_wready())  {
        multicore_fifo_push_blocking(period);
    }
  }
}

void core1()  {
  //send a flag when core1 is ready
  multicore_fifo_push_blocking(FLAG_CORE1);

  //================== Core1 loop ====================
  while(1)  {

    //Give me the period value of the signal
    if (multicore_fifo_rvalid())  {
      printf("p: %ld\n", multicore_fifo_pop_blocking());
    }    
  }
}

void InterruptSignalA(uint gpio, uint32_t events) {
  period = GetTime64() - top;  //get the period
  top = GetTime64();
}

uint64_t GetTime64()  {
  //conversion of 24bit timer to 64bit (because the timer overflow in 130ms)
  return  (counter64<<24) + (0x00ffffff - systick_hw->cvr);
}

void SysTickINT()   {
  //When the counter overflow, we increment this variable
  counter64++;
}