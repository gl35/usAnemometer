'''Blink the on board LED Pico W pinouts only
Patrick Palmer Jan 2024'''
from machine import Pin, Timer, PWM, UART, ADC, RTC
#from neotimer import *
from rp2 import PIO, StateMachine, asm_pio
import utime 
#from array import *
import time, gc, _thread
import select
import sys
import array

machine.freq(125000000)
# machine.freq(138000000)

# rtc = RTC()

timer = machine.Timer() # timer for the blinking led

led = machine.Pin("LED", machine.Pin.OUT) #GPIO LED
shutdownDamp = Pin (17, Pin.OUT) #SHUTDown Digital Amplifier GPIO 17

# pdm1_On = Pin (10, Pin.OUT) #GPIO 10 & 11 to power on and off the PDM mics
testOut = Pin (10, Pin.OUT) #GPIO 10 & 11 to power on and off the test output
pdm2_On = Pin (11, Pin.OUT)


#LED blinking logic

# def blink():
#     led.on()
#     utime.sleep_ms(10)
#     led.off()
#     utime.sleep_ms(1000)
#     return
# timer.init(freq=10, mode=Timer.PERIODIC, callback=blink)


#1st PWM - 39kHz to excit the ultrasonic generator
pwm1 = PWM ( Pin ( 15, Pin.OUT ) ) # GP15
pwm1.freq ( 1000 ) # 39kHz
pwm1.duty_u16 ( 32767 ) # duty 50% (65535/2)

#2nd PWM - 1.024Mhz clk for PDM mic
fclk = 1024000 #variable for PDM clock frequency
pwm2 = PWM ( Pin ( 1, Pin.OUT ) ) # GP1
pwm2.freq ( fclk ) # fclk = 1.024MHz, 3.072Mhz 
pwm2.duty_u16 (32767 ) # duty 50% (65535/2)


#ADC value for PDM mic 2
# pdm1 = ADC(Pin(26))     # create ADC object on ADC pin GPIO26 - variable name is pdm1 and not adc1 - uncomment if using pdm mic

GPIO_PIN = 20 # Do not connect anything to this pin

 
mic1_FF = Pin (27, Pin.IN, machine.Pin.PULL_UP)     # create PIO pin GPIO27 mic1 - uncomment if using analog mic
mic2_FF = Pin (26, Pin.IN, machine.Pin.PULL_UP)     # create PIO pin GPIO26 mic2 - uncomment if using analog mic 

        
#calculate time
# deltaT = 0
time1 = float()
time2 = float()
windspeed = float()


pin = True
pin1 = True
pin2 = True


def troubleshoot():
    global pin
    print("Pin 27 value is", mic1_FF.value())
#     print("Pin 27 value is", mic2_FF.value())
    print ("time_diff=", jkFF(pin))
    utime.sleep_ms(100)

'''phase difference algorithm
deltaT = time2 - time1;
if deltaT <= (deltaT_pre + T):  # T is the period of the diveded pulses(1/25khz)
    deltaT_prim = deltaT;
    deltaT_pre = deltaT_prim;  # use this to calculate the windspeed
    else:
        deltaT_prim = (deltaT - T);
        deltaT_pre = deltaT_prim;
'''

'''    
#try to make a pio input         
# pio timer example starts here ************************************************
@asm_pio()
def PulseIn():
  set(x, 0)           # X = 0
  wait(0, pin, 0)     # Do {} While ( pin == 1 );
  wait(1, pin, 0)     # Do {} While ( pin == 0 );
  label("loop")       # Do
  jmp(x_dec, "next")  #   X--
  label("next")
  jmp(pin, "loop")    # While ( pin == 1 );
  mov(isr, x)         # Push(X)
  push(block)

rx = StateMachine(0, PulseIn, in_base=mic1_FF, jmp_pin=mic1_FF)
rx.active(1)

tx_pin = Pin(GPIO_PIN, Pin.OUT)

@asm_pio(set_init=[PIO.OUT_LOW])
def PulseOut():
  pull(block)
  mov(x, osr)
  jmp(x_dec, "next")
  label("next")
  set(pins, 1)
  label("loop")
  jmp(x_dec, "loop")
  set(pins, 0)

tx = StateMachine(1, PulseOut, set_base=tx_pin)
tx.active(1)

# Send a pulse
tx.put(4)

# Read how long that pulse was
x = ( 1 << 32) - rx.get()

# Each count takes two PIO cycles
# At 125MHz each PIO cycle is 8ns

print("count = {}, {} cycles, {} ns".format(x, x * 2, x * 2 * 8))                                                    
'''
#pio timer example ends here ******************************
@asm_pio()
def jkFF1():
    wrap_target()
    wait(0, pin, 0)
    irq(block, rel(0))
    wait(1, pin, 0)
    wrap()
    
def handler1(sm):
    time1 =utime.ticks_us() # could also use utime.ticks_us()
    # Print a (wrapping) timestamp, and the state machine object.
#     print ("time1=", time1, sm)
    return time1

def handler2(sm):
    time2 =utime.ticks_us() # could also use utime.ticks_us()
    # Print a (wrapping) timestamp, and the state machine object.
#     print ("time1=", time1, sm)
    return time2

# Instantiate StateMachine(0) with wait_pin_low program on Pin(26).
sm0 = StateMachine(0, jkFF1, freq=120000000, in_base= mic2_FF)
sm0.irq(handler1)

# Instantiate StateMachine(1) with wait_pin_low program on Pin(27).
sm1 = StateMachine(1, jkFF1, freq=120000000, in_base= mic1_FF)
sm1.irq(handler2)

# Now, when Pin(16) or Pin(17) is pulled low a message will be printed to the REPL.
dummy1 = True
dummy2 = True
time_diff = float()
def deltaT():
    global sm1
    global sm2
    time_diff = (handler2(dummy1) - handler1(dummy2))*0.000000001
    print(time_diff)
#     windspeed = 0.01/(time_diff) 
    print("windspeed", windspeed, "[m/s]")
#     utime.sleep_ms(1)

conversion_fact = float()
conversion_fact = 3.3/(65535)

analog = float()
#convert raw data to voltage and if V > 1.5v then store time
# def ADCscaling():
#     v = adc3.read_u16() * conversion_fact
# #     volt1.append (v.value)
#     return v

    
'''UART communication'''
#class uartComm:
    #def call(self):
def uartComm():
    poll_obj = select.poll()
    poll_obj.register(sys.stdin, 1)
    
#     global deltaT 
    while True:
#         blink()
        deltaT()
        if poll_obj.poll(0):
            ch = sys.stdin.read(1)
            if ch == 's': #press t to shutdown
                print ("Start Digital Amplifier")
                #fclk = 39000;                
                shutdownDamp.value(1)   
                
                pwm1.freq ( 25000 ) # 25kHz new ultrasonic 
                pwm1.duty_u16 ( 32767 )
                
                # Start the StateMachine's running.
                sm0.active(1)
                sm1.active(1)
                 
#                 pdm2_On(1) # commented out since we are not using pdm mics.
#                 interrupt()

                
                
            if ch == 't': # press s to start
                shutdownDamp.value (0)
                #PWM.deinit();
                print ("Shut Down Digital Amplifier")
                fclk = 1024000
                pwm2.freq ( fclk )
                pdm2_On(0)
                
                
            #for testing
            if ch == 'p':
                fclk = 3072000
                pwm2.freq ( fclk )
            if ch == 'm':
                fclk = fclk - 100
                pwm2.freq ( fclk )
                
                
#run the ticker in core 1
#_thread.start_new_thread(StateM_func, ())

while True:
    gc.collect()
    free_memory_threshold = gc.mem_free()
    uartComm()

