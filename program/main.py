'''Blink the on board LED Pico W pinouts only
Patrick Palmer Jan 2024'''
from machine import Pin, Timer, PWM, UART, ADC, RTC
# from neotimer import *
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
led.on()
def blink(timer):
    led.off() 
timer.init(freq=10, mode=Timer.PERIODIC, callback=blink)


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

# adc1 = ADC(Pin(26))     # create ADC object on ADC pin GPIO26 mic1 - uncomment if using analog mic
# adc2 = ADC(Pin(27))     # create ADC object on ADC pin GPIO27 mic2 - uncomment if using analog mic 
mic1_FF = Pin (27, Pin.IN, machine.Pin.PULL_UP)     # create PIO pin GPIO27 mic1 - uncomment if using analog mic
# mic2_FF = Pin (26, Pin.IN, machine.Pin.PULL_UP)     # create PIO pin GPIO26 mic2 - uncomment if using analog mic 

# dff2 = Pin( 18, Pin.IN) # GPIO18 Dflipflop output for PDM2
# dff1 = Pin( 19, Pin.IN) # GPIO11 Dflipflop output for PDM1
        
#calculate time
# deltaT = 0
time1 = float()
time2 = float()
windspeed = float()

# @asm_pio()
'''def jkFF1(pin):
    global time1
    mic1_FF.irq(handler=None)
#     mic2_FF.irq(handler=None)
    time1 =utime.time_ns() # could also use utime.ticks_us()
#     print ("mic1 sig", pin)
    print ("time1=", time1)
    return time1

def jkFF2(pin):
    global time2
    mic2_FF.irq(handler=None)
    time2 = utime.time_ns() # could also use utime.ticks_us()
#     print ("mic1 sig", pin)
    print ("time2=", time2)
    return time2
'''

def jkFF(pin):
    global time1
    global time2
    global time_diff
    mic1_FF.irq(handler=None)

    if mic1_FF.value() == True:
        testOut.value(1)
        time1 = utime.ticks_us() # could also use utime.ticks_us()
#         print ("time1=", time1)
    elif mic1_FF.value() == False:
        testOut.value(0)
        time2 = utime.ticks_us()
#         print ("time2=", time2)
    if time1 !=0 and time2 !=0:
        time_diff = (time2 - time1)*(10**-6) # utime.ticks_diff(jkFF2(pin1)[time2], jkFF1(pin2)[time1])  # can also do this
        print("delta T=", time_diff, "s")
        windspeed = 0.01/(time_diff*0.7288) 
        print("windspeed", windspeed, "[m/s]")
        utime.sleep_us(1)
    else:
        time_diff = 1
    return time_diff

pin = True
pin1 = True
pin2 = True


def troubleshoot():
    global pin
    print("Pin 27 value is", mic1_FF.value())
#     print("Pin 27 value is", mic2_FF.value())
    print ("time_diff=", jkFF(pin))
    utime.sleep_ms(100)


def interrupt():
    global time1
    global time2
    global time_diff
    while True:
        if mic1_FF.value() == True:
            testOut.value(1)
            time1 = utime.ticks_us() # could also use utime.ticks_us()
#         print ("time1=", time1)
        elif mic1_FF.value() == False:
            testOut.value(0)
            time2 = utime.ticks_us()
#         print ("time2=", time2)
        if time1 !=0 and time2 !=0:
            time_diff = (time2 - time1)*(10**-6) # utime.ticks_diff(jkFF2(pin1)[time2], jkFF1(pin2)[time1])  # can also do this
            print("delta T=", time_diff, "s")
            windspeed = 0.01/(time_diff*0.7288) 
            print("windspeed", windspeed, "[m/s]")
            utime.sleep_us(1)
        else:
            time_diff = 1
        return time_diff
#         troubleshoot()

# pulseW = float()
#using pulse width function
def pulseWidth():
#     global pulseW  
    pulseW = machine.time_pulse_us(27, 0) # good
    print("pulse width = ", pulseW, "us") #good 
    time_diff = pulseW/1000000 # in s
    print("time diff = ", time_diff, "s") #good 
    windspeed = 0.01/(time_diff*0.7288) #- 343 #would need to use 331+0.61*Temp
    print("windspeed", windspeed, "[m/s]")
    
#     utime.sleep_ms(100)
    
_thread.start_new_thread(pulseWidth, ())
    
#try to make a pio input         
# @rp2.asm_pio( in_init = [PIO.IN_HIGH])
# def pio():
#     wait(1, pin, 27)
#     in_(pins, 27)
#                                                     
#     
# pio_sm = StateMachine(0, pio, freq=100000000, in_base=Pin(27))
# pio_sm.active(1)

# _thread.start_new_thread(interrupt, ())

    
# right here Aug 15, need more work to make the time value
'''def deltaT():
    global pin1
    global pin2
    jkFF1(pin1)
    jkFF2(pin2)
    mic1_FF.irq(trigger=machine.Pin.IRQ_RISING, handler=jkFF1)
    mic2_FF.irq(trigger=machine.Pin.IRQ_RISING, handler=jkFF2)
    time_diff = utime.ticks_diff(jkFF2(pin1), jkFF1(pin2))  # utime.ticks_diff(jkFF2(pin1)[time2], jkFF1(pin2)[time1])  # can also do this
    print("delta T=", time_diff)
    windspeed = 0.01/(time_diff**-9) 
    print("windspeed", windspeed, "[m/s]")
'''

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
        pulseWidth()
#         troubleshoot()
        if poll_obj.poll(0):
            ch = sys.stdin.read(1)
            if ch == 's': #press t to shutdown
                print ("Start Digital Amplifier")
                #fclk = 39000;                
                shutdownDamp.value(1)   
                
                pwm1.freq ( 25000 ) # 25kHz new ultrasonic 
                pwm1.duty_u16 ( 32767 )
#                 _thread.start_new_thread(adc_val, ())  
#                 utime.sleep_ms(35)
#                 shutdownDamp.value(0)
                 
                pdm2_On(1)
#                 interrupt()

#                 deltaT()
#                 PDM_adc()
#                 dff()
#                 ADCscaling()
#                 print("adc3 value =", ADCscaling())
#                 print("windspeed = ", windspeed)
                
                
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
                time.sleep(1)
                
#run the ticker in core 1
#_thread.start_new_thread(StateM_func, ())
#uart = uartComm()

# def main_loop():
#     uartComm()


while True:
    uartComm()


    

'''
poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)

def uartComm(ch):
    global poll_obj
    while True:
        if poll_obj.poll(0):
            ch = sys.stdin.read(1)
        if ch == 's': #press t to shutdown
            print ("Going to State0")
            state0.attach_transition(1, state1)
        if ch == 't': # press s to start
            state1.attach_transition(1, state0)
            print ("Going to State1")

        time.sleep(0.1)
        return ch
        
#Start working on state machine here
state_machine = StateMachine()
myTimer = Neotimer(1000)
#led = Pin(25,Pin.OUT)

#============================================================
# States Logic Functions
#============================================================
def state0_logic():
    # Referenced global variables
    # ----> Here <----
    global poll_obj
    #shutdownDamp = Pin (17, Pin.OUT) #SHUTDown Digital Amplifier GPIO 16
    
    if state_machine.execute_once:
        # ----> Code that executes just once during state <----
        # ----> Here <----
        print("Machine in State 0")
        
        myTimer.start()

    # Code that executes continously during state
    # ----> Here <----
    shutdownDamp.value (0);
    #PWM.deinit();
    print ("Shut Down Digital Amplifier")
    
    led.off()
    

def state1_logic():
    # Referenced global variables
    # ----> Here <----
    global poll_obj
    #shutdownDamp = Pin (17, Pin.OUT) #SHUTDown Digital Amplifier GPIO 16
    
    if state_machine.execute_once:
        # ----> Code that executes just once during state <----
        # ----> Here <----
        print("Machine in State 1")
        
        myTimer.start()
        
    # Code that executes continously during state
    # ----> Here <----
    shutdownDamp.value (1);
          
    print ("Start Digital Amplifier")
    #fclk = 39000;
    pwm1.freq ( fclk ) # 39kHz
    led.on()

#============================================================
# Add states to machine (Also create state objects)
#============================================================
# Create States
# ----> Here <----
state0 = state_machine.add_state(state0_logic)
state1 = state_machine.add_state(state1_logic)

#============================================================
# State Transitions Functions (optional)
#============================================================
# Create Transition Functions
# ----> Here <----
def delay_transition():
    if myTimer.finished():
        return True
    else:
        return False

#============================================================
# Attach transitions to states (optional)
#============================================================
#state0.attach_transition(delay_transition, state1)
#state1.attach_transition(delay_transition, state0)



def main_loop():
    #global variable
    global free_memory_threshold
    global ch
    # Manually invoke garbage collection
    # It might be required in both cores, but
    # start using it on the fastest looping core.

    if gc.mem_free() < free_memory_threshold:
        #lock.acquire()
        gc.collect()
        #lock.release()
        
    pass # Do something here
    
#------------------------------------------------------
# Run state machine code in second core
def state_machine_logic():
    while True:
        state_machine.run()

# Start state_machine_logic() on Core 1
_thread.start_new_thread(state_machine_logic, ())
#------------------------------------------------------


# Determine how much memory is free before starting
# main loop in Core 0. This will be used as threshold
# to determine if we should invoke gc.collect()
gc.collect()
free_memory_threshold = gc.mem_free()
print("Free memory",free_memory_threshold)

# Main Loop:
while True:
    main_loop()
'''    
