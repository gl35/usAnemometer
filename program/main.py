'''Blink the on board LED Pico W pinouts only
Patrick Palmer Jan 2024'''
from machine import Pin, Timer, PWM, UART, ADC
from neotimer import *
from statemachine import *
from utime import *
import time, gc, _thread
import select
import sys


timer = machine.Timer() # timer for the blinking led
timer2 = machine.Timer()


led = machine.Pin("LED", machine.Pin.OUT) #GPIO LED
shutdownDamp = Pin (17, Pin.OUT) #SHUTDown Digital Amplifier GPIO 17

pdm1_On = Pin (10, Pin.OUT) #GPIO 10 & 11 to power on and off the PDM mics
pdm2_On = Pin (11, Pin.OUT)


#LED blinking logic
led.on()
def blink(timer):
 led.toggle()
  
timer.init(freq=10, mode=Timer.PERIODIC, callback=blink)

#1st PWM - 39kHz to excit the ultrasonic generator
pwm1 = PWM ( Pin ( 15, Pin.OUT ) ) # GP15
pwm1.freq ( 1000 ) # 39kHz
pwm1.duty_u16 (32767 ) # duty 50% (65535/2)

#2nd PWM - 1.024Mhz clk for PDM mic
fclk = 1024000 #variable for PDM clock frequency
pwm2 = PWM ( Pin ( 1, Pin.OUT ) ) # GP1
pwm2.freq ( fclk ) # fclk = 1.024MHz, 3.072Mhz 
pwm2.duty_u16 (32767 ) # duty 50% (65535/2)

#ADC value for PDM mic 2
adc = ADC(Pin(26))     # create ADC object on ADC pin GPIO26
PDM2 = adc.read_u16()


dff2 = Pin( 18, Pin.IN) # GPIO18 Dflipflop output for PDM2

global ch

    
class ticker:
    def count(self):
        st = time.ticks_us()
        tic = 0
        while tic < 1000000:
            x += 1
        print(x)
        et = time.ticks_us()
        return et-st, x
    
cntr = 0
while cntr < 2:
    cntr += 1
    machine.freq(125000000)
    m = machine.freq()
    t = ticker.count
    print(m, t)
    
'''UART communication'''
#class uartComm:
    #def call(self):
poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)
while True:
    global x
    cntr
    while cntr < 2:
        cntr += 1
        machine.freq(125000000)
        m = machine.freq()
        t = ticker.count
        print(m, t)
        
    print(dff2.value())
    print(PDM2)
    
    if poll_obj.poll(0):
        ch = sys.stdin.read(1)
        if ch == 's': #press t to shutdown
            shutdownDamp.value (1)
            
            print ("Start Digital Amplifier")
            #fclk = 39000;
            pwm1.freq ( 39000 ) # 39kHz
            pwm1.duty_u16 (32767 ) 
            t = ticker.count
            pdm2_On(1)
            print(t)
            
            
        if ch == 't': # press s to start
            shutdownDamp.value (0)
            #PWM.deinit();
            print ("Shut Down Digital Amplifier")
            fclk = 1024000
            pwm2.freq ( fclk )
            pdm2_On(0)

        #for testing
        if ch == 'p':
            fclk = 3072000;
            pwm2.freq ( fclk )
        if ch == 'm':
            fclk = fclk - 100;
            pwm2.freq ( fclk )
            time.sleep(0.1)

#uart = uartComm()
#while True:
    #uart.call()
    


'''poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)

def uartComm():
    global poll_obj
    while True:
        if poll_obj.poll(0):
            ch = sys.stdin.read(1)
        if ch == 's': #press t to shutdown
            print ("Going to State0")
            state0.attach_transition(delay_transition, state1)
        if ch == 't': # press s to start
            state1.attach_transition(delay_transition, state0)
            print ("Going to State1")

        time.sleep(0.1)
        
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