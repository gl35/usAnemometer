'''Blink the on board LED Pico W pinouts only
Patrick Palmer Jan 2024'''
from machine import Pin, Timer, PWM, UART, ADC, RTC
from neotimer import *
from rp2 import PIO, StateMachine, asm_pio
import utime 
#from array import *
import time, gc, _thread
import select
import sys
import array


rtc = RTC()

timer = machine.Timer() # timer for the blinking led

led = machine.Pin("LED", machine.Pin.OUT) #GPIO LED
shutdownDamp = Pin (17, Pin.OUT) #SHUTDown Digital Amplifier GPIO 17

pdm1_On = Pin (10, Pin.OUT) #GPIO 10 & 11 to power on and off the PDM mics
pdm2_On = Pin (11, Pin.OUT)


#LED blinking logic
# led.on()
def blink(timer):
    led.off()  
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

# pwm3 = PWM ( Pin (22, Pin.OUT) ) # GP22 for dflipflop 48000hz
# pwm3.freq (16000000)
# pwm3.duty_u16 (32767)

pdm_Dout = machine.Pin (16, machine.Pin.OUT)

#ADC value for PDM mic 2
# pdm1 = ADC(Pin(26))     # create ADC object on ADC pin GPIO26 - variable name is pdm1 and not adc1 - uncomment if using pdm mic

adc1 = ADC(Pin(26))     # create ADC object on ADC pin GPIO26 mic1 - uncomment if using analog mic
adc2 = ADC(Pin(27))     # create ADC object on ADC pin GPIO27 mic2 - uncomment if using analog mic 

analogV_arr1 = [] #needs more work on array
time_arr1 = []
volt1 = []

analogV_arr2 = [] #needs more work on array
time_arr2 = []
volt2 = []

global ch

machine.freq(125000000)

#analog mic adc value function
i =0;
def adc_val():
    for i in range(0, 400):
        time_arr1.append( utime.time_ns())
        analogV_arr1.append( adc1.read_u16())
        #volt1.append(analogV_arr1)
        
        time_arr2.append( utime.time_ns())
        analogV_arr2.append( adc2.read_u16())


#PDM to ADC function
def PDM_adc():
    for i in range(0, 250):
        analogV_arr[i] = (pdm1.read_u16()/4096)
        #print("Analog Value =", analogV_arr)
        utime.sleep_us(20)

conversion_fact = float()
conversion_fact = 3.3/(65535)
analog = float()
#convert raw data to voltage and if V > 1.5v then store time
def ADCscaling(val_in):
    v = val_in * conversion_fact
#     volt1.append (v.value)
    return v
#ADC2 is the pdm mic for testing the decimaton

n = 15
i = 0
adc_thres = 0.15

pdmval_2_u16 = [None]*n
pdmval_2_raw = [None]*n
def decimation():
    global pdmval_2_u16
    global i
    global n    
#         print(pdmval_2_raw)
    if ADCscaling(adc2.read_u16()) >= adc_thres:  #somethings if the reading is below the threshold and will cause the array error, decrease the threshold so we get the reading        
        for i in range (0, 15):
            pdmval_2_raw[i] = ADCscaling(adc2.read_u16())
            pdmval_2_u16[i] = pdmval_2_raw[i]
            
            utime.sleep_us(20)
            i = i + 1
        if i > 15:
            i = 0;
#     print(pdmval_2_u16)
def pdmOut():
    for i in range (0, 15):
        if pdmval_2_u16[i] > 0.2:
            pdm_Dout.on()
        elif 0 < pdmval_2_u16[i] <= 0.2: 
            pdm_Dout.off()
#     print(pdm_Dout.value())
cntr = 0
    
'''UART communication'''
#class uartComm:
    #def call(self):
def uartComm():
    poll_obj = select.poll()
    poll_obj.register(sys.stdin, 1)
    
#     global deltaT 
    while True:
        decimation()
        pdmOut()
        if poll_obj.poll(0):
            ch = sys.stdin.read(1)
            if ch == 's': #press t to shutdown
                print ("Start Digital Amplifier")
                #fclk = 39000;                
                shutdownDamp.value(1)   
                
                pwm1.freq ( 39000 ) # 39kHz
                pwm1.duty_u16 ( 32767 )
#                 _thread.start_new_thread(adc_val, ())  
#                 utime.sleep_ms(35)
#                 shutdownDamp.value(0)
                 
                #t = ticker.count
                pdm2_On(1)
                decimation()
#                 calTime()
                
#                 PDM_adc()
            
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
                decimation()
            if ch == 'm':
                fclk = fclk - 100
                pwm2.freq ( fclk )
                time.sleep(1)



while True:
    uartComm()
#     ADCscaling(volt1)
    
