from   machine import Pin
from   rp2     import asm_pio, PIO, StateMachine

GPIO_PIN = 20 # Do not connect anything to this pin

rx_pin = Pin(GPIO_PIN, Pin.IN, Pin.PULL_DOWN)


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

rx = StateMachine(0, PulseIn, in_base=rx_pin, jmp_pin=rx_pin)
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