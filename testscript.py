from machine import TouchPad, Pin
import time

touch_pin = TouchPad(Pin(4))
led = Pin(2, Pin.OUT)

THRESHOLD = 200

while True:
    touch_value = touch_pin.read()
    
    if touch_value < THRESHOLD:
        led.value(1)
    else:
        led.value(0)
        
    time.sleep(0.01)