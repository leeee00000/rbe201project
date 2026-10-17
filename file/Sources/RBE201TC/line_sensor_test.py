#!/usr/bin/env python3
import Hobot.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BOARD)


pins = {
    'X1': 15,  
    'X2': 18,  
    'X3': 16,   
    'X4': 11,   
}

print("=" * 50)
print("Line-following module test program (pin mapping corrected)")
print("=" * 50)

for name, pin in pins.items():
    try:
        GPIO.setup(pin, GPIO.IN)
        print(f"✅ {name} Physical pin initialized successfully")
    except Exception as e:
        print(f"❌ {name} Physical pin initialization failed: {e}")

print("\nNow place the module on the white/black surface and observe the changes in all channel readings.")
print("Press Ctrl+C to exit.\n")

try:
    while True:
        values = {}
        for name, pin in pins.items():
            try:
                values[name] = GPIO.input(pin)
            except Exception as e:
                values[name] = "ERR"
        
        # Print the current reading
        print(f"\rX1:{values['X1']}  X2:{values['X2']}  X3:{values['X3']}  X4:{values['X4']}  ", end='')
        sys.stdout.flush()
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\n End Of Test")
finally:
    GPIO.cleanup()
    print("GPIO Cleaned up")
