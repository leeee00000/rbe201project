#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time

def test():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("1. Setting motor type to 4 (TT motor without encoder)...")
    ser.write(b"$mtype:4#\r\n")
    ser.flush()
    time.sleep(0.15)
    resp = ser.read_all()
    print("   Response:", repr(resp))

    print("2. Testing forward at PWM 1800 (50% speed) for 1.5 seconds...")
    ser.write(b"$pwm:1800,1800,1800,1800#\r\n")
    ser.flush()
    time.sleep(1.5)

    print("3. Stopping...")
    ser.write(b"$pwm:0,0,0,0#\r\n")
    ser.flush()
    time.sleep(0.1)
    ser.close()
    print("Done!")

if __name__ == "__main__":
    test()

