#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time

def test():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
    # Turn off upload first
    ser.write(b"$upload:0,0,0#\r\n")
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("Testing $pwm:150,150,150,150# ...")
    ser.write(b"$pwm:150,150,150,150#\r\n")
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting > 0:
        print("PWM response:", repr(ser.readline()))

    time.sleep(0.5)
    print("Testing $pwm:0,0,0,0# ...")
    ser.write(b"$pwm:0,0,0,0#\r\n")
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting > 0:
        print("Stop PWM response:", repr(ser.readline()))

    print("Testing $spd:200,200,200,200# ...")
    ser.write(b"$spd:200,200,200,200#\r\n")
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting > 0:
        print("SPD response:", repr(ser.readline()))

    time.sleep(0.5)
    print("Testing $spd:0,0,0,0# ...")
    ser.write(b"$spd:0,0,0,0#\r\n")
    ser.flush()
    time.sleep(0.2)
    while ser.in_waiting > 0:
        print("Stop SPD response:", repr(ser.readline()))

    ser.close()

if __name__ == '__main__':
    test()

