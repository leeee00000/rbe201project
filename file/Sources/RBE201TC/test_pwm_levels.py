#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import urllib.request
import json
import time

def drive(action, pwm, duration):
    url = "http://127.0.0.1:8080/api/drive"
    data = json.dumps({"action": action, "pwm": pwm, "duration": duration}).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        resp = urllib.request.urlopen(req)
        print(f"[API] drive({action}, pwm={pwm}, dur={duration}) -> {resp.read().decode()}")
    except Exception as e:
        print(f"[ERR] {e}")

if __name__ == "__main__":
    print("Testing forward with PWM 220 for 1.5 seconds via web_controller API...")
    drive("forward", 220, 1.5)

