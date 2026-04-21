# OverCar Prime

A fan-made Man-in-the-Middle BLE debug tool for Anki Overdrive,
built with two ESP32 modules and a Python HUD.

> **Disclaimer:** This is an independent fan project, not affiliated 
> with Anki, Inc. or Digital Dream Labs. See [DISCLAIMER](#disclaimer).

## What it does
- Intercepts BLE traffic between the Overdrive app and vehicles
- Decodes and displays packets in real time (Sniffer)
- Allows vehicle ID and firmware changing for research purposes

## Hardware required
- 2× ESP32
- PC with Python 3.x

## Setup
1. Flash `firmware/Prime_Carside.ino` to ESP32 #1
2. Flash `firmware/Prime_Appside.ino` to ESP32 #2
3. Install Python dependencies: `pip install customtkinter pyserial`
4. Run `python Prime.py`

## Disclaimer
OverCar Prime is an independent fan project and is not affiliated 
with, endorsed by, or connected to Anki, Inc. or Digital Dream Labs.
All trademarks remain property of their respective owners.
This tool is provided as-is for educational and research purposes only.
Use at your own risk.
