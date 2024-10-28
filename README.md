# Attitude Indicator for STM32

An Attitude Indicator Example designed for STM32F1 with Arduino IDE

Please be aware that this is only an example and still under development.
This project is mainly used for my school paperwork.

## Mainboard Hardware Specification

Board Information:
Bluepill STM32F103C8T6 with 64K space and external CH340G serial.
Detailed pinout and introduction can be found here:

https://components101.com/microcontrollers/stm32f103c8t8-blue-pill-development-board

Data will be shown on the 1.44-inch TFT screen with an interface that drawn by myself.
And there will be key and serial handler to execute some actions like change the sensitivity or etc.
The system used ST7735 as TFT controller and MPU6050 as main sensor.
Also, a HC-SR04 is used to detect obstacle. This is an optional function.

## Sub-board Hardware Specification

Board Information:
AI Thinker General ESP-01(S) Board with W25Q80 8Mbit(1MB) flash.
Detailed information can be found here:

https://www.esp8266.com/wiki/doku.php?id=esp8266-module-family

This module is mainly used for firmware OTA and network stuff.
Data will be shown in the web browser that can be accessed either by its own hotspot,
or use USB serial to make it connect to an existing wireless access point.

## Get Started

1. Download Adafruit's MPU6050 and ST7735 Library
2. Download Erick Simoes' Ultrasonic Library
3. Download Piotr Stolarz's CoopThreads Library
4. Apply Patch
5. Compile

#### Patch

This patch is needed for ST7735 to work on the STM32F1.

https://github.com/adafruit/Adafruit-GFX-Library/pull/365/files

https://github.com/adafruit/Adafruit-ST7735-Library/pull/156/files

## TODO List

1. Real hardware testing. (This is always the first priority.)
2. ~~ LCD Interface design.~~
3. ~~Key interactive functions.~~
4. Algorithm(vertical speed etc.).
5. Data output format re-design.
6. ESP8266 OTA support.
7. Web interface design.
8. PCB design and final product design.
9. GensouRTOS framework cleanup and packaging.
10. Try port the whole code framework to KEIL.

### Project GensouRTOS:

Welcome to the GensouRTOS Develop Environment!
Basically I'm designing a full-state acceptable system w/redundancy support :)
And all of the system design must fit in a 64k flash space.

**GensouRTOS is NOT an RTOS or an Operation System.**

It's just a code framework for myself. By using the framework I can tweak the system
into many creative projects like this one or other code that I wrote.
The basic development idea is to create a reusable, error-recoverable, user-friendly,
easy-to-copy code framework for my all projects that based on minimal hardware
like STM32F103C8T6 or RP2040 based system.

### Why not FreeRTOS:

Yeah, FreeRTOS is a much better idea. But when you are actually developing on the bare
metal, for myself, my first idea is why not use my knowledge learned from 8051, that's
why I want to program directly on the bare metal. It's funny, though it may not be the
best solution ever created, it won't run so fast, won't run so smooth. But all code I
wrote is an valuable experience for myself.

## Special Thanks to:

Adafruit, Erick Simoes, Piotr Stolarz and STM32duino community and all library providers!
Without those effort, the project can't develop as smooth as possible.
