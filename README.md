# ESP32 Etch-a-Sketch

## Technical Description

This project recreates the functionality of the iconic Etch-a-Sketch toy using
electronic components. In particular, this project is comprised of the following:
components:

* An ESP32 microcontroller running a multitasking, synchronized FreeRTOS application.
* A SPI 128x128 OLED display accessed via DMA
* 2 potentiometers utilized via ADC
* 1 GPIO button

## Setup

If you wish to run the program, below are the steps:

### Software setup

I used the ESP IDF for development of this project. In order to recreate
this example project, you will have to install the ESP IDF as explained at
[https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html)

Once you've set it up (as well as made the necessary hardware connections) you need to...

#### 1. Install the Etch-a-Sketch project

```bash
git clone git@github.com:JSpeedie/ESP32-Etch-a-Sketch.git ESP32-Etch-a-SketchGit
cd ~/esp/esp-idf
mkdir projects
cp -r ~/ESP32-Etch-a-SketchGit/ projects/etch-a-sketch
```

#### 2. Compiling and Running

After installing the project (and making the hardware connections. See below),
we are finally ready to compile and run the project:

```bash
cd ~/esp/esp-idf
get_idf
cd projects/etch-a-sketch
idf.py -p /dev/ttyUSB0 flash monitor
```

### Hardware connections

Below is the schematic I used for the example program.

<!-- <p align="center"> -->
<!--   <img src="https://raw.githubusercontent.com/wiki/JSpeedie/embedded-scribbles/images/ESP32-Tilting-Ball.png" width="50%"/> -->
<!-- </p> -->

Of course you will also need to connect a micro usb to usb cable between the
ESP32 and your development machine in order to flash the program to the ESP32
and to give it power.
