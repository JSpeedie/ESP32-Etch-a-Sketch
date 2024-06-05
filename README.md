# ESP32 Etch-a-Sketch

## Technical Description

This project recreates the functionality of the iconic Etch-a-Sketch toy using
electronic components. In particular, this project is comprised of the following:
components:

* An ESP32 microcontroller running a multitasking, synchronized FreeRTOS application.
* A SPI 128x128 OLED display accessed via DMA
* 2 potentiometers utilized via ADC (+ several processes to reduce noise)
* 1 GPIO button

For more information about how this project was completed, please see the
[Project Report section](#project-report).

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

&nbsp;
&nbsp;

## Project Report

### Straight Lines

The most challenging part of this project was getting the device to draw a
perfectly horizontal or vertical lines. This is the expected behaviour when a
user turns only one of the dials. In the early stages of the project when the
core functionality had just barely been achieved, turning only one of the
horizontal or vertical dials resulted in chopping lines that would be decorated
with little spikes where the program had one moment calculated that the cursor
should be on the adjacent line, and the next calculated that the first line was
correct. It seemed to me that the cause of this was noise from the
ADC/potentiometers. My thought was that if values from the ADCs (representing
the rotation of the potentiometer) are not accurately read, there might be a
pixel or three of shake in the cursor.

This led to my first attempt to solve the problem. If the signal is noisy,
perhaps taking multiple samples per tick and then averaging them would produce
a stable value. When implemented, this did reduce noise, but for the
Etch-a-Sketch, even 1 pixel of shake in the cursor leads to noticeably choppy
lines. Another solution would be needed.

My next attempt was to take the median from a collection of samples taken each
tick. The thinking here was that outliers in the samples of a given tick are
likely noise, and in averaging the samples we aren't ignoring noise but merely
reducing its impact. An improvement would be to ignore the outlier
values by taking not the mean of the samples, but their median. For good
measure, I also kept the last 2 medians from previous ticks and averaged the
median for the current tick with them in an attempt to reduce cursor shakiness
further. Ultimately I did not find that this attempt was significantly better
than just taking an average across a large sample size. Sorting the sample data
taken on a tick is fairly expensive and this did not come with a compensatory
reduction in the number of samples taken per tick - in fact I increased how
many samples I took!

My final attempt took the view that it was not necessarily noise that was
causing cursor shakiness, but rather the game design I had used thus far. My
thought was that no matter how much we reduce noise, there will always be
hypersensitive, borderline positions for the potentiometers where if they
rotate a tiny, tiny degree, the value will cross a threshold and the game will
(correctly) move the cursor. For example, if, hypothetically, the "true" (i.e.
noise free) value of our potentiometer at a given moment translates to the in
game cursor x position of 43.00, that would be a hypersensitive scenario.
Compare this to a situation where the "true" value of the potentiometer gives
us an in game cursor x position of 43.50. In the first scenario it would take a
change of only 0.01 in the calculated cursor x position for the game to move
the cursor to position 42 - an effect noise could easily have on the mean and
median calculations. Since we cannot remove noise entirely, what this means is
that there will always be positions where noise is more likely to introduce
shakiness. The conclusion I drew from this is that we need to implement some
sort of input smoothing code that takes these hypersensitive cases into
account. The solution I thought of is to calculate the cursor's new position
each tick not by mapping the potentiometers' positions to screen coordinates,
but rather by taking the calculated current position for the cursor,
subtracting the previous cursor's position (represented as the cursor's
previous position as an integer pair, cast to doubles, with 0.5 added to each)
and adding that to cursor's previous position. On its own, this only
complicates a simple calculation, however, it makes room for the real solution:
a certainty factor function.

The certainty factor function is the piece of the puzzle that puts this problem
to bed. Essentially, when the game calculates that the new position for the
cursor is a small distance away from the cursor's previous position (e.g.
(83.23, 89.30) -> (83.31, 89.28) ), then we are less certain that that was
indeed a deliberate change in input from the user. Small changes to the cursor
position have a greater likelihood of being the result of noise, or largely
skewed by noise. My thought here was that we need a function that maps changes
in the cursor x or y to what could be compared to a probability. Specifically,
we need small changes to result in very low certainties (close to 0), and large
changes to map very close to 1. The idea then is to multiply the calculated
change in x and y for the cursor by the certainty factor so that small changes
become smaller, and less likely to move the cursor to any adjacent pixel. At
first I thought that it sounded like a slightly modified Sigmoid function would
do the job, but the ramp from squishing values (the function returning
something like 0.001) to approximating values (the function returning something
like 0.999) was not steep enough. I ended up designing an purpose-built
function based off of the asymptotic function $1/x$. The first attempt I felt
was worth committing used the function:

```math
\frac{1}{{-(x-0.25)}^{4}-1}+1
```

What I have committed at the time of writing is:

```math
\frac{2}{{-(x-0.25)}^{4}-2}+1$
```

Here's how they compare:

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/certainty_factor_first_attempt_v_second_attempt.png" width="80%"/>
</p>

I also considered a more aggressive version of my second attempt shown in orange here:

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/certainty_factor_first_attempt_v_second_attempt_v_possible_third.png" width="80%"/>
</p>

Finally, here's a graph showing how my first attempt and second attempt
certainty factor functions affect a given change in x/y position. The y axis of
the dotted line represents the filtered x or y coordinate the cursor will end
up experiencing, while the x axis represents the what that change in x or y
coordinate was pre-filter.

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/certainty_factor_first_attempt_v_second_attempt_plus_output.png" width="70%"/>
</p>

&nbsp;

### Drawing the Lines

tbd.  a^2 + b^2 = c^2, we want to draw a trail to avoid leaving gaps when the user
moves the potentiometer quickly.
