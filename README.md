# ESP32 Etch-a-Sketch

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/9b4392fa-0ebb-4d07-ba5f-08a906804ac6

## Technical Description

This project recreates the functionality of the iconic Etch-a-Sketch toy using
electronic components. In particular, this project is comprised of the following
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

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/ESP32-Etch-a-Sketch.png" width="60%"/>
</p>

Of course you will also need to connect a micro usb to usb cable between the
ESP32 and your development machine in order to flash the program to the ESP32
and to give it power.

&nbsp;
&nbsp;

## Project Report

### Straight Lines

The most challenging part of this project was getting the device to draw
perfectly horizontal or vertical lines. This is the expected behaviour when a
user turns only one of the dials. With no noise filtering, the output from
the ADC the potentiometers are connected to looks like this:

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/db69f253-d8f8-4f31-abba-7b19f651e04a

Of course this isn't acceptable for this project, and I had to mitigate the
noise in some way. The first thing I tried was using the mean of a collection
of samples taken each tick and combining that with a Moving Average filter.
Taking 3 samples per tick and using the means from the last 20 ticks for our
moving average, we get a result that looks like this:

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/b3a43953-420c-43f7-b8eb-649d14180485

It's much, much better, but turning only one of the horizontal or vertical
dials results in choppy lines that are decorated with little spikes or
deviations where the program had one moment calculated that the cursor should
be on the adjacent line, and the next calculated that the first line was
correct. Noise from the ADC/potentiometers is still causing cursor shakiness so
more needs to be done.

My next attempt was to take the median from a collection of samples taken each
tick. The thinking here was that outliers in the samples of a given tick are
likely noise, and in averaging the samples we aren't ignoring noise but merely
reducing its impact. An improvement would be to ignore the outlier values by
taking not the mean of the samples, but their median. This led me to change the
code to take the median rather than the mean and to put that through a 5-point
Moving Average filter (down from 20-point). This results in cursor movement
looking like this:

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/7c6bf2ee-3b1b-474a-a1b1-549f72d5b65c

Moving in the right direction! This was much closer to a 1 pixel wide vertical
or horizontal line than the previous method, but it's still not where we want
it. Further complicating things, while there are less points in the moving
average calculation, the code I used here took 20 samples per tick (up from 3).
So while the result was less cursor shakiness, it comes at the cost of both
significantly more samples as well as sorting the data so the median can be
found. With so much sampling and calculating going on in this potential
solution, I decided that whatever I tried next would have to be something
beyond increasing the sampling or averaging.

My final attempt came from the perspective that it was not necessarily noise
that was causing cursor shakiness, but rather the game design I had used thus
far. My thought was that no matter how much we reduce noise, there will always
be hypersensitive, borderline positions for the potentiometers where if they
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
code that takes these hypersensitive cases into account.

First, the cursor's position - which is stored as an integer, a pixel position
on the screen - should be represented as its integer representation cast to a
double with 0.5 added to it. This way when the ADC produces a double
representing what it thinks should be the cursor's next x or y position, we can
compare it to what is more essentially the cursor's x or y position: a double
that represents the cursor as being in the middle of the pixel it is in! This
immediately has the effect of reducing the impact of noise by eliminating
those hypersensitive, borderline cases.

Treating the cursor's position as its integer value cast to a double with 0.5
added to it allows for another method of combatting noise, however. First,
let's change how the game updates the cursor's position by calculating the
cursor's new position not by mapping the potentiometers' positions to screen
coordinates, but rather by taking the calculated current position for the
cursor, subtracting the previous cursor's position (represented as the cursor's
previous position as an integer pair, cast to doubles, with 0.5 added to each)
and adding that to cursor's previous position. This seems to only complicate a
simple calculation, but really what it is doing is making room for the real
solution: a certainty factor function.

The certainty factor function goes a long way in reducing the cursor shakiness.
Essentially, when the game calculates that the new position for the cursor is a
small distance away from the cursor's previous position (e.g. (83.23, 89.30) ->
(83.31, 89.28) ), then we are less certain that that was indeed a deliberate
change in input from the user. Small changes to the cursor position have a
greater likelihood of being the result of noise, or largely skewed by noise. My
thought here was that we need a function that maps changes in the cursor x or y
to what could be compared to a probability. Specifically, we need small changes
to result in very low certainties (close to 0), and large changes to map very
close to 1. The idea then is to multiply the calculated change in x and y for
the cursor by the certainty factor so that small changes become smaller, and
less likely to move the cursor to any adjacent pixel. At first I thought that
it sounded like a slightly modified Sigmoid function would do the job, but the
ramp from squishing values (the function returning something like 0.001) to
approximating values (the function returning something like 0.999) was not
steep enough. I ended up designing an purpose-built function based off of the
asymptotic function $1/x$. The first attempt I felt was worth committing used
the function:

```math
\frac{1}{{-(|x|-0.25)}^{4}-1}+1
```

Which produced cursor movement that looked like this:

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/5c1ace87-be00-4200-8bcc-acacf11ce73b

The certainty factor function I use at the time of writing is this:

```math
\frac{2}{{-(|x|-0.25)}^{4}-2}+1$
```

And it produces cursor movement that looks like this:

https://github.com/JSpeedie/ESP32-Etch-a-Sketch/assets/11791739/f6ffbc3a-4225-4dca-be5c-46276bd17728

You can see the cursor shakiness has been almost entirely eliminated.
Essentially this function has the effect of giving the row or column the cursor
is currently on a sort of weak gravity, or stickiness. Small x or y changes are
effectively shrunk, and this increases the chance that the x or y change will
be small enough that rounding it will return 0. On the other hand, past a
certain magnitude, a given x or y change will be basically unaltered, losing
that gravity/stickiness.

Other changes I've made since adding the certainty factor function is to take a
narrow (e.g. 3 point) mean around the median in case there are 2 dominant
values near the median. In testing I found this produce less shakiness than
simply taking the median.

I also tried a variety of potential hardware solutions to the noise by using
capacitors. Almost all of these tended to increase noise rather than reduce it.
I can't explain why this was the case as the ESP32 page on ADC recommends using
a capacitor right at the ADC pin you're using to reduce noise, but when
I tried that the noise was more volatile I found. Sometimes across 1000 samples
there would be less standard deviation for the ADC that had a capacitor attached,
but most of the time the standard deviation would double or triple! Something
to look into another time.

With all that said, I've found that the certainty factor function makes a huge
difference! If I had more time I'd love to look into other filters, but for
now, I'm very happy with how reliably the user can draw straight lines.

### Certainty Factor Function Comparisons

For completeness, here's how my first 2 attempts at the certainy factor function compare:

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/certainty_factor_first_attempt_v_second_attempt.png" width="80%"/>
</p>

And here's a graph showing how my first attempt and second attempt
certainty factor functions affect a given change in x/y position. The y axis of
the dotted line represents the filtered x or y coordinate the cursor will end
up experiencing, while the x axis represents the what that change in x or y
coordinate was pre-filter.

<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/JSpeedie/ESP32-Etch-a-Sketch/images/certainty_factor_first_attempt_v_second_attempt_plus_output.png" width="70%"/>
</p>

&nbsp;

### Drawing the Lines

Another problem that had to be addressed if the project was to be acceptably
complete was that moving the cursor quickly should not leave gaps in the line
traced by the cursor's movement. One way this can be remedied is simply by
increasing the ESP32's tick rate and polling more frequently. This is a poor
solution because you give the tasks less time to complete and you increase
the power consumption of the device.

The solution I implemented is to draw a straight line (or trail) between the
cursor's new position and the cursor's previous position. First I used
the Pythagorean theorem to calculate the length of a straight line between
the two positions and then I used a loop to paint the trail. The loop iterated once for each
unit of length of the line, each iteration incrementing the x and y point where the game
would paint a pixel by the change in x or y divided by the length of the line.

There are two problems with this solution, the first being that it only paints
a straight line, and it is entirely possible that the user actually moved the
cursor along a non-linear curve between the two points. The effect of this
problem is minimized by the fact that even at a relatively low tick speed, the
ESP32 updates the cursor's position so fast that any difference between the
cursor's previous position and its current position are so minor that
approximating it as a straight line leads to little to no discrepancy between
what appears on screen and what the user would expect. The second problem
is that the way in which this solution paints the line can lead to "steppy" lines.
Non-cardinal lines always requires some special handling on screens to ensure
that they look no thinner or wider than a perfectly straight line along
the horizontal and vertical axes. Since this is a simple and short project, I
chose not to pursue a proper solution to this problem at this time, and the result
is that some of the diagonal lines look thick and "steppy".
