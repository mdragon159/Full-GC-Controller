# Legacy Due Code
This folder contains older test code used to attempt to read sequences of data from a separate console-to-GC data. Unfortunately, these two sets of code (and with various variations) doesn't work properly. In other words, it doesn't read correctly compared to the output it should really have. Finally, flash wait states may be the greatest reason why these tests don't work on the Due.

Also, take note that these sets of code are meant to work only with the Arduino Dues.

# GC_Test
First series of tests using simpler methods, without using pure assembly or such.

# GC_Test_Timer
First test with a pure timer and interrupts (using a timer library used for the Due).