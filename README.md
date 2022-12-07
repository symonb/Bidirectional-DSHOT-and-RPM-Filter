# Bidirectional-DSHOT-and-RPM-Filter

Bidirectional DSHOT and RPM Filter implementation, for stm32F40X
Probably you can easily adapt it to your MCU.

## BDSHOT

BDShot implementation based on: [great description](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) and [betaflight code](https://github.com/betaflight/betaflight/tree/master/src/main/drivers).

Theoretically code is correct for all DShot modes (300 600 1200). However, it was tested with [BlueJey](https://github.com/mathiasvr/bluejay) ESC software which handle maximally DShot600.

For BDShot bitbanging (manually changing GPIOs values) was implemented. There are two options of bitbanging.

Version 1:

- Every bit frame is divided into sections and for each of the section DMA request is generated.
- After specified sections (at beginning, after 0-bit time and after 1-bit time) to GPIO register can be sent value to set 1 or to set 0 as an output.
- For rest of the sections 0x0 is sent, so GPIOs don't change values.
- It uses only 1 CCR on each timer.
- Idea for reception is the same (oversampling ESC response).

Version 2:

- DMA requests generated only at beginning, after 0-bit time and after 1-bit time.
- For each bit there is only 3 sections, so buffers are much smaller than in version 1.
- However this method uses 3 CCR for each timer (probably not a big deal).
- It works for transferring but for reception it's not useful.

Of course above methods would work with standard DShot as well (you would need to change checksum calculation and invert the signal).

Tested in flight but for small scale, use with caution!

## RPM Filter

The main source of the noises is the motors. Each one introduces its own frequency and since we know these values (BDShot responses) they can be eliminated from measurements with great precision.
For each axis (X, Y, Z) there are created notch filters that remove motors frequencies with a defined number of its harmonics.
Overall there are 3x4x3 notch filters (3 axes, 4 motors, 3 harmonics).
Since we know the exact rpm - notches are narrow (Q = 500).

Notches are designed as biquad filters based on this [description](http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html) and betaflight code. For each iteration new coefficients of the notches are computed and updated for all axes.

Not tested in flight yet
