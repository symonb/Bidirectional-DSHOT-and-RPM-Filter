# Bidirectional-DSHOT-and-RPM-Filter

Bidirectional DSHOT and RPM Filter implementation, for stm32F40X
Probably you can easily adapt it to your MCU.

## DSHOT/BDSHOT

DShot and BDShot implementation based on: [great description](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) and [betaflight code](https://github.com/betaflight/betaflight/tree/master/src/main/drivers).

Theoretically code is correct for all DShot modes (300 600 1200). However, it was tested with [BlueJey](https://github.com/mathiasvr/bluejay) ESC software which handle maximally DShot600.

For DShot and BDShot bitbanging (manually changing GPIOs values) implemented. There are two options of bitbanging.
Version 1:

    Every bit frame is divided in sections and for each section DMA request is generated.
    After specified sections (at beginning, after 0-bit time and after 1-bit time) to GPIO register can be sent value to set 1 or to set 0.
    For rest of the sections 0x0 is sent, so GPIOs don't change values.
    It uses only 1 CCR on each timer.
    Idea for reception is the same (oversampling ESC response)

Version 2:

    DMA requests generated only at beginning, after 0-bit time and after 1-bit time.
    For each bit there is only 3 sections -> buffers are much smaller than in version 1.
    However this method uses 3 CCR for each timer (probably not a big deal).
    It works for transferring but for reception it's not useful.

Tested in flight but for small scale, use with caution!

## RPM Filter

The main source of the noises is the motors. Each one introduces its own frequency and since we know these values (BDShot responses) they can be eliminated from measurements with great precision. For each axes (X,Y,Z) there are created notch filters which removes motors frequencies with defined number of its harmonics. Overall there are 3x4x3 notch filters (3 axes, 4 motors, 3 harmonics). Since we know exact rpm notches are really narrow (Q = 500).

Not tested in flight yet
