# Bidirectional-DSHOT-and-RPM-Filter
Bidirectional DSHOT and RPM Filter implementation for stm32F40X

## DSHOT/BDSHOT
DShot and BDShot implementation based on: [great description](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) and [betaflight code](https://github.com/betaflight/betaflight/tree/master/src/main/drivers).

For DShot and BDShot bitbanging (manually changing GPIOs values) implemented. Everything with DMA handling 

Tested in flight 

## RPM Filter 
The main source of the noises is the motors. Each one introduces its own frequency and since we know these values (BDShot responses) they can be eliminated from measurements with great precision. For each axes (X,Y,Z) there are created notch filters which removes motors frequencies with defined number of its harmonics. Overall there are 3x4x3 notch filters (3 axes, 4 motors, 3 harmonics). Since we know exact rpm notches are really narrow (Q = 500).

Not tested in flight yet
