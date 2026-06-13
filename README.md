# payloadComputerv0
project/code repo for Pitt SOARs deployable 3U cubesat

High Level Execution
1) System armed with Switch
2) STM32 Boots up
    - Initialize IMU, BARO, Light sensor (take baseline)
    - (Optional) Turn on reactionWheel (RW_EN, connect UART) and await arm over wifi
    - Wait for liftoff
3) Liftoff detected
4) Turn on RPI as soon as liftoff
5) RPI starts camera recording on OS boot
6) STM32 data stream to rpi
7) Apogee detected (optional)
8) nose cone seperates (detect with light sensor)
    -2 second delay
9) activate pyro to cut tender descender
10) activate reaction wheel -3s after tender isd cut to give time for parachute to fillup
11) descend until landing
    - note cm4 optimizations (saving .h624) optimizations for sudden power loss due to impact
12) Upon landing stop reaction wheel, stop camera recording, stop data logging, and execute post-processing
13) (optional) after landing, shutdown OS, turnoff RPI (this can also be done after like 10 minutes of recording or something)

System Telemtry:
RPI CM4 <---STM Data---> STM32 <---CONT Data (optional), Enable signal---> RW ESP32
