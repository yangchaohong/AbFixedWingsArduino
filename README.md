# AbFixedWingsArduino
AbFixedWings For Arduino

How to compile-Using Arduino IDE 1.x.x on Windows

1.Download https://github.com/rogerclarkmelbourne/Arduino_STM32 zip file and unzip to Your_Doc\Arduino\hardware

2.Add these libraries in Arduino IDE:

  Kalman MPU6050
  
3.Change your settings to support your STM32F103 board.

4.Click Upload button to finish the compile.

5.Connect MPU6050's VCC to 3V3

                    GND to GND

                    SCL to PB6

                    SDA to PB7

7.Connect ESC to PB1

          Left aileron to PA3

          Right aileron to PA1

          Rudder to PB0

          Elevator to PA6

8.Connect USART1(PA9,PA10) to your Air terminal by CH340/CP2102/UART etc.

9.Good Luck!
