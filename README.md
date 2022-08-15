# balance_robot_stm32# balance_robot_stm32

It's my hobby robotics project. The robot is based on STM32F411 Nuceleo-64 board as a controller. The robot uses simple complementary filter to calculate tilt angle and a PID controller.

## Hardware

- ***Controler*** - STM32F411 Nucleo-64
- ***Motors*** - Pololu 75:1 Metal Gearmotor 25D MP 12V
- ***Motor controler*** - MAX14870 Single Brushed DC Motor Driver
- ***Custom power distribution board*** - unfortunately I lost the schematic and all the files for my design  

## My code

My code can be found in */Core/Inc* (header files) and */Core/Src* (source files)

- ***balance_pid.h/balance_pid.c*** - PID controller used to control angle of the robot,
- ***comp_filter.h/comp_filter.c*** - complementary filter used to filter IMU data and calculate robots' angle,
- ***ICM20948_I2C.h/ICM20948_I2C.c*** - simple library to set up ICM20948 and read acceleration and gyro data using I2C,
- ***max14870.h/max14870.c*** - library to control PWM motor controller.

## Video

![robot](https://user-images.githubusercontent.com/81962102/184668689-62edc141-8e64-47f4-84fb-a21931672c51.gif)

## Future plans

- Create model and simulation of the robot in Python (for LQR regulator),
- Implement Kalman filter to be able to use state controller,
- Add speed controller to be able to drive robot around. 
