# Introduction
This is a development project, research about kinematic of Three-wheeled omnidirectional mobile robot and wheel odometry. The kinematic is better to apply, good to make trajectory system by using inverse kinematics. I try wheel odometry for robot position control and it's still experimental. I'm not sure if it is worth, but if you want to develop it I prefer you to setup additional encoder for only measure robot position.
I use STM32CubeIDE to program NUCLEO-G491RE and ArduinoIDE to program ESP32.

# Basic Theory
![image (4)](https://github.com/user-attachments/assets/35bd8398-e8ee-434a-9269-5d4dd010372f)
![image (5)](https://github.com/user-attachments/assets/14456823-caf9-4228-892d-b60d82aae46d)

Forward Kinematic                                                                                    
![image (6)](https://github.com/user-attachments/assets/1da19131-9193-4970-ad7a-ac36eb17e5b5)

Inverse Kinematic                                                                                      
![image (7)](https://github.com/user-attachments/assets/1e243339-b0f2-4206-9d8d-1014f24f821b)

# STM32CubeMx Setup
All pinout setup shown below                                                                    
![image (8)](https://github.com/user-attachments/assets/82a144be-df54-493e-9d8d-711106622054)

RCC is default from NUCLEO-G491RE                                                                                
![Screenshot 2024-11-22 231724](https://github.com/user-attachments/assets/49857a06-2b60-42e8-8799-c1fcc2666dbb)

In clock configuration, I try to use HSE 24MHz as input frequency. But i can't reach 170 MHz on MCU clock.
Another solution is using HSI 16 MHz as input frequency and let the CubeMx set the PLLCLK default and reach maximum clock 170 MHz.
You can use HSE 24 MHz to have 168 Mhz clock or use HSI 16 MHz to have 170 MHz clock.
In this case, I decide to use default clock configuration setting which is HSI 16 MHz.

HSE 24 MHz Clock Configuration                                                                                   
![Screenshot 2024-11-22 232828](https://github.com/user-attachments/assets/80a8b0a2-92d3-4ab0-abbf-30363a0e7174)
HSI 16 MHz Clock Configuration (default)                                                                             
![Screenshot 2024-11-22 234127](https://github.com/user-attachments/assets/d1abad0e-d8ec-4dee-ba4e-1b968c5204ec)

In SYS Config, I use default configuration. The Debug menu is Serial wire. For Timebase Source in default is Systick. But in this case I use TIM6 as Timbase Source because I using RTOS for some reason
![Screenshot 2024-11-22 234251](https://github.com/user-attachments/assets/477a6f57-b861-4546-8ec0-bc5121d057c0)

Timer 1, Timer 2, and Timer 8 is used to encoder mode. The settings for timer as encoder mode is shown below
![Screenshot 2024-11-22 230554](https://github.com/user-attachments/assets/46db6416-7f53-4db7-ba89-8dc60172169e)

Timer 3, Timer 4, and Timer 15 is used to be timer slave with Timer 1, Timer 2, and Timer 8.
The master-slave timer can be applied by using timer interconnection by ITR Register                                                                                                                          
Reference : https://youtu.be/3D67GEdGxGM?si=2g2nT9sZaXuv2mdo                                                                                                                          
![image (10)](https://github.com/user-attachments/assets/6b5d5fb4-f1aa-45d7-9537-ac831202e269)

The timer pairs that can be used as master and slave are as follows :

TIM1->TIM3 (ITR0)

TIM2->TIM4 (ITR1)

TIM8->TIM15 (ITR5)

The timer slave settings is shown below                                                                   
![image (11)](https://github.com/user-attachments/assets/d8a0f68f-4874-4977-9de2-bf0eddfebb91)

The timer must be allowed global interrupt                                                                       
![Screenshot 2024-11-22 231210](https://github.com/user-attachments/assets/ec8e728b-ff91-4419-9ab4-a13e26ceddc1)                                                                                                      
For Timer 15 case, we can choose TIM1 break Interrupt and TIM15 global Interrupt                                     
![Screenshot 2024-11-22 235026](https://github.com/user-attachments/assets/d8ef2df7-ec78-43db-8ecd-c0314ddfcc1d)

For PWM Generator I use Timer 20 with 3 channels. The parameter settings of Timer 20 is shown below                                       
![image](https://github.com/user-attachments/assets/a75cdf3f-30e7-4b97-a3dd-7bc2148117cc)                                                                         
Reference : https://youtu.be/OwlfFp8fPN0?si=Pdec2krsvjeB7S1X                                                                                                 

I use USART1 for communication between NUCLEO-G491RE and ESP32. I set the mode is Asynchronous because it's very easy to use. And the baudrate is 2000000 Bits/s because the communication process will be fast and ESP32 can handle it.                                      
![image](https://github.com/user-attachments/assets/318c55ac-d895-4275-a07b-066ecc411d3e)                                                                                                                                                     
Another configuration of UART is DMA. I use DMA method in this case, and on DMA Request Setting I use Circular Mode                                                                                   
![image](https://github.com/user-attachments/assets/602cf322-12e4-46e5-a7fe-8d12f7e4d924)

I2C1 for LCD with I2C peripheral. The setting I used is default.                                    
![image](https://github.com/user-attachments/assets/9b0bd69a-4726-402c-baae-3c025a9630d5)

PA8, PB4, PB5, and PB10 are setted as GPIO Input Mode                                                                                
![image](https://github.com/user-attachments/assets/dd19dc35-e222-4185-8723-32aa1044a4e8)                                                                         
PA6, PA7, PC4, PC5, PB0, PB1 are setted as GPIO Output Mode                                                                                                                   
![image](https://github.com/user-attachments/assets/a9fca8ae-160a-4936-966c-6c4c100d3204)                                                                   
In GPIO Setting, only modify GPIO Input Mode. I set them into Pull-Up Mode                                                                       
![image](https://github.com/user-attachments/assets/73436740-dace-40fa-aa4c-1e4048c97f7d)

Last thing is Real Time Operating System. Here some reference about how to applied RTOS in STM32                                                                                                  
https://embeddedthere.com/getting-started-with-freertos-in-stm32-example-code-included/                                                                                             
https://www.digikey.com/en/maker/projects/getting-started-with-stm32-introduction-to-freertos/ad275395687e4d85935351e16ec575b1?msockid=2c5a67d83c406dd9152d72d93d166c7c                                                                      
I use FreeRTOS and make three Task. Task01 for Sensor or Position Calculation. Task02 for actuator. And Task03 is optional task, its used to do STM32 system restart automatically                                                                            
![image](https://github.com/user-attachments/assets/6608b4be-e0da-40b0-a4ec-c2ac76446d56)


# Electrical Schematic
![Schematic](https://github.com/user-attachments/assets/23adb82b-1ca0-4184-ae98-45fd43e70b9c)  
I recommend to use 12V/24V Power Input                                                                                                  
The picture shown below is the prototype of electrical system                                                                          
![WhatsApp Image 2024-05-07 at 17 25 49_dee94b2c](https://github.com/user-attachments/assets/2314d9c6-298e-4bae-92cd-adabcb910e99)

The Specification of Motor DC:
https://www.tokopedia.com/mri/motor-dc-pg45-180rpm-50kgfcm-60w-200ppr-encoder?extParam=src%3Dshop%26whid%3D316656

# 3D Visualization
The 3d Design is just sketch. I lost my file design, so i cannot preview the actual scale and all measurements.
![KRAI Gajah 3 v13_1](https://github.com/user-attachments/assets/95f320dc-e734-4a09-85be-c2b1ac201839)                                                                                                    
The distance between the center point of the robot to the center point of each wheel is 26 cm.                                                                                                                 
![WhatsApp Image 2024-11-22 at 18 24 57_68995b9a](https://github.com/user-attachments/assets/1c779729-8079-4aae-9140-b9980982d660)

# Additional Feature: Position Control System
The position robot can be controlled by using wheel odometry. Wheel odometry can be obtained from from kinematic equation                                                               
![WhatsApp Image 2024-07-18 at 16 58 15_7dd97819](https://github.com/user-attachments/assets/67050ef1-4a5e-4da8-aa99-b3fa5bd094d6)

# User Interface
You can choose remote mode or autonomous mode for test wheel odometry control. If you choose remote mode, you need to connect robot with Joystick PS3 and then you can drive the robot. In autonomous mode you can setup the X,Y target position and run the robot drive to target.

# Conclusion
It's better to apply inverse kinematic to drive the robot because we don't have to set speed or PWM to each motors.
I suggest for the next research and development:
1. If you don't like the electrical system, you can remake it. Just design a compact PCB!
2. Do not use encoder motor for position measurement because it's relative. Better use AS5600, its a absolute encoder and you need I2C peripheral for it
3. If you want to use another type/specs motor, you need to check and calibrate your code especially in RPM measurement and PID Controller.
4. The User Interface should have some function like display setting PID parameter, inverse kinematic Parameter, or maybe we can changing macaddress joystick. So we don't need to change the parameters by programming. The programming process in the future will only be carried out for maintenance purposes.
5. Try to use MPU9250 to capture robot orientation data by Kalman Filter. Or you can merge the orientation data from wheel odometry and MP9250 with Complementary Filter.
6. Make a Graphical User Interface (GUI) to track and control the trajectory
7. Try to apply ROS on it.
8. Visual Odometry(?), with Monocular/RGBD/Stereo Camera
