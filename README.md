# Basic Theory
![image (4)](https://github.com/user-attachments/assets/35bd8398-e8ee-434a-9269-5d4dd010372f)
![image (5)](https://github.com/user-attachments/assets/14456823-caf9-4228-892d-b60d82aae46d)

Forward Kinematic                                                                                    
![image (6)](https://github.com/user-attachments/assets/1da19131-9193-4970-ad7a-ac36eb17e5b5)

Inverse Kinematic                                                                                      
![image (7)](https://github.com/user-attachments/assets/1e243339-b0f2-4206-9d8d-1014f24f821b)

# Electrical Schematic
![Schematic](https://github.com/user-attachments/assets/23adb82b-1ca0-4184-ae98-45fd43e70b9c)

The picture shown below is the prototype of electrical system                                                                          
![WhatsApp Image 2024-05-07 at 17 25 49_dee94b2c](https://github.com/user-attachments/assets/2314d9c6-298e-4bae-92cd-adabcb910e99)

# STM32CubeMx Setup
All pinout setup shown below
![image (8)](https://github.com/user-attachments/assets/82a144be-df54-493e-9d8d-711106622054)

Timer 1, Timer 2, and Timer 8 is used to encoder mode. The timer settings is shown below
![Screenshot 2024-11-22 230554](https://github.com/user-attachments/assets/46db6416-7f53-4db7-ba89-8dc60172169e)

Timer 3, Timer 4, and Timer 15 is used to be timer slave with Timer 1, Timer 2, and Timer 8.
The master-slave timer can be applied by using timer interconnection by ITR Register
![image (10)](https://github.com/user-attachments/assets/6b5d5fb4-f1aa-45d7-9537-ac831202e269)

The timer pairs that can be used as master and slave are as follows :

TIM1->TIM3 (ITR0)

TIM2->TIM4 (ITR1)

TIM8->TIM15 (ITR5)

The timer setting is shown below                                                                   
![image (11)](https://github.com/user-attachments/assets/d8a0f68f-4874-4977-9de2-bf0eddfebb91)

The timer must be allowed global interrupt                                                                       
![Screenshot 2024-11-22 231210](https://github.com/user-attachments/assets/ec8e728b-ff91-4419-9ab4-a13e26ceddc1)

# 3D Visualization
![KRAI Gajah 3 v13_1](https://github.com/user-attachments/assets/95f320dc-e734-4a09-85be-c2b1ac201839)
![WhatsApp Image 2024-11-22 at 18 24 57_68995b9a](https://github.com/user-attachments/assets/1c779729-8079-4aae-9140-b9980982d660)

# Position Control System
![WhatsApp Image 2024-07-18 at 16 58 15_7dd97819](https://github.com/user-attachments/assets/67050ef1-4a5e-4da8-aa99-b3fa5bd094d6)
