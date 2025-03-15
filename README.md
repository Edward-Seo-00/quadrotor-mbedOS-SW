# Overview
This project is Mbed OS software for quadrotor drone. It was conducted as a graduation thesis team project in a major design course. 
Developed in [Arm Keil Studio Cloud](https://studio.keil.arm.com/) without Git/GitHub, resulting in no version control, with only the final product being published.

# Hardware
Development was carried out by modifying the existing entry-level drone, **SYMA X5**, to replace its original embedded system. The software of remote controller was provided by the lab, while only the hardware was built manually.

## Drone

![image](https://github.com/user-attachments/assets/e21f9b26-abf7-4fca-a962-74f612a5f0c0)

### Configuration
- microcontroller(MCU): mbed LPC1768; [offcial page](https://os.mbed.com/platforms/mbed-LPC1768/)
- IMU sensor: MPU9250; 9-axis motion tracking device
- RF module: nRF24L01; wireless transceiver module
- original drone product: SYMA X5

![image](https://github.com/user-attachments/assets/913c57f7-75e1-40df-8811-07e119b9ff5d)

## Remote Controller

![image](https://github.com/user-attachments/assets/ca96d89d-4290-4874-8abe-8bb24db70567)

### Configuration
- MCU: STM32
- RF module: nRF24L01
- Joystick: PlayStation 2 controller joystick

![image](https://github.com/user-attachments/assets/1a7be183-465d-4413-b7fa-25941134d29e)
