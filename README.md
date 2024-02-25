Introducing the Smart Irrigation System Controller Board powered by the ESP32-S3!
![Water-Automation-PCB-Top](https://github.com/gvi70000/Smart-Irrigation-System-With-ESP32-S3/assets/248221/6131a7d8-b635-4296-a92e-ede97a524f0d)

![Water-Automation-PCB-Bottom](https://github.com/gvi70000/Smart-Irrigation-System-With-ESP32-S3/assets/248221/b1737dae-6d86-4ffd-bd94-ae18653b9981)

This hardware combines innovation and efficiency to revolutionize the way you manage your irrigation needs. Equipped with a host of advanced features, including an onboard BMP581 for precise weather data, support for up to 10 valve controls via relays, and seamless serial communication with a VL53L4CX sensor to accurately measure water levels in your well, this controller board is your key to achieving optimal irrigation and conserving valuable resources.

Say goodbye to guesswork and hello to intelligent, data-driven irrigation management, Welcome to the future of smart agriculture.

This feature-packed ESP32-S3-based Smart Irrigation System Controller Board is designed to streamline your irrigation management with unparalleled precision and control.

The board serves as the central hub for your irrigation system, seamlessly connecting and communicating with MQTT servers for efficient operation.

With 10 GPIO pins at your disposal, capable of driving up to 10 relays through 3V3 to 5V level shifters, you can effortlessly control the water flow in your irrigation system. The status of each output is visually presented using dedicated LEDs (L1 to L10 pins), providing real-time feedback to keep you informed.

For manual control of your valves, we've incorporated an analog input to read an array of push buttons. The voltage divider for the button array is thoughtfully designed for easy interfacing and intuitive operation.

The board also integrates Serial Port 1 of the ESP32-S3 to communicate with a VL53L4CX Time-Of-Flight sensor. This sensor plays a crucial role in monitoring water levels in your main reservoir or well, ensuring they remain within safe limits. The communication between the ESP32-S3 and the VL53L4CX is facilitated via an STM32F103 microcontroller, enhancing its capabilities. Notably, the power to the sensor is intelligently managed by the ESP32-S3 program to conserve energy.

To provide comprehensive environmental data, I've included a BMP581 pressure and temperature sensor on the I2C port. This allows you to access essential information such as atmospheric pressure and temperature. Additionally, the I2C port is open for additional device connections, provided that the I2C pull-up resistors are located on this controller board for your convenience.

For situations demanding higher current, the board offers a controlled output switch on S25V, directly connected to the 5V onboard power supply.

When it comes to programming, you have the flexibility to choose between the USB C port or the ESP32-S3 Serial Port 0 (Tx, Rx, 3V3, GND pins), making the process straightforward and accessible.
