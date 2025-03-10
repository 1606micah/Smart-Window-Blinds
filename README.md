# Smart Window Blinds

I created an automated window blind mechanism powered by a stepper motor and an ESP32 microcontroller, with a relay integrated to ensure efficient power consumption. I established wireless communication between two ESP32 devices using the ESP-NOW protocol and incorporated a touch sensor for user input to enable real-time interaction. The system was further connected to a personalized smart home server via ESPHome, allowing localized control and effortless integration with existing smart home setups.

This repo has two Arduino code files which contain the codes for the receiver ESP32 and sender ESP32. The sender is responsible for sending the command to the receiver esp and the receiver code is responsible for both the precise movement of the motor and the execution of the wireless communication. 

Aside from the Arduino files, a few pictures show the project's setup, consisting of the motor setup and the wiring between esp32 and the motor driver.
