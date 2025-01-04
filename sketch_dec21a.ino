#include <esp_now.h>
#include <WiFi.h>

#define STEP_PIN 25  // GPIO pin for STEP
#define DIR_PIN 26   // GPIO pin for DIR
#define STEPS_PER_REV 200  // Steps per revolution for the NEMA17 motor

int relay = 13;
int touch = 32;
bool lastTouch = false;
int touchState = 0;

typedef struct struct_message {
    int blind_status;
} struct_message;

struct_message myData;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(myData)) {
        memcpy(&myData, incomingData, sizeof(myData));
        Serial.print("Received blind_status: ");
        Serial.println(myData.blind_status);
    } else {
        Serial.println("Data length mismatch.");
    }
}

void setup() {
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the receive callback
    esp_now_register_recv_cb(OnDataRecv);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(relay, OUTPUT);
    pinMode(touch, INPUT);  // Set the touch sensor pin as input

    // Set initial direction to forward
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(relay, LOW);
}

void stepMotor(int steps, int delayTime, int direction) {
    int rotation = 0;
    digitalWrite(DIR_PIN, direction);  // Set motor direction
    while (rotation < 4) {  // Rotate 4 full revolutions
        for (int i = 0; i < steps; i++) {
            digitalWrite(STEP_PIN, HIGH);  // Step HIGH
            delayMicroseconds(delayTime);  // Pulse duration
            digitalWrite(STEP_PIN, LOW);   // Step LOW
            delayMicroseconds(delayTime);  // Pulse interval
        }
        rotation++;
    }
}

void rotate(int touchState) {
    if (touchState == 1 && lastTouch == false) {
        Serial.println("Forward rotation");
        digitalWrite(relay, HIGH);
        delay(100);
        stepMotor(STEPS_PER_REV, 1000, LOW);  // Rotate forward
        lastTouch = true;
        digitalWrite(relay, LOW);
    } else if (touchState == 1 && lastTouch == true) {
        Serial.println("Backward rotation");
        digitalWrite(relay, HIGH);
        delay(100);
        stepMotor(STEPS_PER_REV, 1000, HIGH);   // Rotate backward
        lastTouch = false;
        digitalWrite(relay, LOW);
    }
}

void loop() {
    touchState = myData.blind_status; 
    delay(50); // Read touch sensor state
    rotate(touchState);  // Control motor based on touch input
    delay(1000);  // Wait for 1 second
}
