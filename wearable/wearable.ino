// -------------------------------------
// This code runs on the wearable device
// -------------------------------------
#include "definitions.cpp"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Development mode
#define INDEV

// ADAFRUIT library instance
Adafruit_MPU6050 mpu;

// Represents the state of the device.
// @type Wearable_State
Wearable_State state = AWAITING;

// Let us define all the input/output pins defined.
const int PIN_BUTTON_OUT = 2;
const int PIN_LED = 8;

// Setup function;
// Runs when the arduino starts up.
void setup() {
    // Set the pin modes;
    pinMode(PIN_BUTTON_OUT, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    // Attach the interrupts;
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_OUT), buttonIsPressed, FALLING);

    // Initialize Serial output
    Serial.begin(9600);

    // Run hardware initialization functions
    initMPU();
}

// Loops
void loop() {
    
    switch(state) {

        case AWAITING:
            // get the MPU data
            detectAcceleration();
        break;

    }
    
    delay(50);

}

void detectAcceleration() {

    float detectionThreshold = 11.0f;
    // When in development, use the potentiometer to determine the deceleration threshold.
    #ifdef INDEV
        int sensorValue = analogRead(A0);
        float fraction = (1.0f / 1024.0f) * (float)sensorValue;
        detectionThreshold = fraction * 50.0f;
    #endif

    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    float accelerationZ = a.acceleration.z;

    Serial.print("# Threshold: ");
    Serial.print(detectionThreshold);
    Serial.print(", detected: ");
    Serial.print(accelerationZ);
    Serial.println("m/s^2");

    if(accelerationZ > detectionThreshold) {
        Serial.println(accelerationZ);
        initAlertedState();
    }

}

// Initializes the MPU library for reading.
void initMPU() {
    if(!mpu.begin()) {
        Serial.println("MPU sensor was not detected. Please check the connections and restart the device.");
        return;
    }

    // Initialize the MPU class instance.
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// When either the button is pressed or a fall is detected, initialize the alerted state;
void initAlertedState() {
    state = ALERTED;
    Serial.println("Triggered Alarm State");

    digitalWrite(PIN_LED, HIGH);
}

void deactivateAlert() {
    state = AWAITING;
    Serial.println("Deactivated Alarm.");

    digitalWrite(PIN_LED, LOW);
}

// buttonIsPressed() is activated by an interrupt on the PIN_BUTTON_OUT port.
// Should activate/deactive the alert mechanism.
//      TODO: Activation/Deactivation sometimes gets triggered within the same tick/couple of ticks.
//      solution: implement timer.
void buttonIsPressed() {

    switch (state) {

        // When AWAITING, should activate the emergency signal loop.
        case AWAITING:
            initAlertedState();
        break;

        case ALERTED:
            deactivateAlert();
        break;

    }

}
