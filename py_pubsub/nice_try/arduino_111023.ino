#define SERIAL_RX_BUFFER_SIZE 256
#include <AccelStepper.h>
#include <MultiStepper.h>

// Определение пинов и настройка двигателей

typedef struct {
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    float max_speed;
    float acceleration;
    AccelStepper* stepper;
} Motor;

Motor motors[] = {
    {36, 34, 30, 1500, 500, NULL},
    {46, 48, 62, 1000, 250, NULL},
    {60, 61, 56, 2000, 1000, NULL},
    {54, 55, 38, 500, 250, NULL},
    {26, 28, 24, 1000, 500, NULL}
};

const uint8_t NUM_MOTORS = sizeof(motors) / sizeof(Motor);
MultiStepper steppers;

void setup() 
{
    Serial.begin(115200);
    Serial.println("READY");
    
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].stepper = new AccelStepper(1, motors[i].step_pin, motors[i].dir_pin);
        motors[i].stepper->setMaxSpeed(motors[i].max_speed);
        motors[i].stepper->setAcceleration(motors[i].acceleration);
        steppers.addStepper(*motors[i].stepper);
    }
}

void loop() {
    delay(10); // задержка 10 мс

    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');

        // Очистка буфера
        while (Serial.available()) {
            Serial.read();
        }

        if (data == "R?") {
            Serial.println("READY");
            return;
        }

        long positions[NUM_MOTORS];
        if (splitString(data, positions, NUM_MOTORS)) {
            steppers.moveTo(positions);
            steppers.runSpeedToPosition();

            for (uint8_t k = 0; k < NUM_MOTORS - 1; k++) {
                Serial.print(positions[k]);
                Serial.print(",");
            }
            Serial.println(positions[NUM_MOTORS - 1]);
        }
    }
    //Serial.println(SERIAL_RX_BUFFER_SIZE);

}

bool splitString(const String &data, long *output, uint8_t size) {
    uint8_t index = 0;
    for (uint8_t i = 0; i < size; i++) {
        int nextComma = data.indexOf(',', index);
        if (nextComma == -1 && i == size - 1) {
            output[i] = data.substring(index).toInt();
        } else if (nextComma != -1) {
            output[i] = data.substring(index, nextComma).toInt();
            index = nextComma + 1;
        } else {
            return false;
        }
    }
    return true;
}