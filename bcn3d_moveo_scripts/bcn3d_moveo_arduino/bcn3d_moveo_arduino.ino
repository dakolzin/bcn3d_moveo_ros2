#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Определение пинов и настройка для сервопривода
Servo gripper;
const uint8_t GRIPPER_PIN = 11;

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
    {46, 48, 62, 300, 150, NULL},
    {60, 61, 56, 2000, 1000, NULL},
    {54, 55, 38, 1200, 600, NULL},
    {26, 28, 24, 1000, 500, NULL}
};

const uint8_t NUM_MOTORS = sizeof(motors) / sizeof(Motor);
MultiStepper steppers;

void setup() 
{
    Serial.begin(115200);
    Serial.println("READY");

    gripper.attach(GRIPPER_PIN);
    
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].stepper = new AccelStepper(1, motors[i].step_pin, motors[i].dir_pin);
        motors[i].stepper->setMaxSpeed(motors[i].max_speed);
        motors[i].stepper->setAcceleration(motors[i].acceleration);
        steppers.addStepper(*motors[i].stepper);
    }
}

void loop() {
    delay(5); // задержка 5 мс
    
    if (Serial.available()) {
        String data = readSerialData();

        if (data == "R?") {
            Serial.println("READY");
            return;
        }

        long positions[NUM_MOTORS + 1];
        if (splitString(data, positions, NUM_MOTORS + 1)) {
            steppers.moveTo(positions);
            steppers.runSpeedToPosition();

            for (uint8_t k = 0; k < NUM_MOTORS; k++) {
                Serial.print(positions[k]);
                Serial.print(",");
            }
            gripper.write(positions[NUM_MOTORS]);
            Serial.println(positions[NUM_MOTORS]);
        }
    }
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

String readSerialData() {
    String receivedData = "";
    char inChar;
    while (true) {
        if (Serial.available()) {
            inChar = Serial.read();
            if (inChar == '\n') { // Если получен символ новой строки
                break; // Завершаем чтение
            }
            receivedData += inChar; // Добавляем символ к строке
        }
    }

    while (Serial.available() > 0) {
        Serial.read(); // Очищаем оставшиеся данные
    }
    return receivedData;
}
