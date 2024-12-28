#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Настройки сети
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);
EthernetServer server(8001);

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
MultiStepper steppers;  // Объявление steppers

void setup() {
    //Serial.begin(57600);      

    Ethernet.begin(mac, ip);
    
    // Вывод присвоенного IP-адреса
    //Serial.print("My IP address: ");
    //Serial.println(Ethernet.localIP());
    server.begin();
    //Serial.println("Server is ready");

    gripper.attach(GRIPPER_PIN);  

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].stepper = new AccelStepper(AccelStepper::DRIVER, motors[i].step_pin, motors[i].dir_pin);
        motors[i].stepper->setMaxSpeed(motors[i].max_speed);
        motors[i].stepper->setAcceleration(motors[i].acceleration);
        steppers.addStepper(*motors[i].stepper);
    }
}

void loop() {
    EthernetClient client = server.available();
    if (client) {
        //Serial.println("Client connected");
        while (client.connected()) {
            if (client.available()) {
                String data = client.readStringUntil('\n');
                //Serial.print("Received: ");
                //Serial.println(data);  // Вывод принятых данных

                long positions[NUM_MOTORS + 1];  // Позиции для двигателей и сервопривода
                if (splitString(data, positions, NUM_MOTORS + 1)) {
                    // =====================
                    // Управление шаговиками
                    steppers.moveTo(positions);
                    steppers.runSpeedToPosition();
                    
                    // Управление сервоприводом (последнее значение в positions)
                    gripper.write(positions[NUM_MOTORS]);
                    // =====================
                    //Serial.println("Motors and gripper moved.");
                } else {
                    //Serial.println("Failed to parse data.");
                }

                client.println("OKEY\n");
            }
        }
        client.stop();
        //Serial.println("Connection closed");
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
