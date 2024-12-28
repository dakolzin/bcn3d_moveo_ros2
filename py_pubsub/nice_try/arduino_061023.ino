#include <AccelStepper.h>
#include <MultiStepper.h>

// Определение пинов и настройка двигателей

// Joint 1
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1,E0_STEP_PIN, E0_DIR_PIN);

MultiStepper steppers;

void setup() 
{
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(1000);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);

  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  Serial.begin(115200);

  Serial.println("READY");
}

void loop() {
  delay(10); // задержка 10 мс
  
  // Проверка, переданы ли все данные
  if (Serial.available() < 5) {
      return;
  }
  
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    // Код для очистки буфера
    while (Serial.available()) {
        Serial.read();
    }
    
    Serial.println(data); 
    
    if (data == "R?") {
        Serial.println("READY");
        return;
    }
    
    int positions[5];
    int commas = countCommast(data);
    
    // Проверка количества аргументов
    if (commas != 4) {
      Serial.println("Error: Invalid number of arguments");
      return;
    }
    
    int i = 0;
    bool parseError = false;
    
    for (int j = 0; j < 5; j++) {
      int nextComma = data.indexOf(',', i);
      if (nextComma == -1) {
        positions[j] = data.substring(i).toInt();
      } else {
        positions[j] = data.substring(i, nextComma).toInt();
        i = nextComma + 1;
      }
      // Проверка успешного разбора
      if (positions[j] == 0 && data.substring(i, nextComma) != "0") {
        parseError = true;
        break;
      }
    }
    if (parseError) {
      Serial.println("Error: Invalid data received");
      return;
    }

    long stepper_positions[5];
    stepper_positions[0] = positions[0];
    stepper_positions[1] = positions[1];
    stepper_positions[2] = positions[2];
    stepper_positions[3] = positions[3];
    stepper_positions[4] = positions[4];

    steppers.moveTo(stepper_positions);
    steppers.runSpeedToPosition(); 

    // Отправка обработанных позиций обратно на компьютер
    for (int k = 0; k < 4; k++) {
      Serial.print(stepper_positions[k]);
      Serial.print(",");
    }
    Serial.println(stepper_positions[4]);
  }
}

int countCommast(String str) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == ',') {
      count++;
    }
  }
  return count;
}