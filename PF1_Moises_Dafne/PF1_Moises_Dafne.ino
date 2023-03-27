// Dafne Vania Peña Cortés  (A01732610)
// Moisés Miles Barajas     (A01730779)

#include <Arduino_FreeRTOS.h>
#include "queue.h"
#include "semphr.h"
#include <Keypad.h>                     // https://github.com/Chris--A/Keypad
#include <EasyBuzzer.h>

#define S1  A0                          // Fotoresistencia
#define S2  A1                          // Potenciómetro
 
const int buzzPin = 3;                  // Buzzer
const int EchoPin = 4;                  // Ultrasónico
const int TriggerPin = 2;

const byte ROWS = 4;                    // Rows on the keypad
const byte COLS = 4;                    // Columns on the keypad
char keys[ROWS][COLS] = {               // Define the keypad matrix
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = { 12, 11, 10, 9 }; // Row pins of the keypad
byte colPins[COLS] = { 8, 7, 6, 5 };    // Column pins of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Queue handle
QueueHandle_t serialGKQueue;
//Semaphore handle
SemaphoreHandle_t xBuzzerMutex;

String password = "0000";
String inputPw = "";
int8_t enableAlarm = 1;
int8_t alarmOn = 0;

struct sen {
   int8_t id;
   int8_t value;
};

void setup() {
  Serial.begin(9600);
  
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // Buzzer pin configuration
  EasyBuzzer.setPin(buzzPin);
  
  // Creation of tasks
  xTaskCreate(vS1Task,        "SENSOR 1 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vS2Task,        "SENSOR 2 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vS3Task,        "SENSOR 3 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vGKSerialTask,  "SERIAL GATEKEEPER TASK", 100, NULL, 1, NULL);
  xTaskCreate(vKeypadTask,    "KEYPAD HANDLER TASK",    100, NULL, 2, NULL);

  // Creation of the queue
  serialGKQueue = xQueueCreate(4, sizeof(sen));
    
  // Check if queue has been created
  if (serialGKQueue == NULL){
    Serial.println("Queue not created");
    while(1);
  }

  //Creation of the binary sempahore
  xBuzzerMutex = xSemaphoreCreateMutex();

  // Check if mutex has been created
  if(xBuzzerMutex == NULL)
  {
    Serial.println("Mutex not created");
    while(1);
  } else {
    // Give the semaphore outside the infinite loop
    xSemaphoreGive(xBuzzerMutex);
  }
}

void getInput() {  
  char key;
  int n = 4;

  inputPw = "";
  for(int i=0; i<n; i++) {
    key = keypad.getKey();
    while(key == NO_KEY) { // When a key is not pressed
      key = keypad.getKey();
    }
    inputPw += key;
  }
  Serial.println(inputPw);
  
}

void configPassword() {
  getInput();

//  byte at = 0;
//  const char *p = inputPw;
//  password = "";
//  
//  while(*p++) {
//    password.concat(inputPw[at++]);
//  }

  password = inputPw;
  
  Serial.println(password);
  buzzerOn(4);
}

void enAlarm() {
  getInput();
  if(password == inputPw) {
    if(enableAlarm == 1){
      enableAlarm = 0;
    } else {
      enableAlarm = 1;
    }
  }
}

void alarmOff() {
  getInput();
  if(alarmOn == 1 && password == inputPw){
    alarmOn = 0;
  }
}

void buzzerOn(int sensor) {
  //Take semaphore
  xSemaphoreTake(xBuzzerMutex, portMAX_DELAY);
  switch(sensor) {
    case 1:
      // Activate buzzer for a moment (60 Hz, 1 s)
      EasyBuzzer.singleBeep(60, 200);
      break;
    case 2:
      // Activate buzzer for a moment (60 Hz, 1 s)
      EasyBuzzer.singleBeep(100, 600);
      break;
    case 3:
      // Activate buzzer for a moment (60 Hz, 1 s)
      EasyBuzzer.singleBeep(140, 1000);
      break;
    case 4:
      // Activate buzzer for a moment (60 Hz, 1 s)
      EasyBuzzer.singleBeep(20, 1000);
      break;
    default:
      // Activate buzzer for a moment (60 Hz, 1 s)
      EasyBuzzer.singleBeep(60, 1000);
      break;
  }

  //Give semaphore
  xSemaphoreGive(xBuzzerMutex);
}

int ping(int TriggerPin, int EchoPin) {
  long duration, distanceCm;
  
  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  
  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292 / 2;   //convertimos a distancia, en cm
  return distanceCm;
}

// Fotoresistencia
void vS1Task(void * pvParameters) {  
  sen infoToSend;
  infoToSend.id = 1;
  BaseType_t qStatus;
  int analogValue;

  while(1) {
    // Entering critical section
    taskENTER_CRITICAL();
    
    analogValue = analogRead(S1);
    analogValue = map(analogValue, 0, 1023, 0, 255);
    if(analogValue >= 100) {
      infoToSend.value = 1;
    } else {
      infoToSend.value = 0;
    }
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS) {
      Serial.println("S1 data NOT sent to queue"); 
    } else {
      Serial.print("S1 sent to queue: ");
      Serial.println(infoToSend.value);

      if(infoToSend.value == 1) {
        alarmOn = 1;
        buzzerOn(1);
      } else {
        alarmOn = 0;
      }
    }

    // Exit critical section
    taskEXIT_CRITICAL();
    
    // Necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(900));
  }
}

// Potenciómetro
void vS2Task(void * pvParameters) {  
  sen infoToSend;
  infoToSend.id = 2;
  BaseType_t qStatus;
  int analogValue;

  while(1) {
    // Entering critical section
    taskENTER_CRITICAL();
    
    analogValue = analogRead(S2);
    analogValue = map(analogValue, 0, 1023, 0, 255);
    if(analogValue >= 127) {
      infoToSend.value = 1;
    } else {
      infoToSend.value = 0;
    }
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS) {
      Serial.println("S2 data NOT sent to queue"); 
    } else {
      Serial.print("S2 sent to queue: ");
      Serial.println(infoToSend.value);

      if(infoToSend.value == 1) {
        alarmOn = 1;
        buzzerOn(2);
      } else {
        alarmOn = 0;
      }
    }
    
    // Exit critical section
    taskEXIT_CRITICAL();
    
    // Necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(900));
  }
}

// Ultrasónico
void vS3Task(void * pvParameters) { 
  sen infoToSend;
  infoToSend.id = 3;
  BaseType_t qStatus;
  int cmDist;

  while(1) {
    // Entering critical section
    taskENTER_CRITICAL();
    
    cmDist = ping(TriggerPin, EchoPin);
    if(cmDist <= 10) {
      infoToSend.value = 1;
    } else {
      infoToSend.value = 0;
    }
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS) {
      Serial.println("S3 data NOT sent to queue"); 
    } else {
      Serial.print("S3 sent to queue: ");
      Serial.println(infoToSend.value);

      if(infoToSend.value == 1) {
        alarmOn = 1;
        buzzerOn(3);
      } else {
        alarmOn = 0;
      }
    }
    
    // Exit critical section
    taskEXIT_CRITICAL();
    
    // Necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(900));
  }
}


void vGKSerialTask(void * pvParameters) {
  sen senInfoReceived;
  
  BaseType_t qStatus;
  //const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  while(1) {
    // Entering critical section
    taskENTER_CRITICAL();

    if(enableAlarm == 1) {
      qStatus = xQueueReceive(serialGKQueue, &senInfoReceived, portMAX_DELAY);
  
      if(qStatus == pdPASS) {
        switch (senInfoReceived.id) {
          case 1:
            Serial.print("(S1, ");
            Serial.print(senInfoReceived.value);
            Serial.print(")");
            break;
          case 2:
            Serial.print("(S2, ");
            Serial.print(senInfoReceived.value);
            Serial.print(")");
            break;
          case 3:
            Serial.print("(S3, ");
            Serial.print(senInfoReceived.value);
            Serial.print(")");
            break;
          default:
            Serial.print("Invalid sensor ID");
            break;
        }
      } else {
        Serial.println("Sensor data NOT received from queue"); 
      }
    }

    Serial.println("");

    // Exit critical section
    taskEXIT_CRITICAL();
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vKeypadTask(void * pvParameters) {
  char key;

  while(1) {
    key = keypad.getKey();

    if (key != NO_KEY) { // When a key is pressed
      Serial.println(key);
      switch(key){
        case 'A':
          configPassword();
          break;
        case 'B':
          enAlarm();
          break;
        case 'C':
          alarmOff();
          break;
        default:
          Serial.println("Not a valid option");
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void loop() {
  // Función para que funcione la librería
  EasyBuzzer.update();
}
