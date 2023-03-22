// Dafne Vania Peña Cortés  (A01732610)
// Moisés Miles Barajas     (A01730779)

#include <Arduino_FreeRTOS.h>
#include "queue.h"

#define S1  2
#define S2  3
#define S3  4

QueueHandle_t serialGKQueue;
struct sen{
   int8_t id;
   int8_t value;
};

void setup()
{
  //open serial port
  Serial.begin(9600);
  
  //creation of tasks
  xTaskCreate(vS1Task,        "SENSOR 1 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vS2Task,        "SENSOR 2 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vS3Task,        "SENSOR 3 TASK",          100, NULL, 2, NULL);
  xTaskCreate(vGKSerialTask,  "SERIAL GATEKEEPER TASK", 100, NULL, 1, NULL);
  
  //creation of the queue
  serialGKQueue = xQueueCreate(5, sizeof(sen));
    
  //check if queue has been created
  if(serialGKQueue == NULL){
    Serial.println("Queue not created");
    while(1);
  }
}

void vS1Task(void * pvParameters)
{  
  sen infoToSend;
  infoToSend.id = 1;
  BaseType_t qStatus;

  while(1)
  {    
    infoToSend.value = digitalRead(S1);
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS)
    {
      Serial.println("S1 data NOT sent to queue"); 
    }
    else
    {
      Serial.print("S1 sent to queue: ");
      Serial.println(infoToSend.value);
    }
    
    //necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vS2Task(void * pvParameters)
{  
  sen infoToSend;
  infoToSend.id = 2;
  BaseType_t qStatus;

  while(1)
  {    
    infoToSend.value = digitalRead(S2);
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS)
    {
      Serial.println("S2 data NOT sent to queue"); 
    }
    else
    {
      Serial.print("S2 sent to queue: ");
      Serial.println(infoToSend.value);
    }
    
    //necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vS3Task(void * pvParameters)
{  
  sen infoToSend;
  infoToSend.id = 3;
  BaseType_t qStatus;

  while(1)
  {    
    infoToSend.value = digitalRead(S3);
    
    qStatus = xQueueSend(serialGKQueue, &infoToSend, portMAX_DELAY);

    if(qStatus != pdPASS)
    {
      Serial.println("S3 data NOT sent to queue"); 
    }
    else
    {
      Serial.print("S3 sent to queue: ");
      Serial.println(infoToSend.value);
    }
    
    //necessary for the operation of the serial monitor
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


void vGKSerialTask(void * pvParameters)
{
  sen senInfoReceived;
  
  BaseType_t qStatus;
  //const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  while(1)
  {
    //entering critical section
    taskENTER_CRITICAL();

    //read temperature value (always the first one)
    qStatus = xQueueReceive(serialGKQueue, &senInfoReceived, portMAX_DELAY);

    if(qStatus == pdPASS)
    {
      switch (senInfoReceived.id) {
        case 1:
          Serial.print("S1: ");
          Serial.print(senInfoReceived.value);
          break;
        case 2:
          Serial.print("S2: ");
          Serial.print(senInfoReceived.value);
          break;
        case 3:
          Serial.print("S3: ");
          Serial.print(senInfoReceived.value);
          break;
        default:
          Serial.print("Invalid sensor ID");
          break;
      }
    }
    else
    {
      Serial.println("Sensor data NOT received from queue"); 
    }

    Serial.println("");

    //exit critical section
    taskEXIT_CRITICAL();
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void loop() {}
