/* FreeRTOS.org includes. */
#include "Arduino_FreeRTOS.h"
#include "queue.h"
#include "DHT.h"
#include "semphr.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0X27,16,2); 



/* Define */
#define DHTTYPE DHT11
#define DHTPIN 41
#define LED1   45
#define LED2   47
#define PIR    33
#define BUZZER 43

/* DHT11 */
DHT dht(DHTPIN, DHTTYPE);

float Hum = 0;
float Temp = 0;
volatile int check_person = 0;

/*  Task control */
String StateLed1 = "0";
String StateLed2 = "0";

/* Module Sim */
int _timeout;
String _buffer;
String number = "0978879560"; 

/* The task function. */
static void vTaskControl( void *pvParameters );
static void vTaskDHT( void *pvParameters );
static void vTaskLCD(void *pvParameters);
static void vPIRInterruptHandler( void );
static void vMQ2InterruptHandler( void );
static void vTaskReceiveFromESP8266(void *pvParameters);
static void vTaskSendToESP8266(void *pvParameters);
static void SendMessage(int);
static void CallNumber(void);
static void updateSerial(void);


/* Handle */
TaskHandle_t HandleDHT;
TaskHandle_t HandleLCD;
TaskHandle_t HandleReceiveFromESP8266;
TaskHandle_t HandleSendToESP8266;
TaskHandle_t HandleControl;
TaskHandle_t HandleWarning;


/* Queue */
QueueHandle_t xQueueControl;
QueueHandle_t xQueueDHT;
QueueHandle_t xIntegerQueue;

/* semaphore */
SemaphoreHandle_t xCountingSemaphore;


typedef struct{
  int  Led1;
  int  Led2;
}StructControl;

typedef struct{
  int  Hum;
  int Temp;
}StructDHT;



void setup() {

   /* join i2c bus with address 8 */
  Serial.begin(9600);
  Serial1.begin(9600);
  dht.begin();
  /* Module Sim */
  /* Signal Pin */
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  Serial.print("ATE0\r\n");

  /* LCD */
  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("start"); 


  /* FreeRTOS */
  xCountingSemaphore = xSemaphoreCreateCounting(5, 0);
  
  xIntegerQueue = xQueueCreate( 5, sizeof( unsigned int));
  xQueueControl = xQueueCreate( 10, sizeof( StructControl));
  xQueueDHT     = xQueueCreate( 10, sizeof( StructDHT));
  // config interrupt
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), vPIRInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(2), vMQ2InterruptHandler, FALLING);
   
  if( xCountingSemaphore != NULL )
  {
    xTaskCreate( vTaskWarning, "vTaskWarning", 400, NULL, 5,&HandleWarning);  
  }
  
  xTaskCreate( vTaskSendToESP8266, "vTaskSendToESP8266", 300, NULL, 4,&HandleSendToESP8266); 
   
  if ( xQueueControl != NULL)
  {
    xTaskCreate( vTaskReceiveFromESP8266, "vTaskReceiveFromESP8266", 500, NULL , 3,&HandleReceiveFromESP8266);
    xTaskCreate( vTaskControl, "Task Control", 300, NULL, 4,&HandleControl                                  );
  }
  
  if( xQueueDHT  != NULL )
  {
    xTaskCreate( vTaskDHT, "Task DHT",500, NULL, 2,&HandleDHT );
    xTaskCreate( vTaskLCD, "Task LCD", 500, NULL, 1,&HandleLCD );
  }

  
  /* Start the scheduler so our tasks start executing. */
  vTaskStartScheduler();
  for(;;);
}


static void vTaskDHT(void *pvParameters)
{

  long SendTemp;
  long SendHum;
  
  for(;;)
  {
//   Serial.println("---DHT---");
   Hum = dht.readHumidity();
   Temp = dht.readTemperature();
   
   if(isnan(Hum) || isnan(Temp))
   {
      Temp = 0;
      Hum = 0;
   }

   StructDHT SendDHT;
   SendDHT.Hum = int(Hum);
   SendDHT.Temp= int(Temp);
   

   xQueueSendToFront( xQueueDHT ,&SendDHT, 0 );
 
   vTaskDelay(500/portTICK_PERIOD_MS );
   /* Allow the other sender task to execute. */
   taskYIELD();

  }
}

static void vTaskLCD(void *pvParameters)
{
  
  StructDHT ReceiveDHT;
  
  const TickType_t xTicksToWait = 100/portTICK_PERIOD_MS;
  for(;;)
  {
//    Serial.println("---LCD---");  
    xQueueReceive(xQueueDHT, &ReceiveDHT, xTicksToWait);
    
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("hum:");
    lcd.print(ReceiveDHT.Hum);
    lcd.print("%");
    lcd.setCursor(0,1);
    lcd.print("temp:");
    lcd.print(ReceiveDHT.Temp);
    lcd.print(char(223));
    lcd.print("C");

    vTaskDelay(500/portTICK_PERIOD_MS );
  }
}

static void vTaskReceiveFromESP8266(void *pvParameters)
{ const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  BaseType_t xStatus;
  for(;;)
  { 
    if (Serial1.available() >0)
    {
    
    String str;

    str =  Serial1.readStringUntil('\n');
    StateLed1 = str.substring(0,1);
    StateLed2 = str.substring(2,3);
    StructControl DataToSend;
    DataToSend.Led1 = StateLed1.toInt();
    DataToSend.Led2 = StateLed2.toInt();
    
    xQueueSendToFront( xQueueControl,&DataToSend, xTicksToWait);

    }
    vTaskDelay(500/portTICK_PERIOD_MS );
    taskYIELD();
  }
}

static void vTaskSendToESP8266(void *pvParameters)
{
  StructDHT DHTValue;
  String DataSend;
  
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  for(;;)
  {
    
    DataSend = "";
    xQueueReceive( xQueueDHT, &DHTValue, xTicksToWait );
 
  
    
    DataSend = DataSend + DHTValue.Hum+":"+DHTValue.Temp;
    Serial1.println(DataSend);

    vTaskDelay(300/portTICK_PERIOD_MS );
  }
}

static void vTaskControl(void *pvParameters)
{ BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  StructControl queue_control;
  
  for(;;)
  {
//    Serial.println("---Control---");
    xStatus = xQueueReceive( xQueueControl, &queue_control,xTicksToWait );
    if(xStatus == pdPASS){
      

      if (queue_control.Led1 == 1) digitalWrite(LED1, HIGH);
      else digitalWrite(LED1, LOW);

      if (queue_control.Led2 == 1) digitalWrite(LED2, HIGH);
      else digitalWrite(LED2, LOW);
            
      }
    
    vTaskDelay(300/portTICK_PERIOD_MS );
  }
}


static void vTaskWarning(void *pvParameters)
{ 
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  for(;;)
  { 
    int state = 0;
    String Content;
    for(;;)
    {
      state = 0;
      xSemaphoreTake(xCountingSemaphore, portMAX_DELAY );
      xStatus = xQueueReceive(xIntegerQueue, &state, xTicksToWait);
      if (xStatus == pdPASS)
      {
        digitalWrite(BUZZER, HIGH); 
        CallNumber();
        SendMessage(state);
  
        //delay(2000);
        digitalWrite(BUZZER, LOW);
        if (state == 1)
        {
          attachInterrupt(digitalPinToInterrupt(3), vPIRInterruptHandler, RISING);
        }
        if (state == 2)
        {
          attachInterrupt(digitalPinToInterrupt(2), vMQ2InterruptHandler, FALLING);
        }
       }
    }
  }
}

static void vPIRInterruptHandler( void )
{
  detachInterrupt(digitalPinToInterrupt(3));
  volatile int state2 = 1;
  static BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xCountingSemaphore, (BaseType_t*)&xHigherPriorityTaskWoken);
  xQueueSendToBackFromISR(xIntegerQueue, &state2, (BaseType_t*)&xHigherPriorityTaskWoken);
  if( xHigherPriorityTaskWoken == pdTRUE )
  {

    vPortYield();
  }
}

static void vMQ2InterruptHandler( void )
{
  detachInterrupt(digitalPinToInterrupt(2));
  volatile int state1 = 2;
  static BaseType_t xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( xCountingSemaphore, (BaseType_t*)&xHigherPriorityTaskWoken );
  xQueueSendToBackFromISR( xIntegerQueue, &state1, (BaseType_t*)&xHigherPriorityTaskWoken);
  if( xHigherPriorityTaskWoken == pdTRUE )
  {
    vPortYield();
  }
}



///* functions for module sim 800l */
static void SendMessage(int x)
{
  String message = "";
  if (x == 1)
  {
    message = "Co Trom Dot Nhap!";
  }
  if (x == 2)
  {
    message = "Co khi de chay!";
  }
  Serial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  Serial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  Serial.println("AT+CMGS=\"" + number + "\"\r"); //Mobile phone number to send message
  updateSerial();
  Serial.println(message); //text content
  updateSerial();
  Serial.write(26);
}

static void CallNumber(void)
{
  Serial.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();
  Serial.print("ATD0978879560;\r\n");
  delay(10000); 
  Serial.println("ATH"); //hang up


}

static void updateSerial(void)
{
  delay(500);

  while(Serial.available()) 
  {
    Serial.write(Serial.read());//Forward what Software Serial received to Serial Port
  }
}

 
void loop()
{
}
