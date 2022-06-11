/* SMART HOME USING FreeRTOS 
GROUP: TRAN  CHI CUONG
Members: LE VAN THIEN
         DAO DUY NGU
         TRAN CHI CUONG
         HO THANH LONG
         NGUYEN VU HOAI DUY
*/


/*--------------- include library--------------*/


#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0X27,16,2); 
#include "Arduino_FreeRTOS.h"
#include "queue.h"
#include "DHT.h"
/*--------------- Define Pin--------------*/
#define DHTTYPE DHT11
#define DHTPIN 41
#define LED1   45
#define LED2   47
#define PIR    33
#define BUZZER 43

/*--------------- DHT-11 --------------*/
DHT dht(DHTPIN, DHTTYPE);


/*--------------- Define variable --------------*/
volatile int check_person = 0;
float Hum = 0;                  
float Temp = 0;

String StateLed1 = "0";
String StateLed2 = "0";

/*--------------- Setup phone number --------------*/
String number = "0978879560"; 

/*--------------- declare vTask function --------------*/
void vTaskReceiveFromESP8266(void *pvParameters);
void vTaskSendToESP8266(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskWarning(void *pvParameters);
void vTaskDHT( void *pvParameters );
void vTaskLCD(void *pvParameters);


/*--------------- Handle--------------*/
TaskHandle_t HandleReceiveFromESP8266;
TaskHandle_t HandleSendToESP8266;
TaskHandle_t HandleControl;
TaskHandle_t HandleWarning;
TaskHandle_t HandleDHT;
TaskHandle_t HandleLCD;

/*--------------- Queue--------------*/
QueueHandle_t xQueueControl;
QueueHandle_t xQueueDHT;

/*---------------Struct--------------*/
typedef struct{
  int  Led1;
  int  Led2;
}StructControl;

typedef struct{
  int  Hum;
  int Temp;
}StructDHT;



void setup() {
  //Begin serial communication with Arduino and SIM800L
  Serial.begin(9600);
  Serial1.begin(9600);
  dht.begin();
  /* Module Sim */

  /* Signal Pin */
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  /* LCD */
  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("start"); 


  /* FreeRTOS */
  xQueueControl = xQueueCreate( 10, sizeof( StructControl) );
  xQueueDHT     = xQueueCreate( 10, sizeof( StructDHT));
  // config interrupt
   pinMode(3, INPUT);
   attachInterrupt(digitalPinToInterrupt(3), turn_on_buzzer, RISING);
   
   
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

xTaskCreate( vTaskWarning, "vTaskWarning", 300, NULL, 5,&HandleWarning);   
  /* Start the scheduler so our tasks start executing. */
  vTaskStartScheduler();
}


void vTaskDHT(void *pvParameters)
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

void vTaskLCD(void *pvParameters)
{
  
  StructDHT ReceiveDHT;
  
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  for(;;)
  {
//    Serial.println("---LCD---");  
    xQueueReceive( xQueueDHT, &ReceiveDHT, xTicksToWait );
    
    
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

void vTaskReceiveFromESP8266(void *pvParameters)
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
    
    xQueueSendToFront( xQueueControl,&DataToSend, xTicksToWait );

    }
    vTaskDelay(500/portTICK_PERIOD_MS );
    taskYIELD();
  }
}

void vTaskSendToESP8266(void *pvParameters)
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

void vTaskControl(void *pvParameters)
{ BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  StructControl queue_control;
  
  for(;;)
  {
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


void vTaskWarning(void *pvParameters)
{ 
 
   

  for(;;)
  { 
    int x = 1;
    int GasValue = analogRead(A0);   //đọc giá trị điện áp ở chân A0 - chân cảm biến (value luôn nằm trong khoảng 0-1023)
    int per = map(GasValue,0,1023,0,100);
    if (check_person == 1)
        x = 2;
    if (per > 20 || check_person == 1) //20
    { 
      ;
      digitalWrite(BUZZER, HIGH);       

    CallNumber();
    delay(5000);
      SendMessage(x);
      
      digitalWrite(BUZZER, LOW);
      check_person = 0;
      break;
    }
    vTaskDelay(1000/portTICK_PERIOD_MS );
  }
}

void turn_on_buzzer()
{
  digitalWrite(BUZZER, HIGH);
  check_person = 1;
}



///* functions for module sim 800l */
void CallNumber(void)
{
  Serial.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();

  
  Serial.print(F("ATD"));
  Serial.print(number);
  Serial.print(F(";\r\n"));
  updateSerial();

}
void SendMessage(int x)
{
  String Content;
  if (x == 1)
  {
    Content =" Co Khi De Chay!";
  }
  else
  {
    Content = "Co Trom Dot Nhap!";
  }
  
   Serial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  Serial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();

  Serial.println("AT+CMGS=\"" + number + "\"\r");
  updateSerial();
  Serial.print(Content); 
  updateSerial();
  Serial.write(26);
}
void updateSerial()
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
