#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

WiFiClient client;
char auth[] = "LnhaetDzOIH5e2AdEUo4_euVC8k318CF"; 
const char *ssid =  "B-LINK_29F5";     // replace with your wifi ssid and wpa2 key
const char *pass =  "Thien3920";

int state_led0,state_led1;
String xdata ;

void setup() {
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  delay(10);
  WiFi.begin(ssid, pass);
}

// button blynk
BLYNK_WRITE(V0)                     //  ham nay duoc goi den khi Widget Vo thay doi trang thai
{
   int pinValue1 = param.asInt();       // gan gia tri cua chan V0 cho bien pinValue
   
   if (pinValue1 == 1) {
   state_led0=1;        
   }
   
   else {
   state_led0=0;        
   }
}


BLYNK_WRITE(V1)                  
{
   int pinValue = param.asInt();      
   
   if (pinValue == 1) {
   state_led1=1;        
   }
   
   else {
   state_led1=0;      
   }
}


void GetDataFromArduino()
{
  String Value;
  String HumValue;
  String TempValue;

  if (Serial.available() >0)
  {
  Serial.setTimeout(100);
  Value = Serial.readString();
  
  int idx = Value.indexOf(":");
  int len = Value.length();
  HumValue = Value.substring(0,idx);
  TempValue = Value.substring(idx+1,len);
 
  Blynk.virtualWrite(V3,HumValue.toFloat());
  Blynk.virtualWrite(V4,TempValue.toFloat());
  } 
}

void SendDataToArduino(String data)
{
  Serial.println(data);
}

void loop() {

  Blynk.run(); 
  GetDataFromArduino();

  xdata ="";
  xdata=xdata+state_led0+":"+state_led1;
  SendDataToArduino(xdata); 
    
  delay(1000);
}
