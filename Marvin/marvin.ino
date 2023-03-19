#include "Nidec.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v2.h>

#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "LAPEC"
#define WIFI_PASSWORD "l@pec2022"
// Insert Firebase project API Key
#define API_KEY "AIzaSyA9qv8OdpvhWh3hByQ-ZLVLI5BtRqah98k"
// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://marvin-pid-default-rtdb.firebaseio.com" 

#define LED_PIN 2

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

MPU6050 mpu6050(Wire);

Nidec motor1(19,22,21);
Nidec motor2(14,25,33);

double Kp = 7, Ki = 3.6, Kd = 0.1;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
int output = 0;
double setpoint = 0;

void initializeMPU(){
  // Try to initialize!
  Wire.begin(4,15);
  mpu6050.begin();
  mpu6050.calcGyroOffsets();
}

void turnOffWalking(double output) {
  float new_angle = mpu6050.getAngleX();
  // Serial.println(new_angle);
  if (new_angle > 40 || new_angle < -40) {
    motor1.setForce(0);
    motor2.setForce(0);
  } else {
    motor1.setForce(output);
    motor2.setForce(output);
  }
  // delay(100);
}

void setup() {
  motor2.setInverse(true);
  initializeMPU();
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(10);
  myPID.Start(mpu6050.getAngleX(),output,setpoint);
  pinMode(LED_PIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.begin(115200);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  } 

  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);  
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  const double input = mpu6050.getAngleX();
  const double output = myPID.Run(input);
  motor1.setForce(output);
  motor2.setForce(output);
  turnOffWalking(output);

  /*
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    digitalWrite(2, HIGH);
    // if(Firebase.RTDB.getFloat(&fbdo, F("/PID/Kp"), &Kp) || Firebase.RTDB.getFloat(&fbdo, F("/PID/Ki"), &Ki) || Firebase.RTDB.getFloat(&fbdo, F("/PID/Kd"), &Kd)){ 
    //   Serial.println(setpoint);
    //   Firebase.RTDB.setDouble(&fbdo, F("/PID atual/Kp"), Kp);
    //   Firebase.RTDB.setDouble(&fbdo, F("/PID atual/Ki"), Ki);
    //   Firebase.RTDB.setDouble(&fbdo, F("/PID atual/Kd"), Kd);
    // }

     if(Firebase.RTDB.getFloat(&fbdo, F("/PID/Kp"), &Kp)){
      Serial.print("Kp: ");
      Serial.println(Kp);
      Firebase.RTDB.setFloat(&fbdo, F("/PID atual/Kp"), Kp);   
    }   
    if(Firebase.RTDB.getFloat(&fbdo, F("/PID/Ki"), &Ki)){
      Serial.print("Ki: ");
      Serial.println(Ki);
      Firebase.RTDB.setDouble(&fbdo, F("/PID atual/Ki"), Ki);    
    }  
    if(Firebase.RTDB.getFloat(&fbdo, F("/PID/Kd"), &Kd)){
      Serial.print("Kd: ");
      Serial.println(Kd);
      Firebase.RTDB.setDouble(&fbdo, F("/PID atual/Kd"), Kd);    
    }  
    if(Firebase.RTDB.getFloat(&fbdo, F("/PID/setpoint"), &setpoint)){
      Serial.print("Setpoint: ");
      Serial.println(setpoint);
      Firebase.RTDB.setDouble(&fbdo, F("/PID atual/setpoint"), setpoint);    
    }  
  } */
}

void testeNidec(){
  Nidec motort(19,22,21);
  int i=-255;
  for(i=-255;i<=255;i++){
    motort.setForce(i);
    delay(200);
  }
  for(i=255;i>=-255;i--){
    motort.setForce(i);
    delay(200);
  }
}
