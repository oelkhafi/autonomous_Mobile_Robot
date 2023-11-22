#include <Ultrasonic.h>
#include <Servo.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "23F_PubSubClient.h"
#include "23F_WiFiManager.h" 
#include "SSD1306.h"

//MQTT Communication associated variables
char payload_global[100];                     
boolean flag_payload;                         

//MQTT Setting variables  
const char* mqtt_server= "192.168.0.218";            //MQTT Broker(Server) Address
const char* MQusername = "user";              //MQTT username
const char* MQpassword = "Stevens1870";       //MQTT password
const char* MQtopic    = "louis_lidar_new";   //MQTT Topic (Arena I/II)
const int mqtt_port    = 1883   ;             //MQTT TCP/IP port number 

//WiFi Define
WiFiClient espClient;                         
PubSubClient client(espClient);      

//WiFi Setting variables
const char* ssid     = "TP-Link_8FFD";        //Wi-Fi SSID (Service Set IDentifier)   
const char* password = "68287078";            //Wi-Fi Password

/////////////////////////////////////////////////////////////////////////////robot variables///////////////////////////////////////////////////////////////////
//define pins for the motors
#define motor1pin D0 //GPIO pin setting for motor1
#define motor2pin D2 // GPIO pin setting for motor2
Servo motor1; //create servo object for right motor
Servo motor2; //create servo object for left motor

//define pins for the Ultrasonic sensors
Ultrasonic ultrasonic_one(D8, D5); //ultrasonic front sensor (trig, echo)
Ultrasonic ultrasonic_two(D9, D6); //ultrasonic right sensor
Ultrasonic ultrasonic_three(D10, D7); //ultrasonic left sensor

//define pins for the OLED display
SSD1306 display(0x3C, D14, D15); 

//define and initialize logic variables
int left_reading = 0;
int front_reading = 0;
int right_reading = 0;

boolean obstacleDetection = false;
boolean withinTarget = false;

float x = 0.0;
float y = 0.0;
double previous_x = 0; //previous x coordinate
double previous_y = 0; //previous y coordinate

double angle = 1.0;
double cp = 1.0;
double turnTime = 0.0;
double targetDistance = 0.0;
int targetNum = 0; 
//    Index V:     0,   1,   2,   3,   4,   5,    6,    7,   8,   9,   10   11    12   13  14
int target_x[] = {700, 180, 180, 180, 750, 2200, 1500, 750, 700, 700, 250,  250, 250, 670, 670}; //Index 2, Index 5, Index 8, Index 11, 14  are targets
int target_y[] = {400, 400, 140, 400, 700, 700,  550,  600, 750, 400, 400,  800, 400, 400, 130}; //Indices for int: 0, 1, 3, 4, 6, 7, 9, 10, 12, 13

/////////////////////////////////////////////////////////////////////////////WiFi and MQTT setup DONT TOUCH///////////////////////////////////////////////////////////////////
//WiFi setup
void setup_wifi() { 
  delay(10);
  // We start by connecting to a Stevens WiFi network
  WiFi.begin(ssid, password);           
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");                        
  }
  randomSeed(micros());                       
}

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload_global[i] = (char)payload[i];
  }
  payload_global[length] = '\0';              
  flag_payload = true;                        
}

//MQTT reconnection
void reconnect() {                                                                
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";       
    clientId += String(random(0xffff), HEX);  
    // Attempt to connect                     
    if (client.connect(clientId.c_str(),MQusername,MQpassword)) {
      client.subscribe(MQtopic);             
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////SETUP loop DONT TOUCH///////////////////////////////////////////////////////////////////
void setup() {  
  //wifi Setup
  Serial.begin(115200);
  setup_wifi();                               
  Serial.println("Wemos POWERING UP ......... ");
  client.setServer(mqtt_server, mqtt_port);         //This 1883 is a TCP/IP port number for MQTT 
  client.setCallback(callback); 

  //motor Setup
  motor1.attach(motor1pin); //motor1 is attached using the motor1pin
  motor2.attach(motor2pin); //motor2 is attached using the motor2pin

  //OLED display setup
  display.init();
  display.flipScreenVertically();
  display.drawString(0, 0, "Group G2");
  display.display();

  previous_x = 2200;
  previous_y = 600;
  drive_forward();
  delay(3200);
  drive_stop();
  delay(1000);
}

/////////////////////////////////////////////////////////////////////////////MAIN loop, BEGIN robot logic///////////////////////////////////////////////////////////////////
void loop() {
  drive_stop();
  delay(200);
  getLocation();
  delay(200);
  
  dotProduct();
  crossProduct();
  if(angle > -10000.0 && angle < 10000.0){ // NAN angle workaround
  turnTime = (43/9)*(angle);
  }
  displayData();

//turning logic
if((x < 1 && y < 1)&&(x != previous_x && y != previous_y)){
 }
 else if(cp <= 0){
  turn_right();
  delay(turnTime);
  previous_x = x;
  previous_y = y;
  obstacleDetection = true;
 }
 else{
  turn_left();
  delay(turnTime);
  previous_x = x;
  previous_y = y;
  obstacleDetection = true;
 }

while(obstacleDetection == true){
  drive_forward();

  left_reading = ultrasonic_three.read(CM) * 10;
  delay(10);
  front_reading = ultrasonic_one.read(CM) * 10;
  delay(10);
  right_reading = ultrasonic_two.read(CM) * 10; //convert readings to mm so units are the same throughout code
  delay(10);
  
  getLocation();
  dotProduct();
  crossProduct();
  displayData();
  
//target finding logic  
  targetDistance = sqrt(sq(target_y[targetNum] - y) + sq(target_x[targetNum] - x));
  if(targetDistance <= 120){
    withinTarget = true;
  }
  if(targetDistance <= 100){
      if(targetNum == 0 || targetNum == 1 || targetNum == 3 || targetNum == 4 || targetNum == 6 || targetNum == 7 || targetNum == 9 || targetNum == 10 || targetNum == 12 || targetNum == 13){
      targetNum++; 
      drive_stop();
      delay(250);
      obstacleDetection = false;
      withinTarget = false;
    }
     else if(targetDistance <= 60){
    drive_stop();

    targetNum++;
    display.clear();
    display.drawString(0, 0, "Target Reached. We go next.");
    display.display();
    delay(2000);
    display.clear();
    obstacleDetection = false;
    withinTarget = false;
  }
  }
  
//obstacle avoidance logic  
  else if(left_reading < 60){
    turn_right();
    delay(100);
    previous_x = x;
    previous_y = y;
  }
  else if(right_reading < 60){
    turn_left();
    delay(100);
    previous_x = x;
    previous_y = y;
  }
  else if(front_reading < 50){
    if(cp <= 0){
      turn_right();
      delay(200);
      previous_x = x;
      previous_y = y;
    }
    else{
      turn_left();
      delay(200);
      previous_x = x;
      previous_y = y;
    }
  }
  else if((x > previous_x + 100 || x < previous_x - 100) || (y > previous_y + 100 || y < previous_y - 100)){
  obstacleDetection = false;
  }
 }
}
/////////////////////////////////////////////////////////////////////////////END robot logic///////////////////////////////////////////////////////////////////
//robot subroutines
void drive_forward(){
 if(withinTarget == true){
  motor1.write(75);
  motor2.write(75);
 }
 else{
 motor1.write(70);
 motor2.write(70);
 }
}

void turn_right(){
 motor1.write(120);
 motor2.write(60);
}

void turn_left(){
 motor1.write(60);
 motor2.write(120);
} 

void drive_reverse(){
 motor1.write(105);
 motor2.write(105);
} 

void drive_stop(){
 motor1.write(90);
 motor2.write(90);
}

void dotProduct(){
  double vectorDrive = (((x - previous_x) * (x - previous_x)) + ((y - previous_y) * (y - previous_y)));
  double vectorTarget = (((target_x[targetNum] - x) * (target_x[targetNum] - x)) + ((target_y[targetNum] - y) * (target_y[targetNum] - y)));
  double dotProduct = (((x - previous_x) * (target_x[targetNum] - x)) + ((y - previous_y) * (target_y[targetNum] - y)));

  if(vectorDrive != 0 && vectorTarget != 0){
    angle = acos((dotProduct) / (sqrt(vectorDrive) * sqrt(vectorTarget)));
    angle = ((angle*180) / (M_PI)); //convert radians to degrees
  }
}

void crossProduct(){
  cp = (((x - previous_x) * (target_y[targetNum] - y) - (y - previous_y) * (target_x[targetNum] - x)));
}

void getLocation(){
  //subscribe the data from MQTT server
  if (!client.connected()) {
    Serial.print("...");
    reconnect();
  }
  client.loop();                              
  
  String payload(payload_global);              
  int testCollector[10];                      
  int count = 0;
  int prevIndex, delimIndex;
    
  prevIndex = payload.indexOf('[');           
  while( (delimIndex = payload.indexOf(',', prevIndex +1) ) != -1){
    testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
    prevIndex = delimIndex;
  }
  delimIndex = payload.indexOf(']');
  testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
   
  //Robot location x,y from MQTT subscription variable testCollector 
  x = testCollector[0];
  y = testCollector[1];
}

void displayData(){
  display.clear(); //clears the OLED display
  String string_current = "x: " + (String)x + " y " + (String)y;
  String string_angle = "Angle: " + (String)angle;
  String string_cp = "CP: " + (String)cp;
  String string_previous = "Prev x: " + (String)previous_x + " Prev y: " + (String)previous_y; 
  display.drawString(0, 0, string_current);
  display.drawString(0, 15, string_angle);
  display.drawString(0, 30, string_cp);
  display.drawString(0, 45, string_previous);
  display.display();
}




    
  
  
