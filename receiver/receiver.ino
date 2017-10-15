// Incluímos la librería para poder controlar el servo
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int Input_A = 5;
int Input_B = 6; 

int arrow_control_0 = 2;
int arrow_control_1 = 3;


// Declaramos la variable para controlar el servo
Servo servoMotor_left;
Servo servoMotor_right;

RF24 radio(10, 9); // CE, CSN
const byte address[6] = "00001";

void setup() {
  // Iniciamos el monitor serie para mostrar el resultado
  Serial.begin(9600);
 
  // Iniciamos el servo para que empiece a trabajar con el pin 9
  servoMotor_left.attach(5);
  servoMotor_right.attach(6);

  pinMode(arrow_control_0,OUTPUT);
  pinMode(arrow_control_1,OUTPUT);
  
  digitalWrite(arrow_control_0,LOW);
  digitalWrite(arrow_control_1,LOW);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  Serial.println("Hello");
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
}

void servo_forward (){
  servoMotor_left.write(180);
  servoMotor_right.write(0);
  digitalWrite(arrow_control_0,HIGH); //pin 0 GEMMA
  digitalWrite(arrow_control_1,LOW);  //pin 2 
}

void servo_reverse (){
  servoMotor_left.write(0);
  servoMotor_right.write(180);
  digitalWrite(arrow_control_0,LOW);
  digitalWrite(arrow_control_1,LOW);
}

void servo_left (){
  servoMotor_left.write(180);
  servoMotor_right.write(180);
  digitalWrite(arrow_control_0,HIGH);
  digitalWrite(arrow_control_1,HIGH);
}

void servo_idle (){
  servoMotor_left.write(90);
  servoMotor_right.write(90);
}

void servo_right (){
  servoMotor_left.write(0);
  servoMotor_right.write(0);
  digitalWrite(arrow_control_0,LOW);
  digitalWrite(arrow_control_1,HIGH);
}

void loop() {

  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);


      if (strcmp(text,"forward")==0){
        servo_forward();
      }

      if (strcmp(text,"backward")==0){
        servo_reverse();
      }

      if (strcmp(text,"left")==0){
        servo_left();
      }

      if (strcmp(text,"right")==0){
        servo_right();
      }
      
      if (strcmp(text,"idle")==0){
        servo_idle();
      }
    }
  
  
}

