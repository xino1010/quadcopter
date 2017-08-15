#include<Servo.h>
 
Servo ESC1; //Crear un objeto de clase servo
Servo ESC2; //Crear un objeto de clase servo
Servo ESC3; //Crear un objeto de clase servo
Servo ESC4; //Crear un objeto de clase servo
 
int vel = 900;

void setup() {
  Serial.begin(9600);
  //Iniciar puerto serial
  delay(1000);
  Serial.println("Serial inicializado");
  
  //Asignar un pin al ESC
  ESC1.attach(3);
  ESC2.attach(5);
  ESC3.attach(6);
  ESC4.attach(9);

  delay(5000);

  ESC1.writeMicroseconds(vel);
  ESC2.writeMicroseconds(vel);
  ESC3.writeMicroseconds(vel);
  ESC4.writeMicroseconds(vel);

  delay(1000);
  Serial.println("Pausa hecha");
}
 
 
void loop() {
  if (Serial.available() >= 1) {
    vel = Serial.parseInt(); //Leer un entero por serial
    Serial.print("Milliseconds: ");
    Serial.println(vel);
    if (vel >= 0 && vel <= 1300) {
      Serial.println("Cambiando velocidad...");
      ESC1.writeMicroseconds(vel); //Generar un pulso con el numero recibido
      ESC2.writeMicroseconds(vel); //Generar un pulso con el numero recibido
      ESC3.writeMicroseconds(vel); //Generar un pulso con el numero recibido
      ESC4.writeMicroseconds(vel); //Generar un pulso con el numero recibido
      Serial.println("Velocidad cambiada...");
    }
  }
  delay(50);
}
