#include <Arduino.h>
// Seguidor de linea negra con sensor TCRT5000


int VelocidadMotor1 = 5; //ENA  5
int VelocidadMotor2 = 9; //ENB  9
//int Motor1A = 6; //IN2
//int Motor1B = 7;  //IN1
//int Motor2C = 8; //IN4
//int Motor2D = 10; //IN3

// *** MOTORES ***

int izqA = 7;    //// pines 5,6,7 y 8 al puente H
int izqB = 8;
int derA = 5;
int derB = 6;
int vel = 130;    // Velocidad de los motores se puede variar (0-255)
int vel2 = 200;
int estado = '1'; // inicia en espera


int infraPin = 2;    // izquierdo - pin del infrarrojos utilizado como entrada digital
int infraPin1=4;  //  derecho
int valorInfra = 0;  // Valor inicial de la lectura digital del infrarrojos izquierdo 
int valorInfra1 = 0;  // derecho

const int ECHO=3;                                          // Pin para recibir el pulso de eco
const int TRIGGER=8;                                       // Pin para enviar el pulso de disparo
//int duracion, distancia;  // para Calcular distancia

const int cerca=7;                                          // Distancia a la cual esquiva el obstaculo. maximo 15 cm
const int giro=500;
const int pasos=600;
const int duraccionMaxPulso=1000;                           // Tiempo maximo de espera en microsegundos
volatile unsigned distancia;

void setup() {
  delay(5000);
   pinMode(infraPin, INPUT);    
   pinMode(infraPin1, INPUT); 
   pinMode(izqA,OUTPUT);
   pinMode(izqB,OUTPUT);
   pinMode(derA,OUTPUT);
   pinMode(derB,OUTPUT);
   pinMode(VelocidadMotor1, OUTPUT);
   pinMode(VelocidadMotor2, OUTPUT);

   pinMode(ECHO, INPUT);                                     // Establece ECHO como pin de entrada digital
   pinMode(TRIGGER, OUTPUT);                                 // Establece TRIGGER como pin de salida digital
   
   analogWrite(VelocidadMotor1, 120); //motor izquierdo
   analogWrite(VelocidadMotor2, 120);  //motor derecho
       
   analogWrite(izqA, LOW);
   analogWrite(izqB, LOW);
   analogWrite(derA, LOW);
   analogWrite(derB, LOW);

}

void loop() {
  valorInfra = digitalRead(infraPin);    // valor de la entrada que lee el infrarrojo izquierdo
  valorInfra1 = digitalRead(infraPin1);  // derecho

  medir ();
  
  if (distancia<cerca)
  {
    esquiva();
  }
  else  
  {
        // nada
  }
             
          

   // 0 = blanco / 1 = negro
  
  if(valorInfra == 0 && valorInfra1 == 0){ // Hacia delante
     
  //digitalWrite(Motor1A, HIGH);
      //digitalWrite(Motor2D, HIGH);
      //delay(20);                      // Tiempo para control de velocidad
      //digitalWrite(Motor1A, LOW);
      //digitalWrite(Motor2D,LOW);
       //delay(20);                     // Tiempo para control de velocidad

    analogWrite(derA, vel2); 
    analogWrite(izqA, vel);
    delay(2);
    analogWrite(derB, 0);   
    analogWrite(izqB, 0);
    delay(2);

}

 if(valorInfra == 0 && valorInfra1 == 1){  // Giro derecha
  
   // digitalWrite(Motor1A, LOW);
    //digitalWrite(Motor2D,LOW);
    //delay(25);
    //digitalWrite(Motor1A, HIGH);
    //digitalWrite(Motor2D,LOW);
    //delay(20);

    analogWrite(derB, 0);  //vel2 
    analogWrite(izqB, 0);
    delay(2);
    analogWrite(izqA, 0);
    analogWrite(derA, vel2); 
    delay(2);
     
}

 if(valorInfra == 1 && valorInfra1 == 0){ // Giro izquierda

 // digitalWrite(Motor1A,LOW);
  //digitalWrite(Motor2D, LOW);
  //delay(25);
  //digitalWrite(Motor1A,LOW);
  //digitalWrite(Motor2D, HIGH);
  //delay(20);

    analogWrite(derB, 0);  //vel2 
    analogWrite(izqB, 0);
    delay(2);
    analogWrite(derA, 0); 
    analogWrite(izqA, vel);  
    delay(2);  
}

 if(valorInfra == 1 && valorInfra1 == 1){  // STOP
     
      //digitalWrite(Motor1A, LOW);
      //digitalWrite(Motor1B, LOW);
      //digitalWrite(Motor2C, LOW);
      //digitalWrite(Motor2D, LOW);

    analogWrite(derB, 0);   
    analogWrite(izqB, 0);
    analogWrite(derA, 0);   
    analogWrite(izqA, 0);
      
}
}
void medir ()
{
  analogWrite(TRIGGER, LOW);                                  // Triger en bajo
  delayMicroseconds(2);                                        // Esperamos 2 milisegundos
  analogWrite(TRIGGER, HIGH);                                 // Triger en alto 
  delayMicroseconds(10);                                       // Esperamos 10 milisegundos
  analogWrite(TRIGGER, LOW);                                  // Triger en bajo
  distancia = (pulseIn(ECHO, HIGH,duraccionMaxPulso))/29.15/2; // Medimos la distancia basandonos en que el tiempo que 
  if ( distancia == 0 ) {distancia = duraccionMaxPulso;}       // tarda el sonido en recorrer 1 cm es de 29 microsegundos
}

void esquiva()
{
   derecha();
   delay(giro);
   parar();
   delay(2000); 
   avanza();
   delay(pasos); 
   izquierda();
   delay(giro);
   parar();
   delay(2000); 
   avanza();
   delay(2*pasos); 1
   derecha();
   delay(giro); 
   parar();
   delay(2000);
   avanza();
}

// Para entender las rutinas, hay que tener en cuentra, que los servos se encuentran enfrentados

void derecha()
{
    analogWrite(derB, 0);  //vel2 
    analogWrite(izqB, 0);
    delay(2);
    analogWrite(izqA, 0);
    analogWrite(derA, vel2); 
    delay(2);
    
      
 // miservo1.write(centroServo+velocidad);             // Posiciona el servo1 en la posicion adelante 
 // miservo2.write(centroServo+velocidad);             // Posiciona el servo2 en la posicion adelante 
}

void izquierda()
{
    analogWrite(derB, 0);  //vel2 
    analogWrite(izqB, 0);
    delay(2);
    analogWrite(derA, 0); 
    analogWrite(izqA, vel);  
    delay(2);
 // miservo1.write(centroServo-velocidad);             // Posiciona el servo1 en la posicion atras 
 //  miservo2.write(centroServo-velocidad);             // Posiciona el servo2 en la posicion atras 
}

void avanza()
{
    analogWrite(derA, vel2); 
    analogWrite(izqA, vel);
    delay(2);
    analogWrite(derB, 0);   
    analogWrite(izqB, 0);
    delay(2);
 // miservo1.write(centroServo-velocidad);            // Posiciona el servo1 en la posicion atras 
 // miservo2.write(centroServo+velocidad);            // Posiciona el servo2 en la posicion adelante 
}
void parar()
{
    analogWrite(derA, 0); 
    analogWrite(izqA, 0);
    delay(2);
    analogWrite(derB, 0);   
    analogWrite(izqB, 0);
    delay(2);
 // miservo1.write(centroServo-velocidad);            // Posiciona el servo1 en la posicion atras 
 // miservo2.write(centroServo+velocidad);            // Posiciona el servo2 en la posicion adelante 
}

//LO QUE ESTA DOCUMENTADO ES EL CODIGO PARA TRABAJAR CON SEUDOMOTORES 360

