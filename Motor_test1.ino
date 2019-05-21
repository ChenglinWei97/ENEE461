const int pwmR = 3 ;  //initializing pin 2 as pwm
const int in_1R = 7 ;
const int in_2R = 8 ;
const int pwmL = 5 ;  //initializing pin 2 as pwm
const int in_1L = 12 ;
const int in_2L = 11 ;

//For providing logic to L298 IC to choose the direction of the DC motor 
void TurnLeft(int speed, int time){
  //For Turning LEFT
digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,LOW) ;
analogWrite(pwmR,speed) ;
digitalWrite(in_1L,LOW) ;
digitalWrite(in_2L,HIGH) ;
analogWrite(pwmL,speed) ;
delay(time);
//For brake
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,HIGH) ;
digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,HIGH) ;
delay(1000) ;
}

void TurnRight(int speed, int time){
  //For Turning RIGHT
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,LOW) ;
analogWrite(pwmL,speed) ;
digitalWrite(in_1R,LOW) ;
digitalWrite(in_2R,HIGH) ;
analogWrite(pwmR,speed) ;
delay(time);
//For brake
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,HIGH) ;
digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,HIGH) ;
delay(1000) ;
}

void setup()
{
pinMode(pwmR,OUTPUT) ;   //we have to set PWM pin as output
pinMode(in_1R,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_2R,OUTPUT) ;
pinMode(pwmL,OUTPUT) ;   //we have to set PWM pin as output
pinMode(in_1L,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_2L,OUTPUT) ;
}

void loop()
{
//For Clock wise motion , in_1 = High , in_2 = Low

digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,LOW) ;
analogWrite(pwmR,255) ;
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,LOW) ;
analogWrite(pwmL,255) ;
/*setting pwm of the motor to 255
we can change the speed of rotaion
by chaning pwm input but we are only
using arduino so we are using higest
value to driver the motor  */

//Clockwise for 3 secs
delay(3000) ;     

//For brake
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,HIGH) ;
digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,HIGH) ;
delay(1000) ;

//For Anti Clock-wise motion - IN_1 = LOW , IN_2 = HIGH
digitalWrite(in_1L,LOW) ;
digitalWrite(in_2L,HIGH) ;
digitalWrite(in_1R,LOW) ;
digitalWrite(in_2R,HIGH) ;
delay(3000) ;

//For brake
digitalWrite(in_1L,HIGH) ;
digitalWrite(in_2L,HIGH) ;
digitalWrite(in_1R,HIGH) ;
digitalWrite(in_2R,HIGH) ;
delay(1000) ;

//For Turning LEFT
TurnLeft(255,3000);

//For Turning RIGHT
TurnRight(255,3000);

 }
