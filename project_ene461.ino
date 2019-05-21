const int pwmR = 3 ;  //initializing pin 2 as pwm
const int in_1R = 7 ;
const int in_2R = 8 ;
const int pwmL = 5 ;  //initializing pin 2 as pwm
const int in_1L = 12 ;
const int in_2L = 11 ;
const int Sig_Front = 2;
const int Sig_Back = 10;
float speed_max = 60;
float speed_min = 15;
float velocity_actual;
float velocity_need;
long duration_front;
long distance_front;
long duration_back;
long distance_back;
float velocity_final;
float velocity_initial = 50;
float c;
float proportional;
float integral;
float distance_safe = 50;
float distance_critical = 15;

double kp=0.4;
double kd=0.02;
double ki=0.1;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint=15;
double cumError, rateError;
double PIDcontrol(double inp){
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;   // compute derivative

  double out = kp*error + ki*cumError + kd*rateError;                //PID output               
  
  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time
  
  return -out; // feedback
}

void GoForward(float speed){
  digitalWrite(in_1R,HIGH) ;
  digitalWrite(in_2R,LOW) ;
  analogWrite(pwmR,speed*1.08) ;
  digitalWrite(in_1L,HIGH) ;
  digitalWrite(in_2L,LOW) ;
  analogWrite(pwmL,speed*0.88) ;
  Serial.print("Going Forward at speed");
  Serial.println(speed);
  delay(100);
}

void GoBackward(float speed){
  digitalWrite(in_1R,LOW) ;
  digitalWrite(in_2R,HIGH) ;
  analogWrite(pwmR,speed*1.0) ;
  digitalWrite(in_1L,LOW) ;
  digitalWrite(in_2L,HIGH) ;
  analogWrite(pwmL,speed*0.78) ;
  Serial.print("Going Backward at speed");
  Serial.println(speed);
  delay(100);
  
}
void Brake(){
  digitalWrite(in_1L,HIGH) ;
  digitalWrite(in_2L,HIGH) ;
  digitalWrite(in_1R,HIGH) ;
  digitalWrite(in_2R,HIGH) ;
  delay(100) ;
}

void setup()
{
  pinMode(pwmR,OUTPUT) ;   //we have to set PWM pin as output
  pinMode(in_1R,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in_2R,OUTPUT) ;
  pinMode(pwmL,OUTPUT) ;   //we have to set PWM pin as output
  pinMode(in_1L,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in_2L,OUTPUT) ;
  pinMode(Sig_Back,INPUT);
  Serial.begin(9600);
}

void loop()
{
int status = 0;

velocity_actual = velocity_initial;
while(status == 0){
  
  delay(100);
  pinMode(Sig_Front,OUTPUT);
  digitalWrite(Sig_Front,LOW);
  delay(2);
  digitalWrite(Sig_Front,HIGH);
  delay(5);
  digitalWrite(Sig_Front,LOW);
  pinMode(Sig_Front,INPUT);
  duration_front = pulseIn(Sig_Front,HIGH);
  distance_front = duration_front*0.034/2;
  Serial.print("Front Distance:");
  Serial.println(distance_front);

  //If the front distance is 0, test again until its not
  while(distance_front == 0){
      delay(100);
      pinMode(Sig_Front,OUTPUT);
      digitalWrite(Sig_Front,LOW);
      delay(2);
      digitalWrite(Sig_Front,HIGH);
      delay(5);
      digitalWrite(Sig_Front,LOW);
      pinMode(Sig_Front,INPUT);
      duration_front = pulseIn(Sig_Front,HIGH);
      distance_front = duration_front*0.034/2;
      Serial.print("Front Distance:");
      Serial.println(distance_front);
  }
  
  pinMode(Sig_Back,OUTPUT);
  digitalWrite(Sig_Back,LOW);
  delay(2);
  digitalWrite(Sig_Back,HIGH);
  delay(5);
  digitalWrite(Sig_Back,LOW);
  pinMode(Sig_Back,INPUT);
  duration_back = pulseIn(Sig_Back,HIGH);
  distance_back = duration_back*0.034/2;
  Serial.print("Back Distance:");
  Serial.println(distance_back);

  c = float(distance_safe / distance_front) ;
  Serial.print("c is");
  Serial.println(c);
  velocity_need = float(velocity_actual / c);
  if (velocity_need > speed_max ||velocity_need < 0 ){
    velocity_need = speed_max; 
  }
  if (velocity_need < speed_min && velocity_need > 0){
    velocity_need = speed_min; 
  }
  Serial.print("speed is");
  Serial.println(velocity_need);
  if(distance_front>distance_critical){
    GoForward(velocity_need);
  }
  else if(distance_front <= distance_critical){
    if(distance_back < 15){
      Brake();
      delay(3000);
    }
    while(distance_front > 1 && distance_front < 30 && (distance_back > 20||distance_back == 0)){
        delay(100);
        pinMode(Sig_Front,OUTPUT);
        digitalWrite(Sig_Front,LOW);
        delay(2);
        digitalWrite(Sig_Front,HIGH);
        delay(5);
        digitalWrite(Sig_Front,LOW);
        pinMode(Sig_Front,INPUT);
        duration_front = pulseIn(Sig_Front,HIGH);
        distance_front = duration_front*0.034/2;

          //If the front distance is 0, test again until its not
  while(distance_front == 0){
      delay(100);
      pinMode(Sig_Front,OUTPUT);
      digitalWrite(Sig_Front,LOW);
      delay(2);
      digitalWrite(Sig_Front,HIGH);
      delay(5);
      digitalWrite(Sig_Front,LOW);
      pinMode(Sig_Front,INPUT);
      duration_front = pulseIn(Sig_Front,HIGH);
      distance_front = duration_front*0.034/2;
      Serial.print("Front Distance:");
      Serial.println(distance_front);
  }
        
      output = PIDcontrol(distance_front);

      
      cumError = 0;
      rateError = 0;
      output *=0.2;
      Serial.print("output is");
      Serial.println(output);
      Serial.print("Front Distance:");
      Serial.println(distance_front);
      if(output > 0){
         if (output >speed_max){
          output = speed_max;
         }
         if (output < speed_min && output > 0){
            output = speed_min; 
         }
         GoForward(output);
         delay(200);
      }
      else{
        output = -output;
        if (output >speed_max){
          output = speed_max;
         }
        if (output < speed_min && output > 0){
          output = speed_min; 
        }
        GoBackward(output);
        delay(200);
      }

    }
  }
  
  
}
delay(10000);
/*setting pwm of the motor to 255
we can change the speed of rotaion
by chaning pwm input but we are only
using arduino so we are using higest
value to driver the motor  */


 }
