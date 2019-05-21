/* 
 *  ENEE461 FINAL PROJECT: Simplified Autonomous Car
 *  
 *  This code implements the controller used to adjust the car's speed
 *  in the presence of obstacles either in front of or behind the car.
 *  
 *  To collect and visualize data from significant controller singals,
 *  a sketch written in Processing communicates with this program over a 
 *  common serial port and formats the data into a .csv file that can then be 
 *  plotted in Matlab. See the project Github repository for the \
 *  Processing source code.
 */

/* GLOBAL CONSTANTS */
// Define pin connections on the board:
const int pwmR = 3 ;      //initializing pin 3 as right motor pwm
const int in_1R = 7 ;
const int in_2R = 8 ;
const int pwmL = 5 ;      //initializing pin 5 as left motor pwm
const int in_1L = 12 ;
const int in_2L = 11 ;
const int Sig_Front = 2;  //front distance sensor output
const int Sig_Back = 10;  //back distance sensor output

// Define speed and distance limits:
const float speed_max = 60;
const float speed_min = 15;
const float distance_safe = 50;
const float distance_critical = 15;
const float velocity_initial = 50;


/* CONTROL SIGNALS & VALUES*/
float velocity_actual, velocity_need;
long duration_front, duration_back;
long distance_front, distance_back;
float velocity_final;
float c;
float proportional;
float integral;
double kp=0.4;
double kd=0.015;
double ki=0.1;
double currentTime, previousTime;
double elapsedTime;
double error, lastError;
double input, output, setPoint=15;
double cumError, rateError;

/* DATA COLLECTION */
double data[] = {0,15,0,0,0};      //Collect needed signals and send to serial port after each iteration

/* HELPER FUNCTIONS */
double PIDcontrol(double frontDist){
  currentTime = (double) millis();                       //get current time
  data[0] = currentTime;
  elapsedTime = (double)(currentTime - previousTime);    //compute time elapsed from previous computation
  
  error = setPoint - frontDist;                          //determine error
  data[3] = error;
  
  cumError = elapsedTime*(error);                        //compute integral
  
  rateError = (error - lastError)/elapsedTime;           //compute derivative

  double out = kp*error + ki*cumError + kd*rateError;    //PID output           
  
  lastError = error;                                     //remember current error
  previousTime = currentTime;                            //remember current time
  
  return -out;                                           //feedback
}

//The next few functions tell the motors what to do:
void GoForward(float speed){
  digitalWrite(in_1R,HIGH) ;
  digitalWrite(in_2R,LOW) ;
  analogWrite(pwmR,speed*1.08) ;
  digitalWrite(in_1L,HIGH) ;
  digitalWrite(in_2L,LOW) ;
  analogWrite(pwmL,speed*0.88) ;
  delay(100);
}
void GoBackward(float speed){
  digitalWrite(in_1R,LOW) ;
  digitalWrite(in_2R,HIGH) ;
  analogWrite(pwmR,speed*1.0) ;
  digitalWrite(in_1L,LOW) ;
  digitalWrite(in_2L,HIGH) ;
  analogWrite(pwmL,speed*0.78) ;
  delay(100);
  
}
void Brake(){
  digitalWrite(in_1L,HIGH) ;
  digitalWrite(in_2L,HIGH) ;
  digitalWrite(in_1R,HIGH) ;
  digitalWrite(in_2R,HIGH) ;
  delay(100) ;
}

//This function returns the distance measurement from the given sensor in centimeters
float GetDistance(int sig) {
  long duration;
  
  delay(100);
  pinMode(sig,OUTPUT);
  digitalWrite(sig,LOW);
  delay(2);
  digitalWrite(sig,HIGH);
  delay(5);
  digitalWrite(sig,LOW);
  pinMode(sig,INPUT);
  duration = pulseIn(sig,HIGH);

  return duration*0.034/2;
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
  pinMode(Sig_Front,INPUT);
  Serial.begin(9600);
}

void loop()
{
  int status = 0;
  velocity_actual = velocity_initial;
  
  while(status == 0){

    //Get front and back distances from sensors
    distance_front = GetDistance(Sig_Front);
    //If the front distance is 0, test again until its not
    while(distance_front == 0){
        distance_front = GetDistance(Sig_Front);
    }

    distance_back = GetDistance(Sig_Back);

    //If not within the target headway distance,
    // adjust speed relative to headway distance
    if(distance_front > distance_critical){
      
      c = float(distance_front / distance_safe);
    
      velocity_need = float(c * velocity_actual);
      if (velocity_need > speed_max || velocity_need < 0 ){
        velocity_need = speed_max; 
      }
      if (velocity_need < speed_min && velocity_need > 0){
        velocity_need = speed_min; 
      }
      data[0] = millis();
      data[2] = distance_front;
      data[3] = setPoint - distance_front;
      data[4] = velocity_need;
      GoForward(velocity_need);
      for(int i=0; i<5; i++) {
          Serial.print(data[i]);
          Serial.print(",");
        }
        Serial.println();
    }

    //If close enough to target headway distance,
    // use PID control to maintain that distance
    else if(distance_front <= distance_critical+2){
      
      if(distance_back < distance_critical){
        Brake();
        delay(3000);
      }

      cumError = 0;
      rateError = 0;
      previousTime = (double) millis();
      while(distance_front > 1 && distance_front < 25 && (distance_back > 20 || distance_back == 0)){
        
        distance_front = GetDistance(Sig_Front);
  
        //If the front distance is 0, test again until its not
        while(distance_front == 0){
            distance_front = GetDistance(Sig_Front);
        }
        data[2] = distance_front;
          
        output = PIDcontrol(distance_front);
  
        cumError = 0;
        rateError = 0;
        output *= 0.2;
        
        if(output > 0){
           if (output >speed_max){
            output = speed_max;
           }
           if (output < speed_min && output > 0){
              output = speed_min; 
           }
           GoForward(output);
           data[4] = output;
           delay(200);
           
        } else {
          output = -output;
          if (output >speed_max){
            output = speed_max;
           }
          if (output < speed_min && output > 0){
            output = speed_min; 
          }
          GoBackward(output);
          data[4] = output*(-1);
          delay(200);
        }

        for(int i=0; i<5; i++) {
          Serial.print(data[i]);
          Serial.print(",");
        }
        Serial.println();
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
