/*
5/14/18
Angela Almquist
Ethan Spaid 

This code implements a PID algorithm to control an inverted pendulum.
*/

//Optimal PID Gain Constants for 30ms sampling period
double k_p = 1.0;  
double k_i = 0.6;  
double k_d = 0.007; 

int MODE = 0;
// 0 == Stablizing
// 1 == Pendulum Fell => Control arm stops moving

//Arduino Pinouts
int direction = 2;
int clock = 4;
int voltage = A2;

//Other terms
int data[2] = {};
int error;
double integral = 0.0;
double pid = 0;
int setPoint = 458; //measured to be 453, but 458 worked best
long period = 100000;
unsigned long absPeriod=100000;
int frequency = 0;
int N = 0;

bool set = false; //To change setPoint
bool k = false;
bool k2;

//sampling
long samplePeriod = 30000;

//Timing
unsigned long time;
unsigned long currentMicros=100001;
unsigned long sampleTime;

//Enforce sampling period
unsigned long prevTime=0;
unsigned long delta_t;

void setup() 
{
  pinMode(direction,OUTPUT);
  pinMode(clock,OUTPUT);
  
  //Outputs to serial monitor
  Serial.begin(1000000);
    
  //Used to create initial (and incorrect) value of delta_t for first derivative term
  sampleTime = micros();
  time = micros();
}

void loop() 
{  
  if(MODE == 0)
  {
      sample(false);
      step();
      N++;   //N steps
      
      currentMicros=micros()-time;
      while(currentMicros<= (absPeriod)) //enforces the delay between control arm steps, very slight offset due to succeeding if statements
      {
        sample(false);
        currentMicros=micros()-time;
      }

      //switch to faster sampling rate if the error is high
      if (abs(error) < 10)
      {
        samplePeriod = 30000;
        k_p = 1.0;  
        k_i = 0.6;  
        k_d = 0.007; 
      }
      else
      {
        samplePeriod = 10000;
        k_p = 0.35;
        k_i = 0.11;  
        k_d = 0.008;
      }

      //Edit setPoint if control arm continuously rotates in one direction
      if((N%400 == 0) && (frequency < 100))
      {
        if (set == true)
        {
        setPoint = 457;
        set = false;
        }
        else
        {
          setPoint == 458;
          set = true;
        }
        N = 0;
      }
      
      time = micros();  
  }
  
  else  //pendulum has fallen
  {
    sample(true);
    if(abs(data[1]-setPoint)<3) //pendulum now in upright position
    {
      delay(1000);
      integral=0;
      frequency = 0;
      MODE = 0; //restart PID algorithm
    }
  }
}

void setPeriod()
{
  pid=PID();

  frequency += pid;
  if(abs(frequency) >12 || abs(frequency)< 6) //avoid frequencies that causes major whiplash in control arm
  {
    period=1000000/(frequency + .01);
  }
  absPeriod = abs(period);
  if(absPeriod < 1000)
  {
    //Serial.println("Program stopped because motor ran too fast");
    MODE = 1;
  } 
  
  k2=k;
  k = period <0; //switch directions
  
  if(k!=k2)
  {
     digitalWrite(direction,k);
     setPoint = 458; //default to 458
     N = 1;
  }
}

void sample(bool enable)
{
  delta_t=(micros()-sampleTime);
  if(delta_t >= samplePeriod || enable)
  {
    collectData();
    setPeriod();
  }
}

void collectData()
{
  data[0] = data[1];
  data[1] = analogRead(voltage);
  sampleTime = micros();

  error = (setPoint-data[1]);
 
  if(abs(error) < 128)
  {
    printData();
  }
  else
  {
    MODE=1;
  }
}

void printData()
{
     Serial.print(1e-3*sampleTime,4); //ms
     Serial.print(", ");
     Serial.println(error);
}

void step()  //control arm takes 1 x 0.9 degree step
{
  digitalWrite(clock,1);
  delayMicroseconds(4);
  digitalWrite(clock,0);
}

double PID()
{
  double correctionVal = k_p*(data[1]-setPoint) + k_d*Derivative() + k_i*Integral();
  return correctionVal;
}
double Derivative()
{
  return 1e6*(data[1]-data[0])/delta_t;
}
double Integral()
{
  integral += 1e-6*((data[1]-setPoint)-0.5*(data[1]-data[0]))*delta_t;
  return integral;
}



