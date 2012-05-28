void output_control (int watts) {
  int pwmvalue = watts;
  analogWrite(SSRpin, pwmvalue);
}


// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/

//#include <PID_v1.h>

//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

//void setup()
//{
  //initialize the variables we're linked to
 // Input = analogRead(0);
//  Setpoint = 100;

  //turn the PID on
//  myPID.SetMode(AUTOMATIC);
//}

//void loop()/
//{
//  Input = analogRead(0);
//  myPID.Compute();
//  analogWrite(3,Output);
//}



