#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>

//Define motors
FEHMotor motor1(FEHMotor::Motor0, 12); // back motor
FEHMotor motor2(FEHMotor::Motor1, 12); //left motor
FEHMotor motor3(FEHMotor::Motor2, 12); //right motor

//Define Constants

#define PI 3.14159265358979323846264338327950288419716939937510
#define Yodersconstant 0.750
#define lmotorcorrection 0.90// correction factor for left wheel because it is stoonger
#define rmotorcorrection 1.025//correction factor for right wheel because it is weaker
//Movement functions
void trans_for(float distance, float mpercent); //translational motion function
void rot(float angle, float mpercent);//rotation function


int main(void)
{
LCD.Clear();
LCD.Write("Starting now");
Sleep(5);
trans_for(23,25);
LCD.Clear();
LCD.Write("Check Up Bitch");
//LCD.Clear();
//LCD.Write("Now Turning");
//LCD.Clear();
//Sleep (5);
//rot(360,25);

}
//Function prototype for translational movement function
void trans_for(float distance, float mpercent)
{
    float time;
    time=(distance/4.6); //4.6 in/s for 25% motor percent
    //Setting motor percent to make the robot move forward
    motor2.SetPercent(mpercent*lmotorcorrection);
    motor3.SetPercent(-mpercent*rmotorcorrection);
    //Move for calculated time
    Sleep(time); //Will change so that the time is calculated from the distance
    //Stop the motors
    motor2.Stop();
    motor3.Stop();
}
//Function prototype for rotation
void rot(float angle, float mpercent)
{
    float ang_rad;
    float t;
    //calculate angle in radians
    ang_rad=(angle*PI/180);
    //calculate time from the necessary angle
    t=Yodersconstant*ang_rad;
    if (angle>0) //If the angle of the turn is greater than 0, then the turn will be cw
    {
        motor1.SetPercent(mpercent);
        motor2.SetPercent(mpercent);
        motor3.SetPercent(mpercent);
        Sleep(t);
        motor1.Stop();
        motor2.Stop();
        motor3.Stop();
    }

    else if (angle<0) //if the angle of the turn is less than 0, then the turn will be ccw
    {
        motor1.SetPercent(-mpercent);
        motor2.SetPercent(-mpercent);
        motor3.SetPercent(-mpercent);
        Sleep(t);
        motor1.Stop();
        motor2.Stop();
        motor3.Stop();
    }
    else //if for some reason it does not turn this will show that the error is in the logic
    {
        LCD.Clear();
        LCD.Write("ERROR");
    }
}
