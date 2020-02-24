#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <cmath>

void waitForStart();
void rot(int mode,float angle);
void trans(int mode,float angle,float velocity);
void moveToButton(int color);
void StopAll();
int testForColor();

//0 for nothing, 1 for blue, 2 for red


FEHMotor motor1(FEHMotor::Motor0,12); //Back Motor
FEHMotor motor2(FEHMotor::Motor1,12); //Left Motor
FEHMotor motor3(FEHMotor::Motor2,12); //Right Motor

AnalogInputPin   CdS_cell(FEHIO::P0_0);

AnalogInputPin  lineR(FEHIO::P1_0);
AnalogInputPin  lineC(FEHIO::P1_1);
AnalogInputPin  lineL(FEHIO::P1_2);

//Constants
#define wheeldia            2.0
#define centerrad           3.0
#define yodersConstant      0.01
#define rotpercent       25.0
#define transpercent    30.0
#define maximumpercent      50.0
#define PI                  3.14159265358979323846
/* (motor #2)   (motor #3)
 *  --> O-----O <--
 *       \   /
 *        \ /
 *         o
 *      (motor #1)
 *
*/

//Motor Rotational Speeds Relative to the Back Motor Moving clockwise
#define motor1multiccw      0.93           //This motor is really bad
#define motor1multicw       0.97           //Each Multiplier modifies the percent each wheel gets
#define motor2multiccw      1.0
#define motor2multicw       1.0
#define motor3multiccw      1.03           //Tuning from vex variance
#define motor3multicw       1.05           //Tuning from vex variance

//Threshold values for the CdS Cells
#define BlueMax     1.5
#define BlueMin     0.5
#define RedMax      0.49999999
#define RedMin      0

//Threshold values for the Optosensors
#define lineMin     3.3
#define lineMax     3.3

int main() {
    float startime;
    float endtime;
    int mode = 3;
    switch(mode) {
    case 1:

        //This is the tuning area
        //Test our motors
        LCD.Clear();
        LCD.WriteRC("Testing Motors",3,3);
        motor1.SetPercent(20);
        LCD.WriteRC("Motor1",4,3);
        Sleep(1.0);
        StopAll();
        motor2.SetPercent(20);
        LCD.WriteRC("Motor2",5,3);
        Sleep(1.0);
        StopAll();
        motor3.SetPercent(20);
        LCD.WriteRC("Motor3",6,3);
        Sleep(1.0);
        StopAll();
        //Test the CDS Cell
        LCD.Clear();
        LCD.WriteRC("CdS_cell Voltage",3,3);
        LCD.WriteRC("Optosensor Voltage",6,3);
        LCD.WriteRC("Right",7,3);
        LCD.WriteRC("Center",8,3);
        LCD.WriteRC("Left",9,3);
        while(true) {
            LCD.WriteRC(CdS_cell.Value(),4,3);
            LCD.WriteRC(lineR.Value(),7,10);
            LCD.WriteRC(lineC.Value(),8,10);
            LCD.WriteRC(lineL.Value(),9,10);
        }
        break;
    case 2:
        //This is the main course running function!
        LCD.Clear();
        waitForStart();


        break;
    case 3: //Tune Rotaton
        rot(1,3600);

        break;

    default:
        LCD.Clear();
        LCD.WriteRC("ERROR: Not a valid mode.",0,0);
        break;
    }
}

void waitForStart() {
    while(testForColor() != 2) {
        LCD.WriteRC("Waiting for Course Light...",3,3);
    }
}

void rot(int mode,float angle) {  //Dumbed down verison of rot, much more reliable
    switch(mode) {
        case 1 :
            float direction;
            float seconds;
            if(angle >= 0 ){
                direction = -1.0;
            }   else if (angle < 0) {
                direction = 1.0;
            }
            seconds = abs(angle * yodersConstant);
            motor1.SetPercent(rotpercent * direction);
            motor2.SetPercent(rotpercent * direction);
            motor3.SetPercent(rotpercent * direction);
            Sleep(seconds);
            StopAll();
            break;
        case 2:
            //Currently Defunct, Waiting for RPS exploration
            break;
    }
}

void trans(int mode,float angle,float velocity) { // The current kinematics incur significant dift when not moving in a cardinal direction (UP DOWN LEFT RIGHT)
    switch(mode) {
        case 1: //translate from robots local heading
            float x;
            float y;
            x = cos(((angle+90)/180) * PI);
            y = sin(((angle+90)/180) * PI);

            float m1percent;
            float m2percent;
            float m3percent;

            m1percent = -1 * x;     //X Equations
            m1percent = m1percent + 0;   //Y Equations
            m1percent = m1percent * velocity;

            m2percent = cos(PI/3) * x;   //X Equations
            m2percent = m2percent + sin(PI/3) * y;   //Y Equations
            m2percent = m2percent * velocity;

            m3percent = cos(PI/3) * x;   //X Equations
            m3percent = m3percent - sin(PI/3) * y;   //Y Equations
            m3percent = m3percent * velocity;

            if(m1percent <= 0) {
                motor1.SetPercent(m1percent * motor1multicw);
            }
            else {
                motor1.SetPercent(m1percent * motor1multiccw);
            }

            if(m1percent <= 0) {
                motor2.SetPercent(m2percent * motor2multicw);
            }
            else {
                motor2.SetPercent(m2percent * motor2multiccw);
            }

            if(m1percent <= 0) {
                motor3.SetPercent(m3percent * motor3multicw);
            }
            else {
                motor3.SetPercent(m3percent * motor3multiccw);
            }

            break;
        case 2:	//trans relative to RPS
            break;
    }
}

void StopAll() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
}

int testForColor() {
    if(CdS_cell.Value() >= BlueMin && CdS_cell.Value() <= BlueMax) {
        return 1; //Blue
        LCD.WriteRC("Its Blue!",4,4);
    }
    else if(CdS_cell.Value() >= RedMin && CdS_cell.Value() <= RedMax) {
        return 2; //Red
        LCD.WriteRC("Its Red!",4,4);
    }
    else {
        LCD.WriteRC("No Color Detected",4,4);
        return 0; //No Color
    }
}

void moveToButton(int color) {
    if(color == 1) {//The color is blue is on the left
        trans(1,90,30);
        Sleep(1.0);
        StopAll();

        trans(1,0,30);
        Sleep(1.0);
        StopAll();

        trans(1,180,30);
        Sleep(1.0);
        StopAll();

        trans(1,-90,30);
        Sleep(1.0);
        StopAll();
    }
    else if (color == 2) {//The color is red is on the right
        trans(1,90,30);
        Sleep(1.0);
        StopAll();

        trans(1,0,30);
        Sleep(1.0);
        StopAll();

        trans(1,180,30);
        Sleep(1.0);
        StopAll();

        trans(1,-90,30);
        Sleep(1.0);
        StopAll();
    }
    else {
        //Do nothing if no color is detected
    }
}
