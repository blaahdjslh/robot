#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <math.h>


void waitForStart();
//Waits until the color red is detected by the robot

void rot(int mode,float angle);
//Rotates the Robot
//Input mode        Rotates relative to robot(1) Rotates to RPS relative angle(2)
//Input angle       Amount to rotate in degrees sign dictates rotation direction(1), Angle to match the RPS (2)

void trans(int mode,float angle,float velocity);
//Translates the Robot
//Input mode        Translating relative to robot(1) or RPS(2)
//Input angle       Angle to translate at(1), angle to translate at relative to rps(2)
//Input velocity    Speed to translate at (in motor %)

//void transR(int mode, float angle, float velocity);
//Same as trans except attempts to compensate for extra rotation incurred when translating
//at non-cardinal directions (EX:45 Degrees)

void transD(float angle, float velocity, float distance);
//Translates the Robot a specific distance at a specific velocity
//Input angle       Angle to translate at
//Input velocity    Speed to translate at (in motor %)
//Input distance    Distance to Travel in Inches

//void transDR(float angle, float velocity, float distance);
//Same as transD except attempts to compensate for extra rotation incurred when translating
//at non-cardinal directions (EX: 45 Degrees)

void moveToButton(int color);
//Moves to button based on color input

//void scanForLight();
//Scans back and forth until a light is seen
//Potential problems : ----> Could possibly think light is only blue due to stopping when it senses either color.
//                           Therefore it could stop before it realizes the color is red, that is really bad.
//                           Properly accounting for this would take more code that is really necessary so
//                           just tune the line following movement.
//Comment : This method should not be used for alignment, use the line following kit instead.

void centerOnLine(float limit);
//Stops the Robot when a line is at its center
//Input limit : Maximum amount of time the robot can look for a line in seconds (helpful for breaking search loop)
//Comment :

void endOnLine(float limit);
//Stops the Robot when no lines are detected by it
//Input limit : Maximum amount of the robot can look fo the end of a line in seconds (helpful for breaking search loop)
//Comment :

void StopAll();
//Stops all motors
//Comment :

int testForColor();
//Tests for the color currently detected by the CdS Cell
//0 for no color
//1 for blue detected
//2 for red detected

int testForLine();
//Tests for position of the line detected by the cell
//0 For no line detected
//10 Line on the left
//15 Line on the center and left
//20 Line on the center
//25 Line on the center and right
//30 Line on the right

FEHMotor motor1(FEHMotor::Motor0,12); //Back Motor
FEHMotor motor2(FEHMotor::Motor1,12); //Left Motor
FEHMotor motor3(FEHMotor::Motor2,12); //Right Motor

AnalogInputPin   CdS_cell(FEHIO::P0_0);

AnalogInputPin  lineR(FEHIO::P1_0);
AnalogInputPin  lineC(FEHIO::P1_1);
AnalogInputPin  lineL(FEHIO::P1_2);

//------Constants------//
#define wheeldia            2.0
#define centerrad           3.0
#define yodersConstant      0.0005028464    //<----- Dr. Yoder
//Data for Yoder's Constant
//0.01257116 at 25%
#define rotpercent          25.0
#define transpercent        30.0
#define PI                  3.14159265358979323846
/* (motor #2)   (motor #3)
 *  --> O-----O <--
 *       \   /
 *        \ /
 *         o
 *      (motor #1)
 *
*/

//Motor Rotational Speed multipliers to account for variance.
//Currently only y = ax calcualtions take place to determine motor multiplier
//Future versions should implement y = ax + b in order to account for friction
//These are the "a" constants
#define motor1multicw       1.0         // <------ Reference, do not tune
#define motor1multiccw      1.0
#define motor2multicw       1.0
#define motor2multiccw      1.0
#define motor3multicw       1.0
#define motor3multiccw      1.0
//These are the "b" constants
#define motor1multicwo      0.0         // <------ Reference, do not tune
#define motor1multiccwo     0.0
#define motor1multicwo      0.0
#define motor1multiccwo     0.0
#define motor1multicwo      0.0
#define motor1multiccwo     0.0

//Motor %
#define dvx                 0.203
//Data
//6.1 in/s at 30% ---> 0.203
#define dvy                 0.187
//Data
//5.6 in/s at 30% ---> 0.187
#define R_constant          0.01
//Data
//None (!)            

//Threshold values for the CdS Cells
#define BlueMax             1.5
#define BlueMin             0.5
#define RedMax              0.49999999
#define RedMin              0

//Threshold values for the Optosensors
#define lineMax     3.3
#define lineMin     1.9

//Robot Diagnostics Display Current Orientation
/*
 *   0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
 *   __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
 *0 |CdS_Voltage Label                      Diagnostics Label
 *1 |CdS_Voltage
 *2 |
 *3 |
 *4 |Optosensor_Voltage Label
 *5 |Left                         Center                     Right
 *6 |Left Voltage                 Center Voltage             Right Voltage
 *7 |
 *8 |lastDetectedColor Label
 *9 |lastDetectedColor Value
 *10|
 *11|lastDetectedLine Label
 *12|lastDetectedLine Value
 *13|
 */

int main() {
    float startime, endTime;
    int mode = 2;
    
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
        //waitForStart();
        LCD.Clear();
        //Diagnostic Labels
        LCD.WriteRC("CdS Voltage",0,0);
        LCD.WriteRC("Diagnostics",0,14);
        LCD.WriteRC("Optosensor Voltage",4,0);
        LCD.WriteRC("Left",6,0);
        LCD.WriteRC("Center",6,8);
        LCD.WriteRC("Right",6,16);
        LCD.WriteRC("last color",8,0);
        LCD.WriteRC("last line",11,0);

        while(true) { //Still testing the sensors
            testForColor();
            testForLine();
        }
        break;
    case 3: //Tune Rotaton
        rot(1,3600);
        break;
    case 4: //Tune Translation
        trans(1,0,30);
        Sleep(3.0);
        break;
    default:
        LCD.Clear();
        LCD.WriteRC("ERROR: Not a valid mode.",0,0);
        break;
    }
}

void waitForStart() {
    while(testForColor() != 2) {
        LCD.WriteRC("Waiting For Light",3,3);
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
            seconds = angle * yodersConstant * rotpercent;
            seconds = fabs(seconds);
            motor1.SetPercent(rotpercent * direction);
            motor2.SetPercent(rotpercent * direction);
            motor3.SetPercent(rotpercent * direction);
            //LCD.WriteRC(seconds,13,0);
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

            if(m2percent <= 0) {
                motor2.SetPercent(m2percent * motor2multicw);
            }
            else {
                motor2.SetPercent(m2percent * motor2multiccw);
            }

            if(m3percent <= 0) {
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
//void transR(int mode, float angle, float velocity);

void transD(float angle, float velocity, float distance) {
    //Declare Variables
    float x,y;
    float m1percent, m2percent, m3percent;
    float Dx, Dy, seconds;
    //Determine seconds to sleep
    x = cos(((angle+90)/180) * PI);
    y = sin(((angle+90)/180) * PI);
    Dx = fabs(x * distance);
    Dy = fabs(y * distance);
    seconds = Dx / (velocity * dvx) +  Dy / (velocity * dvy);
    //Determine Motor Speeds //X             //Y
    m1percent = velocity * ( -1 * x          + 0 * y );
    m2percent = velocity * ( cos(PI/3) * x   + sin(PI/3) * y );
    m3percent = velocity * ( cos(PI/3) * x   - sin(PI/3) * y );
    //Set Motor Speeds
    if(m1percent <= 0) {
        motor1.SetPercent(m1percent * motor1multicw);
    } else {
    motor1.SetPercent(m1percent * motor1multiccw);
    }
    if(m2percent <= 0) {
        motor2.SetPercent(m2percent * motor2multicw);
    } else {
        motor2.SetPercent(m2percent * motor2multiccw);
    }
    if(m3percent <= 0) {
        motor3.SetPercent(m3percent * motor3multicw);
    } else {
        motor3.SetPercent(m3percent * motor3multiccw);
    }
    //Sleep then Stop
    Sleep(seconds);
    StopAll();
}

//void transDR(float angle, float velocity, float distance)

void StopAll() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
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
        trans(1,-90,30);
        Sleep(1.0);
        StopAll();

        trans(1,0,30);
        Sleep(1.0);
        StopAll();

        trans(1,180,30);
        Sleep(1.0);
        StopAll();

        trans(1,90,30);
        Sleep(1.0);
        StopAll();
    }
    else {
        LCD.WriteRC("ERROR:NOBUTTONCOLOR",13,0);
    }
}

void scanForLight() {
    //Currently Defunct
}

void centerOnLine(float limit) {
    float eTime = limit + TimeNow();
    while(eTime > TimeNow()) {
        if(testForLine() == 20) {
            break;
        }
    }
}

void endOnLine(float limit) {
    float eTime = limit + TimeNow();
    while(eTime > TimeNow()) {
        if(testForLine() == 0) {
            break;
        }
    }
}

int testForColor() {
    LCD.WriteRC(CdS_cell.Value(),1,0);
    LCD.WriteRC("        ",9,0);
    if(CdS_cell.Value() >= BlueMin && CdS_cell.Value() <= BlueMax) {
        LCD.WriteRC("1 - Blue",9,0);
        return 1; //Blue
    }
    else if(CdS_cell.Value() >= RedMin && CdS_cell.Value() <= RedMax) {
        LCD.WriteRC("2 - Red ",9,0);
        return 2; //Red
    }
    else {
        LCD.WriteRC("0 - None",9,0);
        return 0; //No Color
    }
}

int testForLine() {
    float rV,cV,lV;
    int rO,cO,lO;
    int state;
    rV = lineR.Value();
    cV = lineC.Value();
    lV = lineL.Value();

    if(rV > 1.9) {
        rO = 1;
    }
    else {
        rO = 0;
    }

    if(cV > 1.9) {
        cO = 1;
    }
    else {
        cO = 0;
    }

    if(lV > 1.0) {
        lO = 1;
    }
    else {
        lO = 0;
    }

    if(rO + cO + lO == 0) {
        state = 0;
    }
    if(rO == 1 && cO + lO == 0) {
        state = 10;
    }
    if(rO + cO == 2 && lO == 0) {
        state = 15;
    }
    if(rO + cO + lO == 3) {
        state = 20;
    }
    if(cO == 1 && rO + lO == 0) {
        state = 20;
    }
    if(lO + cO == 2 && rO == 0) {
        state = 25;
    }
    if(lO == 1 && cO + rO == 0) {
        state = 30;
    }

    LCD.WriteRC("  ",12,0);
    LCD.WriteRC(state,12,0);
    return state;
}
