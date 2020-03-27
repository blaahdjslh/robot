#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHRPS.h>
#include <math.h>



void waitForStart(float waitTime);
//Waits until the color red is detected by the robot

void rot(float angle);
//Rotates the Robot
//Input angle       Amount to rotate in degrees sign dictates rotation direction

void rotRPS(float angle);
//Rotates the robot to match the desired RPS heading
//Input angle   Heading to match

void headRPS(float angle);
//Executes rotRPS twice with RPS settling time
void xyRPS(float x, float y);
//Executes matchRPS twice with RPS settlings time
//Only works whne robot is pointing straight foward (0.0)! will be fixed later

void tran(int mode,float angle,float velocity);
//tranlates the Robot
//Input mode        tranlating relative to robot(1) or RPS(2)
//Input angle       Angle to tranlate at(1), angle to tranlate at relative to rps(2)
//Input velocity    Speed to tranlate at

void matchRPS(float x, float y);
void locate(float x, float y);

void alignRPS(float x,float y);
//translates horizontal and vertical in order to match to RPS coordinates
//Only works when robot is pointing straight forward (0.0)! will be fixed later

float aRPS(float angle);
//Calculates the smallest possible angle between the robot and the desired RPS coordinate
//Input angle desired RPS angle

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

void stop();
//Stops all motors
//Comment :

void servoArm(float degree);

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
//50 For all lines detected
//100 for Left and Right Detected

FEHMotor motor1(FEHMotor::Motor0,12); //Back Motor
FEHMotor motor2(FEHMotor::Motor1,12); //Left Motor
FEHMotor motor3(FEHMotor::Motor2,12); //Right Motor

AnalogInputPin   CdS_cell(FEHIO::P0_0); //CdS Cell

AnalogInputPin  lineR(FEHIO::P1_0); //Line following kit right
AnalogInputPin  lineC(FEHIO::P2_0); // Line following kit middle
AnalogInputPin  lineL(FEHIO::P3_0); //Line following kit left

FEHServo arm(FEHServo::Servo7); //Servo Motor


//------Constants------//

#define wheeldia            2.0
#define centerrad           3.0
#define yodersConstant      0.00049526269
//Old 0.00049663841
//Old 0.0005028464    //<----- Dr. Yoder

//Data
//0.01257116 at 25%
#define rotpercent          25.0
#define tranpercent         30.0
#define maximumpercent      50.0
#define PI                  3.14159265358979323846

#define vx                  6   //  in/s
#define vy                  6   //  in/s
#define rpsratio            1   //  Realin/RPSin
#define rpsOffset           180.00  //Angle that is pointing "forward" from rps heading
/* (motor #2)   (motor #3)
 *  --> O-----O <--
 *       \   /
 *        \ /
 *         o
 *      (motor #1)
 *
*/
/*  Arm horizontal at 20%
 *  Arm vertical at 100%
 *
 *
 *
 */
/*//Robot coordinates
 *
 *                  0 degrees
 *                      x
 *                      |
 *                      |
 *                      |
 * 90 degrees  <--------o-------->
 *
 *
 * //RPS coordinates
 *
 *                  180 degrees
 *                       x
 *                       |
 *                       |
 *                       |
 * 270 degrees  <--------o-------->
 */

//Motor Rotational Speeds.
//Motor 1 cw is always defined as 1.0
#define motor1multiccw      0.95           //This motor is really bad
#define motor1multicw       0.975          //Each Multiplier modifies the percent each wheel gets
#define motor2multiccw      1.0
#define motor2multicw       1.0
#define motor3multiccw      1.01           //Tuning from vex variance
#define motor3multicw       1.01           //Tuning from vex variance


//Threshold values for the CdS Cells
#define BlueMax     1.5
#define BlueMin     0.5
#define RedMax      0.49999999
#define RedMin      0


//Threshold values for invidivual Optosensors
#define lineRMin    3.0
#define lineRMax    3.4

#define lineCMin    3.22
#define lineCMax    3.4

#define lineLMin    2.9
#define lineLMax    3.4
/*
#define lineRMin    3.1
#define lineRMax    3.3

#define lineCMin    3.22
#define lineCMax    3.3

#define lineLMin    2.7
#define lineLMax    3.3
*/
//Servo arm constants
#define ServoMax 2328
#define ServoMin 500



//Robot Diagnostics Display

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
 *8 |lastDetectedColor Label9
 *9 |lastDetectedColor Value
 *10|
 *11|lastDetectedLine Label
 *12|lastDetectedLine Value
 *13|
 */

int main() {
    //arm.TouchCalibrate();
    servoArm(100.0);
    arm.SetMin(ServoMin);
    arm.SetMax(ServoMax);
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
        stop();
        motor2.SetPercent(20);
        LCD.WriteRC("Motor2",5,3);
        Sleep(1.0);
        stop();
        motor3.SetPercent(20);
        LCD.WriteRC("Motor3",6,3);
        Sleep(1.0);
        stop();
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
            LCD.WriteRC(testForLine(),11,10);
        }

        break;

    case 2:

        //This is the main course running function!

        LCD.Clear();
        //waitForStart(10.0);
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


        //tran(1,0,30);
        //Sleep(1.0);
        //stop(); // Step 1
        //rot(45); // Step 2
        //tran(1,0,30);
        //Sleep(2.9);
        //stop(); //Step 3
        //rot(-81.5); //Step 4 //Needs to be tuned later, add motor correction factors
        //tran(1,0,45);
        //Sleep(3.0); // Adjusted time
        //stop(); //Step 5 (the approach)
        //tran(1,180,30);
        //Sleep(2.0);
        //stop();//Step 6 back up to approach ramp
        //rot(-90);//Step 7 turn to the right
        //tran(1,0,30);
        //Sleep(2.0);
        //stop();// Step 8 Line up with ramp
        //rot(90);//Step 9 trun left to face the ramp
        //tran(1,0,50);
        // Sleep(3.2);
        // stop();//Step 10: Go up the ramp

    //PT3 code starts here


       // tran(1,0,30);
       // Sleep(1.7);
       // stop(); //Step 1: Leave platform
       // rot(-45);// Step 2: turn to face ramp
       // tran(1,0,75);
       // Sleep(4.0);
       // stop();

        rot(180);
        servoArm(0.);
        tran(1,180,30);
        Sleep(1.2);
        stop();
        servoArm(35.);
        Sleep(2.0);
        tran(1,180,10);
        Sleep(2.3);
        stop();
        servoArm(90.);


        //For the wheel

        while(true) { //Still testing the sensors

            testForColor();

            testForLine();

        }

        break;

    case 3: //Test Wall Alignment
        RPS.InitializeTouchMenu();
        waitForStart(5.0);
        Sleep(5.0);
        tran(1,0,30);
        Sleep(1.5);
        headRPS(0.0);
        xyRPS(18.0,18.0);
        headRPS(0.0);
        tran(1,0,80);
        Sleep(2.4);
        headRPS(0.0);
        matchRPS(17.0,50);
        headRPS(0.0);
        matchRPS(17.0,50);
        rot(-90);
        tran(1,0,30);
        Sleep(3.0);
        tran(1,0,70);
        Sleep(0.1);
        tran(1,180,30);
        Sleep(1.23);
        rot(90);
        //headRPS(0.0);
        tran(1,0,20);
        while(testForLine() != 100) {
            //Do nothing and wait
        }
        //endOnLine(5.0);
        stop();

        //Put the Burger Up
        servoArm(20.0);
        rot(180);

        tran(1,180,15);
        Sleep(.4);
        servoArm(50.0);
        tran(1,180,14);
        Sleep(0.75);
        servoArm(70.0);
        Sleep(0.75);
        servoArm(120.0);
        Sleep(2.0);
        stop();

        //Leave the burger and recalibrate
        tran(1,0,30);
        Sleep(1.0);
        servoArm(100.0);
        stop();

        //Flip down the burger
        tran(1,90,30);
        Sleep(0.33);
        stop();
        tran(1,180,30);
        Sleep(0.7);
        stop();
        rot(-30);
        servoArm(70);
        rot(45);
        Sleep(0.5);
        servoArm(100.0);
        tran(1,0,30);
        Sleep(1.0);
        stop();

        //Try for a lever
        headRPS(0.0);
        matchRPS(17.0,50);
        headRPS(0.0);
        matchRPS(17.0,50);
        rot(45);
        tran(1,0,30);
        Sleep(1.0);
        rot(182);
        servoArm(100.0);
        tran(1,180,30);
        Sleep(1.0);
        tran(1,90,30);
        Sleep(0.33);
        rot(60);
        servoArm(50);
        stop();
        tran(1,0,15);
        Sleep(3.0);
        stop();
        /*
        headRPS(180);
        xyRPS(25,54);
        rot(90);
        tran(1,0,30);
        tran(1,0,30);
        Sleep(1.5);
        stop();
        tran(1,180,30);
        Sleep(1.0);
        rot(180);
        */
        break;

    case 4: //Feedback
        RPS.InitializeTouchMenu();
        while(true) {
        LCD.WriteRC(RPS.X(),8,1);
        LCD.WriteRC(RPS.Y(),8,10);
        LCD.WriteRC(RPS.Heading(),10,1);
        Sleep(100);
        }
        break;
    default:
        LCD.Clear();
        LCD.WriteRC("ERROR: Not a valid mode.",0,0);
        break;
    }
}

void waitForStart(float waitTime) {
    float endTime;
    endTime = waitTime + TimeNow();
    while( (testForColor() != 2) && endTime < TimeNow()) {
        LCD.WriteRC("Waiting For Light",3,3);
    }
}

void rot(float angle) {  //Dumbed down verison of rot, much more reliable
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
    stop();
}

void rotRPS(float angle) {
    rot(aRPS(angle));
}

void tran(int mode,float angle,float velocity) { // The current kinematics incur significant dift when not moving in a cardinal direction (UP DOWN LEFT RIGHT)
    switch(mode) {
        case 1: //tranlate from robots local heading
            float x,y;
            x = cos(((angle+90)/180) * PI);
            y = sin(((angle+90)/180) * PI);

            float m1percent, m2percent, m3percent;

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
            } else {
                motor1.SetPercent(m1percent * motor1multiccw);
            }

            if(m1percent <= 0) {
                motor2.SetPercent(m2percent * motor2multicw);
            } else {
                motor2.SetPercent(m2percent * motor2multiccw);
            }

            if(m1percent <= 0) {
                motor3.SetPercent(m3percent * motor3multicw);
            } else {
                motor3.SetPercent(m3percent * motor3multiccw);
            }

            break;
        case 2:	//tran relative to RPS
            break;
    }
}
/*
void locate(float Xf, float Yf){

    float Xi, Yi, dx, dy, s, angle;

    Xi = RPS.X();
    Yi = RPS.Y();

    dx = Xf -Xi;
    dy = Yf - Yi;
    angle = atan(dy / dx);

    rot(angle);
    s = sqrt(pow(dx,2) + pow(dy,2));
    tran(1, 0, tranpercent);
    //Sleep for some time based on s
}
*/
void matchRPS(float x, float y) {
    float realX,realY, dx, dy, sx ,sy;
    float ax = -90;
    float ay = 0;
    realX = RPS.X();
    realY = RPS.Y();
    if(realX >= 0.0 && realY >= 0.0) {
        dx = ( x - realX ) * rpsratio;
        dy = ( y - realY ) * rpsratio;
        sx = fabs( dx / vx );
        sy = fabs( dy / vy );

        if(dx > 0) {
            ax = -90;
        } else {
            ax = 90;
        }

        if(dy > 0) {
            ay = 0;
        } else {
            ay = 180;
        }

        tran(1,ax,tranpercent);
        Sleep(sx);
        tran(1,ay,tranpercent);
        Sleep(sy);
        stop();
    } else {
        //do nothing
    }
}

float aRPS(float angle) {
    //Calculate the smallest angle between RPS heading and desired angle
    angle = angle + rpsOffset;
    if(angle >= 360.0) {
        angle = angle - 360.0;
    }
    float delta, resa, myrot;
    resa = RPS.Heading();
    if(resa >= 0) {
        delta = angle - resa;
        myrot = delta;
        if(fabs(delta) >= 180) {
            if(delta > 0) {
                myrot = delta - 360;
            } else {
                myrot = delta + 360;
            }
        }
    } else {
        myrot = 0;
    }
    return(myrot);
}

void stop() {
    motor1.Stop();
    motor2.Stop();
    motor3.Stop();
}

void moveToButton(int color) {
    if(color == 1) {//The color is blue is on the left
        tran(1,90,30);
        Sleep(1.0);
        stop();

        tran(1,0,30);
        Sleep(1.0);
        stop();

        tran(1,180,30);
        Sleep(1.0);
        stop();

        tran(1,-90,30);
        Sleep(1.0);
        stop();
    }
    else if (color == 2) {//The color is red is on the right
        tran(1,-90,30);
        Sleep(1.0);
        stop();

        tran(1,0,30);
        Sleep(1.0);
        stop();

        tran(1,180,30);
        Sleep(1.0);
        stop();

        tran(1,90,30);
        Sleep(1.0);
        stop();
    } else {
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
        LCD.WriteRC(TimeNow(),1,1);
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
    int state = 0;

    rV = lineR.Value();
    cV = lineC.Value();
    lV = lineL.Value();

    if(rV > lineRMin && rV < lineRMax) {
        rO = 1;
    }
    else {
        rO = 0;
    }

    if(cV > lineCMin && cV < lineCMax) {
        cO = 1;
    }
    else {
        cO = 0;
    }

    if(lV > lineLMin && lV < lineLMax) {
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
        state = 50;
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

    if(rO + lO == 2) {
        state = 100;
    }
    LCD.WriteRC(state,12,0);
    return state;

}

void servoArm(float degree)
    {
        arm.SetDegree(degree);
    }

void headRPS(float angle) {
    rotRPS(angle);
    Sleep(0.3);
    rotRPS(angle);
    Sleep(0.3);
    rotRPS(angle);
    Sleep(0.3);
}

void xyRPS(float x, float y) {
    matchRPS(x,y);
    Sleep(0.3);
    matchRPS(x,y);
    Sleep(0.3);
    matchRPS(x,y);
    Sleep(0.3);
}
