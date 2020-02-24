#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHMotor.h>

FEHMotor    right_motor(FEHMotor::Motor0,9.0);
FEHMotor    left_motor(FEHMotor::Motor1,9.0);

AnalogInputPin  rightphoto(FEHIO::P0_0);
AnalogInputPin  centerphoto(FEHIO::P0_1);
AnalogInputPin  leftphoto(FEHIO::P0_2);

int main(void)
{
    float rV,cV,lV;
    int rO,cO,lO;
    int state;
    while(true) {
        rV = rightphoto.Value();
        cV = centerphoto.Value();
        lV = leftphoto.Value();
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

        if(rO + cO == 2 && lO == 0) {
            state = 1;
        }
        if(rO == 1 && cO + lO == 0) {
            state = 1;
        }
        if(rO + cO + lO == 3) {
            state = 2;
        }
        if(cO == 1 && rO + lO == 0) {
            state = 2;
        }
        if(lO + cO == 2 && rO == 0) {
            state = 3;
        }
        if(lO == 1 && cO + rO == 0) {
            state = 3;
        }

        switch(state) {
            case 1:
                right_motor.SetPercent(5);
                left_motor.SetPercent(30);
                break;
            case 2:
                right_motor.SetPercent(25);
                left_motor.SetPercent(25);
                break;
            case 3:
                right_motor.SetPercent(30);
                left_motor.SetPercent(5);
                break;
        }

        LCD.WriteRC(state,1,1);
    }
}
