#define m1mp  1.0   //Minimum Rotation %
#define m2mp  1.0   //Minimum Rotation %
#define m3mp  1.0   //Minimum Rotation %
#define m1s   1.0   //Slope of Rotation %
#define m2s   1.0   //Slope of Rotation %
#define m3s   1.0   //Slope of Rotation %


//Friction Calculations
#define CoMx  0.0   //Center of Mass of Robot in X
#define CoMy  0.0   //Center of Mass of Robot in Y
#define CoeFS 1.0   //CoeF for radial displacement
#define CoeFR 1.0   //CoeF for radial rotation

//Battery Voltage Compensation compBV() = bVx2 * pow(batV,2) + bVx1 * batV + bvx0
#define bVx2  1.0   //Battery Voltage Coe^2
#define bvx1  1.0   //Battery Voltage Coe^1
#define bvx0  1.0   //Battery Voltage Coe 

//velocity in x and y compensation
#define vx    6.1   //velocity in in/s in the x direction
#define vy    5.6   //velocity in in/s in the y direction

//Compensation Functions
float compBV();                                       //done
float calcFR(int wheel, float percent, float angle);  //WIP
float calcFS(int wheel, float percent, float angle);  //WIP
void  aRPS(float a);                                  //done

//Movement Functions
//a angle
//d distance
//v velocity
void tran(float a, float v);                          //done
void tranD(float a, float d, float v);                //done
void tranRPS(float a, float v);                       //done
void tranDRPS(float a, float d, float v);             //VALIDATE
void jrot(float a, float v);                          //done
void jrotRPS(float a, float v);                       //done
void mpRPS(float v);                                  //WIP

//Tuning Functions
void motorcalibrate();                                //WIP
void translateCalibrate();                            //WIP
void rotateCalibrate();                               //WIP
void bVspeedCalibrate();                              //WIP

int main() {
  motorcalibrate();
  translateCalibrate();
  rotateCalibrate();
  bVspeedCalibrate();
}

float compBV() {
  float v, CoefficientP;
  v = Battery.Voltage();
  CoefficientP = bVx2 * pow(v,2) + bVx1 * v + bVx0;
  return CoefficientP;
}

float calcFR(int wheel, float percent, float rotation) {
    float friction;
    switch(wheel) {
      case 1: //Back Wheel
        break;
      case 2: //Left Wheel
        break;
      case 3: //Right Wheel
        break;
      default://Non-existent Wheel
        return 0;
        break;
    }  
}

float calcFS(int wheel, float percent, float rotation) {
    float friction;
    switch(wheel) {
      case 1: //Back Wheel
        break;
      case 2: //Left Wheel
        break;
      case 3: //Right Wheel
        break;
      default://Non-existent Wheel
        return 0;
        break;
    }  
}

void tran(float a, float v) {
  float x,y;  //x and y amounts
  float m1p, m2p, m3p;  //motorpercents
  x = cos(((a+90)/180) * PI);
  y = sin(((a+90)/180) * PI);
  
  m1p = (-1 * x          + 0 * y)           * v;
  m2p = (cos(PI/3) * x   + sin(PI/3) * y)   * v;
  m3p = (cos(PI/3) * x   - sin(PI/3) * y)   * v;

  
  if(m1p <= 0) {
    motor1.SetPercent(m1p * motor1multicw);
  } else {
    motor1.SetPercent(m1p * motor1multiccw);
  }

  if(m2p <= 0) {
    motor2.SetPercent(m2p * motor2multicw);
  } else {
    motor2.SetPercent(m2p * motor2multiccw);
  }

  if(m3p <= 0) {
    motor3.SetPercent(m3p * motor3multicw);
  } else {
    motor3.SetPercent(m3p * motor3multiccw);
  }
}
void tranD(float a, float d, float v) {
    float x,y;
    float tx,ty,tt;
    x = cos(((a+90)/180) * PI);
    y = sin(((a+90)/180) * PI);
    x = x * d;
    y = y * d;
    tx = x / (vx/v);
    ty = y / (vy/v);
    tt = tx + ty;
    tran(a,v);
    Sleep(tt);
    StopAll();
}
void tranRPS(float a, float v) {
    tran(aRPS(a),v);
}
void tranDRPS(float a, float d, float v) {
    float x,y;
    float tx,ty,tt;
    x = cos(((aRPS(a)+90)/180) * PI);
    y = sin(((aRPS(a)+90)/180) * PI);
    x = x * d;
    y = y * d;
    tx = x / (vx/v);
    ty = y / (vy/v);
    tt = tx + ty;
    tran(aRPS(a),v);
    Sleep(tt);
    StopAll();
}
void jrot(float a, float v) {
    float d,s;  //direction and angle
    if(a >= 0 ){
      d = -1.0;
    }   else if (a < 0) {
      d = 1.0;
    }
    s = fabs(a * yodersConstant * (rotpercent/v));

    motor1.SetPercent(v * d);
    motor2.SetPercent(v * d);
    motor3.SetPercent(v * d);
    //LCD.WriteRC(s,13,0);
    Sleep(s);
    StopAll();
}
void jrotRPS(float a, float v) {
    jrot(aRPS(a),v);
}

void mpRPS() {
  
}

float aRPS(float a) {
    //Calculate the smallest angle between RPS heading and desired angle
    float rpsa, accw, acw;
    rpsa  = RPS.Heading();
    accw  = fabs( a - rpsa);
    acw   = accw - 360;

    if(accw <= 180) {
      return accw;
    } else {
      return acw;
    }
}
