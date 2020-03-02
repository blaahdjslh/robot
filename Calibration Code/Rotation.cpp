#define m1mp  1.0   //Minimum Rotation %
#define m2mp  1.0   //Minimum Rotation %
#define m3mp  1.0   //Minimum Rotation %
#define m1s   1.0   //Slope of Rotation %
#define m2s   1.0   //Slope of Rotation %
#define m3s   1.0   //Slope of Rotation %


//Friction Calculations
#define CoMx  0.0   //Center of Mass of Robot in X
#define CoMy  0.0   //Center of Mass of Robot in Y
#define CoeFS 1.0   //CoeF for radial displaecment
#define CoeFR 1.0   //CoeF for radial rotation

//Battery Voltage Compensation compBV() = bVx2 * pow(batV,2) + bVx1 * batV + bvx0
#define bVx2  1.0   //Battery Voltage Coe^2
#define bvx1  1.0   //Battery Voltage Coe^1
#define bvx0  1.0   //Battery Voltage Coe 

//Compensation Functions
float compBV();
float calcF(int wheel, float percent, float angle);

//Movement Functions
//a angle
//d distance
//v velocity
void tran(float a, float v);
void tranD(float a, float d, float v);
void tranRPS(float a, float v);
void tranDRPS(float a, float d, float v);
void jrot(float a, float v);
void jrotRPS(float a, float v);

//Tuning Functions
void motorcalibrate;
void translateCalibrate;
void rotateCalibrate;
void bVspeedCalibrate;

int main() {
  //Test this Stuff dude
}

float compBV() {
  float v, CoefficientP;
  v = Battery.Voltage();
  CoefficientP = bVx2 * pow(v,2) + bVx1 * v + bVx0;
}
float calcF(int wheel, float percent, float rotation) {
    float friction;
    
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
    
}
void tranRPS(float a, float v) {
    
}
void tranDRPS(float a, float d, float v) {
    
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
    
}
