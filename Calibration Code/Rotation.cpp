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
    
}
void tran(float a, float v) {
    
}
void tranD(float a, float d, float v) {
    
}
void tranRPS(float a, float v) {
    
}
void tranDRPS(float a, float d, float v) {
    
}
void jrot(float a, float v) {
    
}
void jrotRPS(float a, float v) {
    
}
