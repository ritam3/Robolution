#include <Wire.h>
double gX=0,gY=0,gZ=0;
double aX=0,aY=0,aZ=0;
double gx=0,gy=0,gz=0;
double ax=0,ay=0,az=0;
double ret=0;
double rollang=0;
double setpnt = 0;// setpoint
double setpnt1=0;
double timer;
int fflag=0;
int flag=0;
int ena=3,enb=5;
int a1=4,a2=6;// motor pins
int b1=7,b2=2;// motor pins
void motors()
{ //Serial.println("Initialising motors.....");//initialise motors and set every pin high
  pinMode(ena,OUTPUT);
  pinMode(enb,OUTPUT);
  pinMode(a1,OUTPUT);
  pinMode(a2,OUTPUT);
  pinMode(b1,OUTPUT);
  pinMode(b2,OUTPUT);
  digitalWrite(a1,HIGH);
  digitalWrite(a2,HIGH);
  digitalWrite(b1,HIGH);
  digitalWrite(b2,HIGH);
}
/*void stop()
{
  digitalWrite(ena,255);
  digitalWrite(enb,255);
  digitalWrite(a1,HIGH);
  digitalWrite(a2,HIGH);
  digitalWrite(b1,HIGH);
  digitalWrite(b2,HIGH);
}*/
void mpu()
{
  //Serial.println("Initialising mpu.....");
  Wire.beginTransmission(0b1101000);// start communication using address of mpu
  Wire.write(0x6B);                // power management register
  Wire.write(0b00000000);          //sleep=0
  Wire.endTransmission();          //end
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1B);               // gyro config register
  Wire.write(0b00000000);         //sleep
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);              // accelerometer config register
  Wire.write(0b00000000);
  Wire.endTransmission();
}
void gyroget()
{
  Wire.beginTransmission(0b1101000);                         
  Wire.write(0x43);            //access first register of gyro readings                             
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);  // (43-48)                          
  while(Wire.available() < 6);                             
  gX = Wire.read()<<8|Wire.read();                  
  gY = Wire.read()<<8|Wire.read();                  
  gZ = Wire.read()<<8|Wire.read();                 
}
void accelget()
{
  Wire.beginTransmission(0b1101000);                         
  Wire.write(0x3B);     //first register of accl register                                    
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  // AFS_SEL=0 Sensitivity scale factor = 16384
  while(Wire.available()<6);
    aX=(Wire.read()<<8|Wire.read())/16384.0;
    aY=(Wire.read()<<8|Wire.read())/16384.0;
    aZ=(Wire.read()<<8|Wire.read())/16384.0;
}
void callibrate()
{
  //Serial.println("callibrate");
  for(int i=0;i<5000;i++)
  {
    //FS_SEL =0 sensitivity scale factor becomes 131
    gyroget();
    gx+=(gX/131.0);
    gy+=(gY/131.0);
    gz+=(gZ/131.0);
    accelget();
    ax+= (atan((aY)/sqrt(pow(aX,2)+pow(aZ,2))))*57.29; // accl x = taninv(y/x^2 + z^2)
  }
  // take avg of 5000 values
  ax=(ax/5000);
  gx=(gx/5000);
  gy=(gy/5000);
  gz=(gz/5000);  
}
double gxx,grr=0;
double gr()// function to get current gyro read
{
  gyroget();
  timer = millis();//reset timer
  gX/=131.0;
  gxx = (gX - gx);// subtract recent with calculated offset
  return gxx;
}
double accroll=0;
double accl()// function to get current accl reading
{
  accelget();
  accroll = (atan((aY)/sqrt(pow(aX,2)+pow(aZ,2)))*57.29)-ax;//subtract recent with calculated offset
  if((accroll<=360)&&(accroll>=180))
    accroll=accroll-360;
  return accroll;
}
double iniang;
double t,tp=0,dt,f=0;
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  motors();
  mpu();
  timer = millis();//start timer
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);// to indicate access of register succesful
  callibrate();
  Serial.println("Callibrating over");
}
void motrcontrl(double out){
  int vel = abs(out);
  // for forward and backward motion
  if (out>0){  //a1-1,b1-4  
    analogWrite(ena,vel/1.3);
    analogWrite(enb,vel);             
    digitalWrite(a1,LOW);
    digitalWrite(a2,HIGH);
    digitalWrite(b2,HIGH);
    digitalWrite(b1,LOW);
  }else{ 
    analogWrite(ena,vel/1.3);
    analogWrite(enb,vel);      
    digitalWrite(a1,HIGH);
    digitalWrite(a2,LOW);
    digitalWrite(b2,LOW);
    digitalWrite(b1,HIGH);
  }
}
double kp=20,ki=100,kd=1.37;
int ctim,ptim=0;
 double err,rerr,lasterr=0,cumerr=0;
double integral=0;
double pidcmp(double k)// computes pid
{
  Serial.println("pid1");
  double out = 0; 
  ctim = millis();// initialize timer
  double elaptime = (double)(ctim - ptim)/1000.0 ; // calculate diff in time
  err = setpnt - k;                                // calculate diff in angle
  cumerr+= (err)*elaptime;                         
  rerr = (err - lasterr)/elaptime;   
  out = kp*err + ki*cumerr + kd*rerr;
  lasterr = err;
  ptim = ctim;
  if(out>255)// pwm cannot be more than 255
   out = 255;
  if(out<-255)// pwm cannot be more than 255
   out = -255;
  //Serial.println(out);
  motrcontrl(out);
}
unsigned long pwmc1=0;
int ctim1,ptim1=0;
double err1,rerr1,lasterr1=0,cumerr1=0;
int p=0,i=0,d=0;
String ch;
void loop() {
  if(f==0){
  for(int i=0;i<500;i++)// to get more accurate angles
  {
  tp = timer; // to store previous time
  timer = millis(); // to initialize timer
  dt = (timer-tp)/1000; // calculate time difference
  rollang = 0.98*(rollang+ gr()*dt) + 0.02*(accl());//get angle using complementary filter
  }
  f++;
  }
  tp = timer;
  timer = millis();
  dt = (timer-tp)/1000;
  rollang = 0.98*(rollang+ ret*dt) + 0.02*(accl());// to get current angle cf
  Serial.print("ang=");
  Serial.println(rollang);
  pidcmp(rollang);
 if(Serial.available()>0)
  {
    ch = Serial.readString();
    if(ch[0]=='p')
    {
      ch[0]='0';
      kp = ch.toDouble();

    }
    else if(ch[0]=='i')
    {
      ch[0]='0';
      ki = ch.toDouble();

    }
    else if(ch[0]=='d')
    {
      ch[0]='0';
      kd = ch.toDouble();

    }
  }
}
