long newLeft, newRight;
#include <MD_REncoder.h>
#include <Encoder.h>
#include <Wire.h>
long tim=0,tim1;
long vel=0,vol=0;;
int vel1=0,vel2=0;
long positionLeft  = -999;
long positionRight = -999;
Encoder knobLeft(3, 4);
Encoder knobRight(2, 7);
double gX=0,gY=0,gZ=0;
double aX=0,aY=0,aZ=0;
double gx=0,gy=0,gz=0;
double ax=0,ay=0,az=0;
double ret=0;
int ffflag=0;
double rollang=0;
double setpnt = 0;// setpoint
double setpnt1=0;
double timer;
int fflag=0;
int flag=0;
int ena=6,enb=5; //ena=3 enb=5
int a1=A0,a2=A1;// motor pins a1=4 a2=6
int b1=A2,b2=A3;// motor pins b1=7 b2 = 2 
int dir=0,sped=0;
void encoder()
{

  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
   /* Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight; */
}}
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
  pinMode(2,INPUT);
  pinMode(4,INPUT);
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
  vel1=vel;
  vel2=vel;
  // for forward and backward motion
  if (out>0){  //a1-1,b1-4  
        //reverse
    if(fflag==1)
        {
          vel2=vel2*0.90;   //left motor speed 
        }
        if(fflag==-1)
        {
          vel1=vel1*0.90;// right mot speed 
        }
    analogWrite(ena,vel1);
    analogWrite(enb,vel2*0.98);             
    digitalWrite(a1,LOW);
    digitalWrite(a2,HIGH);
    digitalWrite(b2,HIGH);
    digitalWrite(b1,LOW);

    
  }
  else{ 
        //forward
        if(fflag==1)
        {
          vel2=vel2*0.90;   //left motor speed 
        }
        if(fflag==-1)
        {
          vel1=vel1*0.90;// right mot speed 
        }
    analogWrite(ena,vel1);
    analogWrite(enb,vel2*0.95);      
    digitalWrite(a1,HIGH);
    digitalWrite(a2,LOW);
    digitalWrite(b2,LOW);
    digitalWrite(b1,HIGH);
  
}
}
double kp1=40,ki1=150,kd1=2;
double kp=30,ki=100,kd=2;
int ctim,ptim=0;
 double err,rerr,lasterr=0,cumerr=0;
double integral=0;
unsigned long pwmc1=0;
int ctim1,ptim1=0;
double err1,rerr1,lasterr1=0,cumerr1=0;
int flg=0;
long pos=0,pos1=0;;

double pidcmp2(double k)// computes pid
{
  //Serial.println("pid2");
  double out = 0; 
  ctim1 = millis();// initialize timer
  double elaptime = (double)(ctim1 - ptim1)/1000.0 ; // calculate diff in time
  err1 = setpnt1 - k;                                // calculate diff in angle
  cumerr1+= (err1)*elaptime;                         
  rerr1 = (err1 - lasterr1)/elaptime; 
  //rerr1 = -ret;    
  out = kp1*err1 + ki1*cumerr1 + kd1*rerr1;
  lasterr1 = err1;
  ptim1 = ctim1;
  if(out>255)// pwm cannot be more than 255
   out = 255;
  if(out<-255)// pwm cannot be more than 255
   out = -255;

if(fflag==1)
{
  vel=vol;
}
if(ffflag==0)
{
if(flag==1)
{
if(vel>800)
{
  setpnt1=0;
}
if(vel<500)
{
  setpnt1=4;
} 
}
else if(flag==-1)
{
if(vel<-800)
{
  setpnt1=0;
}
if(vel>-500)
{
  setpnt1=-4;
} 
}
}
else if(ffflag==1)
{
  if(flag==1)
{
if(vel>800)
{
  setpnt1=0;
}
if(vel<400)
{
  setpnt1=8;
} 
}
else if(flag==-1)
{
if(vel<-800)
{
  setpnt1=0;
}
if(vel>-400)
{
  setpnt1=-8;
} 
}
}

   
   encoder();
tim=millis();
if(flg==0)
{
  tim1=tim;
  pos=newLeft;
  pos1=newRight;
  flg=1;
}
if((tim-tim1)>50)
{
  vel=((newLeft-pos)*10);
  vol=((newRight-pos1)*10);
  flg=0;
}



  motrcontrl(out);
Serial.println(vel);
}
int p=0,i=0,d=0;
char ch;
void loop() {
      encoder();
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
  rollang = 0.98*(rollang+ gr()*dt) + 0.02*(accl());// to get current angle cf
  pidcmp2(rollang);
 if(Serial.available()>0)
  {
    ch = Serial.read();
    if(ch=='B')
    { fflag=0;
      flag=1;
      setpnt1=4;
    }
    else if(ch=='S')
    {fflag=0;
      flag=0;
      setpnt1=0;
    }
    else if(ch=='F')
    {fflag=0;
      flag=-1;
      setpnt=-4;
    }
    else if(ch=='J')
    {
      flag=1;
      fflag=1;
      setpnt=4;
    }
        else if(ch=='H')
    {
      flag=1;
      fflag=-1;
      setpnt=4;
    }
    else if(ch=='I')
    {
      flag=-1;
      fflag=1;
      setpnt=-4;
    }
    else if(ch=='G')
    {
      flag=-1;
      fflag=-1;
      setpnt=-4;
    }
        else if(ch=='X')
    {
   ffflag=1;
    }
            else if(ch=='x')
    {
   ffflag=0;
    }
        
   /* if(ch[0]=='p')
    {
      ch[0]='0';
      kp1 = ch.toDouble();

    }
    else if(ch[0]=='i')
    {
      ch[0]='0';
      ki1 = ch.toDouble();

    }
    else if(ch[0]=='d')
    {
      ch[0]='0';
      kd1 = ch.toDouble();

    }
    else if(ch[0]=='s')
    {
      ch[0]='0';
      setpnt1 = ch.toDouble();
      flag=1;
    }
    else if(ch[0]=='k')
    {
      ch[0]='0';
      setpnt1 = -(ch.toDouble());
      flag=1;
    }
    else if(ch[0]=='o')
    {
      setpnt1 =0;
      flag=0;
      fflag=0;  
    }*/
  }
}
