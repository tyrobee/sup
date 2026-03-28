// VTOL Flight Controller (SBUS + PID + Filters + OneShot125)

#include <Wire.h>
#include <HardwareSerial.h>

#define MPU 0x68

#define M1 3
#define M2 5
#define M3 6
#define M4 9
#define M5 10

HardwareSerial SBUS(1);

uint16_t sbusChannels[16];
bool sbusFailSafe = false;

float dt;
unsigned long lastTime;

float ax, ay, az;
float gx, gy, gz;
float gx_off=0, gy_off=0, gz_off=0;

float gx_f=0, gy_f=0, gz_f=0;

float nx1=0,nx2=0,ny1=0,ny2=0;
float a0,a1,a2,b1,b2;

float roll=0,pitch=0,yaw=0;

float kp_a=4.0, ki_a=0.02;
float kp_r=0.12, ki_r=0.0, kd_r=0.003;

float roll_i_a=0, pitch_i_a=0;
float roll_i_r=0, pitch_i_r=0, yaw_i_r=0;

float d_roll_f=0,d_pitch_f=0,d_yaw_f=0;

float roll_sp,pitch_sp,yaw_sp,throttle;

bool armed=false;
float mode=1.0;
float modeTarget=1.0;

void initNotch(float freq,float Q){
  float w0 = 2.0*3.14159*freq/500.0;
  float alpha = sin(w0)/(2*Q);

  float b0=1;
  float b1n=-2*cos(w0);
  float b2n=1;
  float a0n=1+alpha;
  float a1n=-2*cos(w0);
  float a2n=1-alpha;

  a0=b0/a0n;
  a1=b1n/a0n;
  a2=b2n/a0n;
  b1=a1n/a0n;
  b2=a2n/a0n;
}

float notch(float x){
  float y = a0*x + a1*nx1 + a2*nx2 - b1*ny1 - b2*ny2;
  nx2=nx1; nx1=x;
  ny2=ny1; ny1=y;
  return y;
}

void readSBUS(){
  static uint8_t buf[25];
  if(SBUS.available()>=25){
    SBUS.readBytes(buf,25);

    if(buf[0]==0x0F){
      sbusChannels[0]=((buf[1]|buf[2]<<8)&0x07FF);
      sbusChannels[1]=((buf[2]>>3|buf[3]<<5)&0x07FF);
      sbusChannels[2]=((buf[3]>>6|buf[4]<<2|buf[5]<<10)&0x07FF);
      sbusChannels[3]=((buf[5]>>1|buf[6]<<7)&0x07FF);
      sbusChannels[4]=((buf[6]>>4|buf[7]<<4)&0x07FF);
      sbusChannels[5]=((buf[7]>>7|buf[8]<<1|buf[9]<<9)&0x07FF);

      sbusFailSafe = buf[23] & (1<<3);
    }
  }
}

void setup(){
  Wire.begin();
  Serial.begin(115200);

  SBUS.begin(100000, SERIAL_8E2, 16, -1);

  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);
  pinMode(M5,OUTPUT);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateIMU();
  initNotch(80,5);

  lastTime=micros();
}

void loop(){
  unsigned long now=micros();
  dt=(now-lastTime)/1000000.0;
  lastTime=now;

  readRC();
  readIMU();
  filterIMU();
  computeAngles();

  if(mode < modeTarget) mode += 0.02;
  else if(mode > modeTarget) mode -= 0.02;

  mode = constrain(mode,0,1);

  angleLoop();
  rateLoop();
}

void readRC(){
  readSBUS();

  roll_sp  = map(sbusChannels[0],172,1811,-30,30);
  pitch_sp = map(sbusChannels[1],172,1811,-30,30);
  throttle = map(sbusChannels[2],172,1811,1000,2000);
  yaw_sp   = map(sbusChannels[3],172,1811,-100,100);

  static bool lastArm=false;
  bool armSwitch = sbusChannels[4] > 1500;

  if(armSwitch && !lastArm && throttle<1100){
    armed = !armed;
  }
  lastArm = armSwitch;

  bool modeSwitch = sbusChannels[5] > 1500;
  modeTarget = modeSwitch ? 1.0 : 0.0;

  if(sbusFailSafe){
    armed=false;
  }
}

void calibrateIMU(){
  for(int i=0;i<500;i++){
    readIMU();
    gx_off+=gx; gy_off+=gy; gz_off+=gz;
    delay(2);
  }
  gx_off/=500; gy_off/=500; gz_off/=500;
}

void readIMU(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);

  ax=Wire.read()<<8|Wire.read();
  ay=Wire.read()<<8|Wire.read();
  az=Wire.read()<<8|Wire.read();

  Wire.read();Wire.read();

  gx=(Wire.read()<<8|Wire.read())/131.0-gx_off;
  gy=(Wire.read()<<8|Wire.read())/131.0-gy_off;
  gz=(Wire.read()<<8|Wire.read())/131.0-gz_off;
}

void filterIMU(){
  float a=0.7;
  gx_f=a*gx_f+(1-a)*gx;
  gy_f=a*gy_f+(1-a)*gy;
  gz_f=a*gz_f+(1-a)*gz;

  gx_f=notch(gx_f);
  gy_f=notch(gy_f);
  gz_f=notch(gz_f);
}

void computeAngles(){
  float accRoll=atan2(ay,az)*57.3;
  float accPitch=atan2(-ax,az)*57.3;

  roll=0.98*(roll+gx_f*dt)+0.02*accRoll;
  pitch=0.98*(pitch+gy_f*dt)+0.02*accPitch;
  yaw+=gz_f*dt;
}

float roll_rate_sp,pitch_rate_sp,yaw_rate_sp;

void angleLoop(){
  float er=roll_sp-roll;
  float ep=pitch_sp-pitch;

  roll_i_a+=er*dt;
  pitch_i_a+=ep*dt;

  roll_rate_sp=kp_a*er+ki_a*roll_i_a;
  pitch_rate_sp=kp_a*ep+ki_a*pitch_i_a;
  yaw_rate_sp=yaw_sp;
}

float roll_prev=0,pitch_prev=0,yaw_prev=0;

void rateLoop(){
  float er=roll_rate_sp-gx_f;
  float ep=pitch_rate_sp-gy_f;
  float ey=yaw_rate_sp-gz_f;

  roll_i_r+=er*dt;
  pitch_i_r+=ep*dt;
  yaw_i_r+=ey*dt;

  float dr=(er-roll_prev)/dt;
  float dp=(ep-pitch_prev)/dt;
  float dy=(ey-yaw_prev)/dt;

  d_roll_f=0.7*d_roll_f+0.3*dr;
  d_pitch_f=0.7*d_pitch_f+0.3*dp;
  d_yaw_f=0.7*d_yaw_f+0.3*dy;

  float r=kp_r*er+ki_r*roll_i_r+kd_r*d_roll_f;
  float p=kp_r*ep+ki_r*pitch_i_r+kd_r*d_pitch_f;
  float y=kp_r*ey+ki_r*yaw_i_r+kd_r*d_yaw_f;

  roll_prev=er;
  pitch_prev=ep;
  yaw_prev=ey;

  mix(r,p,y);
}

void writeOneshot(int pin,int val){
  int pulse = (val-1000)/2 + 125;
  digitalWrite(pin,HIGH);
  delayMicroseconds(pulse);
  digitalWrite(pin,LOW);
}

void mix(float r,float p,float y){
  if(!armed){
    writeOneshot(M1,1000);
    writeOneshot(M2,1000);
    writeOneshot(M3,1000);
    writeOneshot(M4,1000);
    writeOneshot(M5,1000);
    return;
  }

  float m1=throttle+p+r-y;
  float m2=throttle+p-r+y;
  float m3=throttle-p+r+y;
  float m4=throttle-p-r-y;
  float m5=throttle;

  m1*=mode; m2*=mode; m3*=mode; m4*=mode;
  m5*=(1-mode);

  m1=constrain(m1,1000,2000);
  m2=constrain(m2,1000,2000);
  m3=constrain(m3,1000,2000);
  m4=constrain(m4,1000,2000);
  m5=constrain(m5,1000,2000);

  writeOneshot(M1,m1);
  writeOneshot(M2,m2);
  writeOneshot(M3,m3);
  writeOneshot(M4,m4);
  writeOneshot(M5,m5);
}
