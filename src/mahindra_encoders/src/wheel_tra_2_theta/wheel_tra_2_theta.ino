// Keep the circuit stable on a surface for atleast 10 seconds to set offset

#include <Wire.h>  
#include  <LSM6.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

long int sampleNum = 1000; 
float dc_offset = 0;
const byte address = 0xf0f0f0f0c3;
long int predata[5] = {0,0,0,0,0};

RF24 radio(9, 10); // CE, CSN Pins

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

LSM6 imu;

char report[80];

void setup()
{
  delay(5000);
  
  Serial.begin(57600);
  Wire.begin();
  Serial.println("Code has started");
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  
  imu.writeReg(LSM6::CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
  imu.writeReg(LSM6::CTRL1_XL, 0x70); // Acc = 833Hz (High-Performance mode)
  imu.writeReg(LSM6::INT1_CTRL, 0x01); // Acc Data Ready interrupt on INT1
  imu.writeReg(LSM6::CTRL10_C, 0x38); // Gyro X, Y, Z axes enabled
  imu.writeReg(LSM6::CTRL2_G, 0x7C);// Gyro = 833Hz (High-Performance mode) and 2000dps
  imu.writeReg(LSM6::INT2_CTRL, 0x02); // Gyro Data Ready interrupt on INT1
  
  for(int n=0;n<sampleNum;n++)
  { 
    imu.writeReg(LSM6::STATUS_REG, 0x0B); //this hex number represents the status register
    imu.read(); 
    snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d", imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z);
    dc_offset += imu.g.z;
  } 
  
  dc_offset =dc_offset/sampleNum;
  Serial.println(dc_offset); 
  
  radio.begin();
  radio.openWritingPipe(address);
  
  //radio.setPALevel(RF24_PA_MIN);

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  
  radio.stopListening();
}

void loop()
{ 
  float yaw0;
  float yaw, gz;
  double aix, aiy, aiz;
  double gix, giy, giz;
  unsigned long microsNow;
  double ai, cos_theta_z, sin_theta_z;
    
  imu.writeReg(LSM6::STATUS_REG, 0x0B); //this hex number represents the status register
  imu.read();
  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d", imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z);
  
  aix = imu.a.x;
  aiy = imu.a.y;
  aiz = imu.a.z;
  gix = imu.g.x;
  giy = imu.g.y;
  giz = 1.126*(imu.g.z - dc_offset);

  ai = sqrt(sq(aix)+sq(aiy)+sq(aiz));
  sin_theta_z = (-aiz/ai);
  cos_theta_z = sqrt(1 - (sin_theta_z*sin_theta_z)) ;
    
  Serial.print(cos_theta_z);
  giz = giz / cos_theta_z;

  
///////////////////   RUNNING AVERAGE   ////////////
  long int sum = 0, avggz;
  int i;
  for (i = 0; i<4; i++)
  {
      predata[i] = predata[i+1];
  }

   predata[4] = giz;

   for(i = 0; i<5; i++)
   {
    sum = sum + predata[i];
   }

   avggz = sum/5.0;
  
/////////////////////////////////////////////
  
  // convert from raw data to gravity and degrees/second units
  yaw = convertRawGyro(avggz);
  Serial.println(yaw);
  yaw0 = yaw/6.0;

  float data[2];
  data[0] = 2;    // 1 for left and 2 for right
  data[1] = yaw0;
  //delay(10);
  bool ok = radio.write(&data, sizeof(data));
  if (ok)
      Serial.println("ok...\n\r");
  else
      Serial.println("failed.\n\r");
  
 
 }

float convertRawAcceleration(int aRaw) 
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw)
{ 
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  
  float g = (gRaw * 2000.0) / 32768.0;
  return g;
}

    

 
