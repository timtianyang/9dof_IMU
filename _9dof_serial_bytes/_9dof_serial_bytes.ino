#include <Wire.h>
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2



#define ACCEL_X_OFFSET 1
#define ACCEL_Y_OFFSET 0
#define ACCEL_Z_OFFSET -13
#define GYRO_X_OFFSET 43
#define GYRO_Y_OFFSET -3
#define GYRO_Z_OFFSET -40
#define MAGN_X_MIN ((float) -565)
#define MAGN_X_MAX ((float) 601)
#define MAGN_Y_MIN ((float) -660)
#define MAGN_Y_MAX ((float) 515)
#define MAGN_Z_MIN ((float) -163)
#define MAGN_Z_MAX ((float) 934)
/*

sensor2

#define ACCEL_X_OFFSET -4
#define ACCEL_Y_OFFSET 0
#define ACCEL_Z_OFFSET 4
#define GYRO_X_OFFSET -27
#define GYRO_Y_OFFSET 19
#define GYRO_Z_OFFSET 2
#define MAGN_X_MIN ((float) -525)
#define MAGN_X_MAX ((float) 697)
#define MAGN_Y_MIN ((float) -584)
#define MAGN_Y_MAX ((float) 663)
#define MAGN_Z_MIN ((float) -996)
#define MAGN_Z_MAX ((float) 182)

sensor1

#define ACCEL_X_OFFSET -3
#define ACCEL_Y_OFFSET -4
#define ACCEL_Z_OFFSET 4
#define GYRO_X_OFFSET -15
#define GYRO_Y_OFFSET 27
#define GYRO_Z_OFFSET -2
#define MAGN_X_MIN ((float) -525)
#define MAGN_X_MAX ((float) 697)
#define MAGN_Y_MIN ((float) -584)
#define MAGN_Y_MAX ((float) 663)
#define MAGN_Z_MIN ((float) -996)
#define MAGN_Z_MAX ((float) 182)
*/

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))
#define MAGN_X_MIN ((float) -494)
#define MAGN_X_MAX ((float) 647)
#define MAGN_Y_MIN ((float) -573)
#define MAGN_Y_MAX ((float) 605)
#define MAGN_Z_MIN ((float) -1022)
#define MAGN_Z_MAX ((float) 138)

//from sensors
byte accel[6];
byte gyro[6];
byte magnetom[6];
//calculated and calibrated values
int accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz;
//calibrated bytes to send out
//byte caccel[];

long currentT=0;
char dt=0;
int Xmin=0,Xmax=0,Ymin=0,Ymax=0,Zmin=0,Zmax=0;


void setup()
{
  // Init serial output
  Serial.begin(115200);
  
  // Init status LED
  pinMode (13, OUTPUT);
  digitalWrite(13, HIGH);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  delay(20);  // Give sensors enough time to collect data
  read_sensors();
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  delay(300);
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  delay(300);
  digitalWrite(13, LOW);
  currentT = millis();
}
void loop(){
  
 /* while(true){
     
                  digitalWrite(13, HIGH);
                    read_sensors();
                    Serial.print((float)magx);
                    Serial.print(",");
                    Serial.print((float)magy);
                     Serial.print(",");
               
                    Serial.println((float)magz);
                    digitalWrite(13, LOW);                   
                    delay(20);
                    
         
     
  
  }
 */
  
  if(Serial.available()){
      if(Serial.read()==0x31){          
           digitalWrite(13, HIGH);
          // dt=millis()-currentT;
          // currentT = millis();           
           output_sensors_bytes();
                          
           read_sensors();
           compensate_sensor_errors();
           digitalWrite(13, LOW);          
      }
      /*
     else{
         while(true){
           Read_Magn();
           if(magx>Xmax){
               Xmax=magx;
           }
           else if(magx<Xmin){
               Xmin=magx;
           }
           if(magy>Ymax){
               Ymax=magy;
           }
           else if(magy<Ymin){
               Ymin=magy;
           }
           if(magz>Zmax){
               Zmax=magz;
           }
           else if(magz<Zmin){
               Zmin=magz;
           }
           Serial.print(Xmax);Serial.print(",");
           Serial.print(Xmin);Serial.print(";");  
           Serial.print(Ymax);Serial.print(",");
           Serial.print(Ymin);Serial.print(";");
           Serial.print(Zmax);Serial.print(",");
           Serial.print(Zmin);Serial.println(";");
           delay(100);
         }
      } 
      */
  } 
}

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}


void I2C_Init()
{
  Wire.begin();  
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x01);  // /Configure to 10bit resolution at+-4g  right justified
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x0A);  // Set to 100Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
 // byte buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
   accel[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
   accx = ((((int) accel[1]) << 8) | accel[0]);  // X axis (internal sensor y axis)
   accy = ((((int) accel[3]) << 8) | accel[2]);  // Y axis (internal sensor x axis)
   accz = ((((int) accel[5]) << 8) | accel[4]);  // Z axis (internal sensor z axis)
   
  Wire.endTransmission();

}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
   magnetom[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
    magx = ((((int) magnetom[0]) << 8) | magnetom[1]);  // X axis (internal sensor -y axis)
    magy = ((((int) magnetom[4]) << 8) | magnetom[5]);  // Y axis (internal sensor -x axis)
    magz = ((((int) magnetom[2]) << 8) | magnetom[3]);  // Z axis (internal sensor -z axis)
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x19);   //2000dps, 188HZ LP bandwdith
  Wire.endTransmission();
  delay(5);
  
 

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x03);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    gyro[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  gyrox =  ((((int) gyro[0]) << 8) | gyro[1]);    // X axis (internal sensor -y axis)
  gyroy = ((((int) gyro[2]) << 8) | gyro[3]);    // Y axis (internal sensor -x axis)
  gyroz =  ((((int) gyro[4]) << 8) | gyro[5]);    // Z axis (internal sensor -z axis)
}
void compensate_sensor_errors() {
    // Compensate accelerometer error
  accx+=ACCEL_X_OFFSET;
  accy+=ACCEL_Y_OFFSET;
  accz+=ACCEL_Z_OFFSET;
  gyrox+=GYRO_X_OFFSET;
  gyroy+=GYRO_Y_OFFSET;
  gyroz+=GYRO_Z_OFFSET;
 // magx = ((magx - MAGN_X_OFFSET));
 // magy = ((magy - MAGN_Y_OFFSET));
 // magz = ((magz - MAGN_Z_OFFSET));
  
  
  accel[0]=accx>>8; 
  accel[1]=accx&0x00FF;
  accel[2]=accy>>8;
  accel[3]=accy&0x00FF;
  accel[4]=accz>>8;
  accel[5]=accz&0x00FF;
  gyro[0]=gyrox>>8;
  gyro[1]=gyrox&0x00FF;
  gyro[2]=gyroy>>8;
  gyro[3]=gyroy&0x00FF;
  gyro[4]=gyroz>>8;
  gyro[5]=gyroz&0x00FF;
  magnetom[0]=magx>>8;
  magnetom[1]=magx&0x00FF;
  magnetom[2]=magy>>8;
  magnetom[3]=magy&0x00FF;
  magnetom[4]=magz>>8;
  magnetom[5]=magz&0x00FF;
  
    // Compensate magnetometer error

  
}
void output_sensors_bytes()
{
Serial.write(accel[0]);//Serial.print(",");
  Serial.write(accel[1]);//Serial.print(",");
  Serial.write(accel[2]);//Serial.print(",");
  Serial.write(accel[3]);//Serial.print(",");
  Serial.write(accel[4]);//Serial.print(",");
  Serial.write(accel[5]);//Serial.print(",");

  Serial.write(gyro[0]); //Serial.print(",");
  Serial.write(gyro[1]);// Serial.print(",");
  Serial.write(gyro[2]);//Serial.print(",");
  Serial.write(gyro[3]);// Serial.print(",");
  Serial.write(gyro[4]); //Serial.print(",");
  Serial.write(gyro[5]);//Serial.print(",");
  
  Serial.write(magnetom[0]);//Serial.print(",");
  Serial.write(magnetom[1]); //Serial.print(",");
  Serial.write(magnetom[2]);// Serial.print(",");
  Serial.write(magnetom[3]);//Serial.print(",");
  Serial.write(magnetom[4]); //Serial.print(",");
  Serial.write(magnetom[5]); //Serial.println("  ");
  
  //Serial.write(dt); 
 /* 
   Serial.print(accx);Serial.print(",");
   Serial.print(accy);Serial.print(",");
   Serial.print(accz);Serial.print(",");
   Serial.print(gyrox);Serial.print(",");
   Serial.print(gyroy);Serial.print(",");
   Serial.print(gyroz);Serial.print(",");
   Serial.print(magx);Serial.print(",");
   Serial.print(magy);Serial.print(",");
   Serial.println(magz);
   */
}
void outputMag(){
   Serial.flush();
   Serial.print(magx);Serial.print(",");
   Serial.print(magy);Serial.print(",");
   Serial.print(magz);Serial.println();
}

