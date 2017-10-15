#include <Wire.h>
#include <TimerOne.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define    MPU9250_ref_ADDRESS        0x69
#define    MAG_ref_ADDRESS            0x0C
#define    MPU9250_1_ADDRESS          0x68
#define    MAG_1_ADDRESS              0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

RF24 radio(9,8); // CE, CSN

const byte address[6] = "00001";


// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}



// Initial time
long int ti;
volatile bool intFlag=false;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);

  // Reference Sensor initialization

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ref_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ref_ADDRESS,26,0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ref_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ref_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ref_ADDRESS,0x37,0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ref_ADDRESS,0x0A,0x16);

  // Finger Sensor initialization

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_1_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_1_ADDRESS,26,0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_1_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_1_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_1_ADDRESS,0x40,0x04);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_1_ADDRESS,0x0A,0x16);

  //pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt


  // Store initial time
  ti=millis();
}





// Counter
long int cpt=0;

void callback()
{
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
void loop()
{
  while (!intFlag);
  intFlag=false;

  if (digitalWrite)
  {
  // Display time
  //Serial.print (millis()-ti,DEC);
  //Serial.print ("\t");
  }

  // _______________
  // ::: Counter :::

  // Display data counter
  //Serial.print (cpt++,DEC);
  //Serial.print ("\t");



  // ____________________________________
  // :::  accelerometer and gyroscope :::

  // Read accelerometer and gyroscope from reference sensor
  uint8_t Buf[14];
  I2Cread(MPU9250_ref_ADDRESS,0x3B,14,Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  int16_t ax1=-(Buf[0]<<8 | Buf[1]);
  int16_t ay1=-(Buf[2]<<8 | Buf[3]);
  int16_t az1=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx1=-(Buf[8]<<8 | Buf[9]);
  int16_t gy1=-(Buf[10]<<8 | Buf[11]);
  int16_t gz1=Buf[12]<<8 | Buf[13];
  //Serial.print("Reference");
//  // Accelerometer
//  Serial.print (ax1,DEC);
//  Serial.print ("\t");
//  Serial.print (ay1,DEC);
//  Serial.print ("\t");
//  Serial.print (az1,DEC);
//  Serial.print ("\t");
//
//  // Gyroscope
//  Serial.print (gx1,DEC);
//  Serial.print ("\t");
//  Serial.print (gy1,DEC);
//  Serial.print ("\t");
//  Serial.print (gz1,DEC);
//  Serial.print ("\t");
//  // _____________________
  // :::  Magnetometer :::

 // Reference
  // Read register Status 1 and wait for the DRDY: Data Ready

  uint8_t ST1;
  do
  {
    I2Cread(MAG_ref_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ref_ADDRESS,0x03,7,Mag);


  // Create 16 bits values from 8 bits data

  // Magnetometer
  int16_t mx1=-(Mag[3]<<8 | Mag[2]);
  int16_t my1=-(Mag[1]<<8 | Mag[0]);
  int16_t mz1=-(Mag[5]<<8 | Mag[4]);

//  Serial.print (mx1+200,DEC);
//  Serial.print ("\t");
//  Serial.print (my1-70,DEC);
//  Serial.print ("\t");
//  Serial.print (mz1-700,DEC);
//  Serial.print ("\t");
  // Read accelerometer and gyroscope from finger sensor
  uint8_t Buf2[14];
  I2Cread(MPU9250_1_ADDRESS,0x3B,14,Buf2);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  int16_t ax2=-(Buf2[0]<<8 | Buf2[1]);
  int16_t ay2=-(Buf2[2]<<8 | Buf2[3]);
  int16_t az2=Buf2[4]<<8 | Buf2[5];

  // Gyroscope
  int16_t gx2=-(Buf2[8]<<8 | Buf2[9]);
  int16_t gy2=-(Buf2[10]<<8 | Buf2[11]);
  int16_t gz2=Buf2[12]<<8 | Buf2[13];
    // Display values
  //Serial.print("Finger");
//  // Accelerometer
//  Serial.print (ax2,DEC);
//  Serial.print ("\t");
//  Serial.print (ay2,DEC);
//  Serial.print ("\t");
//  Serial.print (az2,DEC);
//  Serial.print ("\t");
////
////  // Gyroscope
//  Serial.print (gx2,DEC);
//  Serial.print ("\t");
//  Serial.print (gy2,DEC);
//  Serial.print ("\t");
//  Serial.print (gz2,DEC);
//  Serial.print ("\t");


  // _____________________
  // :::  Magnetometer :::

 // Reference
  // Read register Status 1 and wait for the DRDY: Data Ready

  uint8_t ST2;
  do
  {
    I2Cread(MAG_1_ADDRESS,0x02,1,&ST2);
  }
  while (!(ST2&0x01));

  // Read magnetometer data
  uint8_t Mag2[7];
  I2Cread(MAG_1_ADDRESS,0x03,7,Mag2);


  // Create 16 bits values from 8 bits data

  // Magnetometer
  int16_t mx2=-(Mag2[3]<<8 | Mag2[2]);
  int16_t my2=-(Mag2[1]<<8 | Mag2[0]);
  int16_t mz2=-(Mag2[5]<<8 | Mag2[4]);


  // Magnetometer
//  Serial.print (mx2+200,DEC);
//  Serial.print ("\t");
//  Serial.print (my2-70,DEC);
//  Serial.print ("\t");
//  Serial.print (mz2-700,DEC);
//  Serial.print ("\t");
  int16_t xfin;
  int16_t yfin;
  int16_t zfin;
  int16_t xref;
  int16_t yref;
  int16_t zref;
  xfin = ax2;
  yfin = ay2;
  zfin = az2;

  xref = ax1;
  yref = ay1;
  zref = az1;

  int16_t ax_total = xfin - yref - 400;
  int16_t ay_total = yfin - xref - 700;
  int16_t az_total = zfin - zref;
  Serial.println(ax_total);
  Serial.println(ay_total);
  Serial.println(az_total);

  if ((ax_total <= -3000) and (az_total <= -600 ))
  {
          //Dedo hacia arriba.
            Serial.print("adelante");
//          const char text[] = "forward";
//          radio.write(&text, sizeof(text));
//          delay(50);
        }
      if ((ax_total >= 3000) and (az_total <= -2000 ))
      {
          // Dedo hacia abajo
            Serial.print("atras");
//          const char text[] = "backward";
//          radio.write(&text, sizeof(text));
//          delay(50);
        }


   if (ay_total >= 6000)
   {
          // Muneca izquierda
            Serial.print("izquierda");
//          const char text[] = "left";
//          radio.write(&text, sizeof(text));
//          delay(50);
                }

    if (ay_total < -6000)
    {
          // muneca derecha
            Serial.print("derecha");
//          const char text[] = "right";
//          radio.write(&text, sizeof(text));
//          delay(50);
      }




//  if ()
//  {
//    }
//  else if ()
//  {const char text[] = "backward";
//  radio.write(&text, sizeof(text));
//  delay(50);
//
//  }
//  else if ()
//  {const char text[] = "left";
//  radio.write(&text, sizeof(text));
//  delay(50);
//
//  }
//  else if ()
//  {const char text[] = "right";
//  radio.write(&text, sizeof(text));
//  delay(50);
//
//  }
//  else if ()
//  {const char text[] = "idle";
//  radio.write(&text, sizeof(text));
//  delay(50);

//  }
  // End of line
  Serial.println("");
 //delay(1000);
 }
