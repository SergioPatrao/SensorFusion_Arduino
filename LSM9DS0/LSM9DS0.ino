/*
  Sérgio Patrão
  12/03/2015
  Coimbra, Portugal
*/

/*
  This sketch aims to configure the LSM9DS0 (Adafruit Breakout Board) to transmit inertial sensors data.

  The wire library is responsible for the I2C communication between the LSM9DS0 and the Asduino.


  Tested with:

  -Arduino Leonardo

  Connections:

  Arduino ---> LSM9DS0
     5V ---> VIN
    GND ---> GND
    SCL ---> SCL
    SDA ---> SDA
      2 ---> DRDY  Gyroscope Interrupt
      3 ---> INT1  Accelerometer Interrupt
      4 ---> INT2  Magnetometer Interrupt

*/

#include <Wire.h>

/*
  REGISTER MAP
*/
//Gyroscope
//Reserved --> 0x00 - 0x0E
#define WHO_AM_I_G			                0x0F //default: 11010100 r --> Device ID
//Reserved --> 0x10 - 0x1F
#define CTRL_REG1_G                     0x20 //default: 00000111 rw --> Data Rate, Bandwidth, Power, Enable Axis
#define CTRL_REG2_G                     0x21 //default: 00000000 rw --> HPF Configuration
#define CTRL_REG3_G                     0x22 //default: 00000000 rw --> Interrupt Configurations
#define CTRL_REG4_G                     0x23 //default: 00000000 rw --> Set Scales
#define CTRL_REG5_G                     0x24 //default: 00000000 rw --> Enable FIFO/HPF
#define REFERENCE_G                     0x25 //default: 00000000 rw --> Reference Value for Interrupt Generation
//Reserved --> 0x26
#define STATUS_REG_G                    0x27 //default: OUTPUT r --> Read to know if there is new data available or if it has overwritten previous data
#define OUT_X_L_G                       0x28 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_X_H_G                       0x29 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_L_G                       0x2A //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_H_G                       0x2B //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_L_G                       0x2C //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_H_G                       0x2D //default: OUTPUT r --> Data (Two's Complement)
#define FIFO_CTRL_REG_G                 0x2E //default: 00000000 rw --> Set Fifo Mode
#define FIFO_SRC_REG_G                  0x2F //default: OUTPUT r --> Fifo Status
#define INT1_CFG_G                      0x30 //default: 00000000 rw --> Set Combination of Interrupts To values higher or Lower than Threshold 
#define INT1_SRC_G                      0x31 //default: OUTPUT r  --> Verify Source of interrupt
#define INT1_TSH_XH_G                   0x32 //default: 00000000 rw --> Set Threshold
#define INT1_TSH_XL_G                   0x33 //default: 00000000 rw --> Set Threshold
#define INT1_TSH_YH_G                   0x34 //default: 00000000 rw --> Set Threshold
#define INT1_TSH_YL_G                   0x35 //default: 00000000 rw --> Set Threshold
#define INT1_TSH_ZH_G                   0x36 //default: 00000000 rw --> Set Threshold
#define INT1_TSH_ZL_G                   0x37 //default: 00000000 rw --> Set Threshold
#define INT1_DURATION_G                 0x38 //default: 00000000 rw --> Duration of Interrupt

//Accelerometer/Magnetometer
//Reserved --> 0x00 - 0x04
#define OUT_TEMP_L_XM                   0x05 //default: OUTPUT r --> Temperature Sensor Data
#define OUT_TEMP_H_XM                   0x06 //default: OUTPUT r --> Temperature Sensor Data
#define STATUS_REG_M                    0x07 //default: OUTPUT r --> Read to know if there is new data available or if it has overwritten previous data
#define OUT_X_L_M                       0x08 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_X_H_M                       0x09 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_L_M                       0x0A //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_H_M                       0x0B //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_L_M                       0x0C //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_H_M                       0x0D //default: OUTPUT r --> Data (Two's Complement)
//Reserved --> 0x0E
#define WHO_AM_I_XM			                0x0F //default: 01001001 r --> Device ID
//Reserved --> 0x10 - 0x11
#define INT_CTRL_REG_M                  0x12 //default: 11101000 rw --> Enable Interruptions and respective mode (latch or not)
#define INT_SRC_REG_M                   0x13 //default: OUTPUT r --> Verify Source of Interrupt
#define INT_THS_L_M                     0x14 //default: 00000000 rw --> Set Threshold       
#define INT_THS_H_M                     0x15 //default: 00000000 rw --> Set Threshold
#define OFFSET_X_L_M                    0x16 //default: 00000000 rw --> Offset
#define OFFSET_X_H_M                    0x17 //default: 00000000 rw --> Offset
#define OFFSET_Y_L_M                    0x18 //default: 00000000 rw --> Offset
#define OFFSET_Y_H_M                    0x19 //default: 00000000 rw --> Offset
#define OFFSET_Z_L_M                    0x1A //default: 00000000 rw --> Offset
#define OFFSET_Z_H_M                    0x1B //default: 00000000 rw --> Offset
#define REFERENCE_X                     0x1C //default: 00000000 rw --> Reference for HPF Data
#define REFERENCE_Y                     0x1D //default: 00000000 rw --> Reference for HPF Data
#define REFERENCE_Z                     0x1E //default: 00000000 rw --> Reference for HPF Data
#define CTRL_REG0_XM                    0x1F //default: 00000000 rw --> Enable FIFO/HPF
#define CTRL_REG1_XM                    0x20 //default: 00000111 rw --> Data Rate, Enable Axis Acc
#define CTRL_REG2_XM                    0x21 //default: 00000000 rw --> Set Acc Scale
#define CTRL_REG3_XM                    0x22 //default: 00000000 rw --> Type of Interruption on INT1_XM
#define CTRL_REG4_XM                    0x23 //default: 00000000 rw --> Type of Interruption on INT2_XM
#define CTRL_REG5_XM                    0x24 //default: 00011000 rw --> Enable Temperature Sensor, Set Data Rate of Mag
#define CTRL_REG6_XM                    0x25 //default: 00100000 rw --> Set Mag Scale
#define CTRL_REG7_XM                    0x26 //default: 00000001 rw --> Magnetic Sensor Mode
#define STATUS_REG_A                    0x27 //default: OUTPUT r --> Read to know if there is new data available or if it has overwritten previous data
#define OUT_X_L_A                       0x28 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_X_H_A                       0x29 //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_L_A                       0x2A //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Y_H_A                       0x2B //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_L_A                       0x2C //default: OUTPUT r --> Data (Two's Complement)
#define OUT_Z_H_A                       0x2D //default: OUTPUT r --> Data (Two's Complement)
#define FIFO_CTRL_REG                   0x2E //default: 00000000 rw --> Set FIFO Mode
#define FIFO_SRC_REG                    0x2F //default: OUTPUT r --> FIFO Status
#define INT_GEN_1_REG                   0x30 //default: 00000000 rw --> Set Combination of Interrupts
#define INT_GEN_1_SRC                   0x31 //default: OUTPUT r --> Source of Interrupts
#define INT_GEN_1_THS                   0x32 //default: 00000000 rw --> Threshold  
#define INT_GEN_1_DURATION              0x33 //default: 00000000 rw --> Interrupt Duration
#define INT_GEN_2_REG                   0x34 //default: 00000000 rw --> Set Combination of Interrupts
#define INT_GEN_2_SRC                   0x35 //default: OUTPUT r --> Source of Interrupts
#define INT_GEN_2_THS                   0x36 //default: 00000000 rw --> Threshold 
#define INT_GEN_2_DURATION              0x37 //default: 00000000 rw --> Interrupt Duration
#define CLICK_CFG                       0x38 //default: 00000000 rw --> Click Detection Interrupt
#define CLICK_SRC                       0x39 //default: OUTPUT r --> Source
#define CLICK_THS                       0x3A //default: 00000000 rw --> Threshold of Interrupt
#define TIME_LIMIT                      0x3B //default: 00000000 rw --> Time Limit
#define TIME_LATENCY                    0x3C //default: 00000000 rw --> Time Latency
#define TIME_WINDOW                     0x3D //default: 00000000 rw --> Time Window
#define Act_THS                         0x3E //default: 00000000 rw --> Sleept To Wake and Return to Sleep Threshold
#define Act_DUR                         0x3F //default: 00000000 rw --> Duration until Return To Sleep


uint8_t AccMag_Addr = 0x1D; //Accelerometer and Magnetometer I2C address
uint8_t Gyro_Addr = 0x6B; //Gyroscope I2C address

float gx, gy, gz; //Variables to store gyroscope data
float gres = 2000.0 / 32768.0; //Gyroscope resolution - Configured with a range of +/- 2000 deg/s
float ax, ay, az; ///Store accelerometer data
float ares = 2.0 / 32768.0;  // Accelerometer resolution - Configured with a range of +/- 2 g
float mx, my, mz; // Store Magnetometer data
float mres = 2.0 / 32768.0; // Magnetometer resolution - Configured with a range of +/- 2 gauss

void setup()
{
  //Initialize Serial Communication  - to send data to a laptop
  Serial.begin(115200);
  //Initialize I2C
  Wire.begin();
  //Initialize LSM9DS0 - IMU (Inertial Measurement Unit)
  beginIMU();
}

void loop()
{

  uint8_t temp[6]; // We'll read six bytes from the gyro into temp
  ReadBytes(Gyro_Addr, OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
  gx = ((temp[1] << 8) | temp[0]); // Store x-axis values into gx
  gy = ((temp[3] << 8) | temp[2]); // Store y-axis values into gy
  gz = ((temp[5] << 8) | temp[4]); // Store z-axis values into gz

  gx = (gres * gx); // gx to deg/s
  gy = (gres * gy); // gy to deg/s
  gz = (gres * gz); // gz to deg/s

  Serial.write(0x49);
  Serial.println(gx); Serial.println(gy); Serial.println(gz);

  uint8_t temp1[6]; // We'll read six bytes from the accelerometer into temp
  ReadBytes(AccMag_Addr, OUT_X_L_A, temp1, 6); // Read 6 bytes, beginning at OUT_X_L_A
  ax = (temp1[1] << 8) | temp1[0]; // Store x-axis values into ax
  ay = (temp1[3] << 8) | temp1[2]; // Store y-axis values into ay
  az = (temp1[5] << 8) | temp1[4]; // Store z-axis values into az

  ax = (ares * ax); // ax to g
  ay = (ares * ay); // ay to g
  az = (ares * az); // az to g

  Serial.write(0x48);
  Serial.println(ax); Serial.println(ay); Serial.println(az);

  uint8_t temp2[6]; // We'll read six bytes from the mag into temp
  ReadBytes(AccMag_Addr, OUT_X_L_M, temp2, 6); // Read 6 bytes, beginning at OUT_X_L_M
  mx = (temp2[1] << 8) | temp2[0]; // Store x-axis values into mx
  my = (temp2[3] << 8) | temp2[2]; // Store y-axis values into my
  mz = (temp2[5] << 8) | temp2[4]; // Store z-axis values into mz

  mx = (mres * mx); // mx to gauss
  my = (mres * my); // my to gauss
  mz = (mres * mz); // mz to gauss

  Serial.write(0x50);
  Serial.println(mx); Serial.println(my); Serial.println(mz);

  delay(50);
}

void beginIMU()
{
  /*
    Configure Data Rate
    Gyro 95 190 380 760Hz
    Acc 3.125 6.125 25 50 100 200 400 800 1600Hz
    Mag 3.125 6.25 12.5 25 50 100Hz
    Configure Scale
    Gyro 245 500 2000 DPS
    Acc 2 4 6 8 16 g
    Mag 2 4 8 12 Gs
  */

  //Gyro
  WriteByte(Gyro_Addr, CTRL_REG1_G, B00001111); // DR1-0 DR0-0 BW1-0 BW0-0 PD-1 ZEN-1 XEN-1 YEN-1
  WriteByte(Gyro_Addr, CTRL_REG2_G, 0x00); // Reserved-0 Reserved-0 HPM1-0 HPM0-0 HPCF3-0 HPCF2-0 HPCF1-0 HPCF0-0
  WriteByte(Gyro_Addr, CTRL_REG3_G, 0x88); // I1_INT1-1 I1_Boot-0 H_Lactive-0 PP_OD-0 I2_DRDY-1 I2_WTM-0 I2_ORun-0 I2_Empry-0
  WriteByte(Gyro_Addr, CTRL_REG4_G, 0b00100000); // BDU-0 BLE-0 FS1-1 FS0-1 Reserved-0 ST1-0 ST0-0 SIM-0
  WriteByte(Gyro_Addr, CTRL_REG5_G, 0x00); // BOOT-0 FIFO_EN-0 HPen-0 INT1_Sel1-0 INT1_Sel0-0 Out_Sel1-0 Out_Sel1-0
  //Acc
  WriteByte(AccMag_Addr, CTRL_REG0_XM, 0x00); // BOOT-0 FIFO_EN-0 WTMN_EN-0 HP_Click-0 HPIS1-0 HPIS2-0
  WriteByte(AccMag_Addr, CTRL_REG1_XM, 0x57); // AODR3-0 AODR2-1 AODR1-0 AODR0-1 BDU-0 AZEN-1 AYEN-1 AXEN-1
  WriteByte(AccMag_Addr, CTRL_REG2_XM, 0x00); // ABW1-0 ABW0-0 AFS2-0 AFS1-0 AFS0-0 AST1-0 AST0-0 SIM-0
  WriteByte(AccMag_Addr, CTRL_REG3_XM, 0x04); // P1_BOOT-0 P1_TAP-0 P1_INT1-0 P1_INT2-0 P1_INTM-0 P1_DRDYA-1 P1_DRDYM-0 P1_EMPTY-0
  //Mag
  WriteByte(AccMag_Addr, CTRL_REG5_XM, 0x94); // TEMP_EN-1 M_RES1-0 M_RES0-0 M_ODR2-1 M_ODR1-0 M_ODR0-1 LIR2-0 LIR1-0
  WriteByte(AccMag_Addr, CTRL_REG6_XM, 0x00); // Reserved-0 MFS1-0 MFS0-0 Reserved-0 Reserved-0 Reserved-0 Reserved-0 Reserved-0
  WriteByte(AccMag_Addr, CTRL_REG7_XM, 0x00); // AHPM1-0 AHPM0-0 AFDS-0 Reserved-0 Reserved-0 MLP-0 MD1-0 MD0-0
  WriteByte(AccMag_Addr, CTRL_REG4_XM, 0x04); // P2_TAP-0 P2_INT1-0 P2_INT2-0 P2_INTM-0 P2_DRDYA-0 P2_DRDYM-1 P2_Overrun-0 P2_WTM-0
  WriteByte(AccMag_Addr, INT_CTRL_REG_M, 0x09); //XMIEN-0 YMIEN-0 ZMIEN-0 PP_OD-0 IEA-1 IEL-0 4D-0 MIEN-1
  //
}

void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t ReadByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t) 1);
  data = Wire.read();
  return data;
}

void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire.write(subAddress | 0x80);     // Put slave register address in Tx buffer
  Wire.endTransmission();       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read(); // Put read results in the Rx buffer
  }
}
