/*
 * test1_MPU6050
 * Robot prints out acceleration and gryo readings from MPU6050, as
 * well as their averages.
 * You need to restart the program (press the reset button on the Arduino
 * board) to restart the averaging.
 * 
 * Need to have "I2C dev" folder in Arduino libraries
 * Need to have "MPU6050" folder in Arduino libraries
 *
 */
/*********************************************************************/

#include "I2Cdev.h"
#include "MPU6050.h"

/*********************************************************************/

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Current accelerometer and gyro zero values.
int ax0 = 0;
int ay0 = 0;
// adjustment of az is not useful.
int gx0 = 0;
int gy0 = 0;
int gz0 = 0;

#define N_AVERAGES 6
double sums[N_AVERAGES];
unsigned long count = 0;
unsigned int loop_count = 0;

/*********************************************************************/

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while( !Serial );
  Serial.println( "test1_MPU6050 version 24" );
  Serial.println( "Printing out accelerometer (a) and gyro (g) readings and averages." );
  Serial.println( "If I hang here, the MPU is in a bad state. Power cycle the robot." );
  delay(1000); // take time to allow above messages to print.
  accelgyro.initialize();
  Serial.println( "Initialized" );
  for ( int i = 0; i < N_AVERAGES; i++ )
    sums[i] = 0;
  delay(1000); // take time to allow above messages to print.
}

/*********************************************************************/

void loop()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // subtract zeros
  ax -= ax0;
  ay -= ay0;
  // adjustment of az is not useful.
  gx -= gx0;
  gy -= gy0;
  gz -= gz0;
  // sum
  sums[0] += ax;
  sums[1] += ay;
  sums[2] += az;
  sums[3] += gx;
  sums[4] += gy;
  sums[5] += gz;
  ++count;

  delay(5);

  // adjust this to set how rapidly stuff prints out
  if ( ++loop_count < 100 )
    return;

  loop_count = 0;

  Serial.print(count);
  Serial.print(" a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  Serial.print(count);
  Serial.print(" ave:\t");
  Serial.print(sums[0]/count); Serial.print("\t");
  Serial.print(sums[1]/count); Serial.print("\t");
  Serial.print(sums[2]/count); Serial.print("\t");
  Serial.print(sums[3]/count); Serial.print("\t");
  Serial.print(sums[4]/count); Serial.print("\t");
  Serial.println(sums[5]/count); 

  Serial.println();
}
