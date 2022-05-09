/**********************************************************************
Run a balancing servo off a clock.
Get input from IMU:
 * Need to have "I2Cdev" folder in Arduino libraries
 * Need to have "MPU6050" folder in Arduino libraries
Estimate body angle (in accelerometer Y units)
/**********************************************************************/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Pins.h"
#include "My_encoders.h"

/**********************************************************************/

#define MAX_ULONG 0xFFFFFFFF

// I had to increase this to 3ms to accomodate reading the IMU
#define SERVO_INTERVAL 3000 // microseconds

// start data collection after N milliseconds after starting servo
#define START_COLLECT  5000 // milliseconds
// start balancer N milliseconds after START_COLLECT
#define START_BALANCE   100 // milliseconds

#define DEBUG_PRINT_INTERVAL 500 // milliseconds, 

#define MAX_COMMAND 255

// #define PI 3.141592653589793
#define ENCODER_TO_RADIANS 0.004027682889218

/**********************************************************************/

bool go = false;  /* motor enable */

// Encoder readings: these can be positive or negative.
long left_angle = 0;
long right_angle = 0;

// velocity filter stuff
float left_diff[10];
float right_diff[10];
float past_left_angle_radians;
float past_right_angle_radians;
float past_left_velocity[10];
float past_right_velocity[10];

MPU6050 accelgyro;

// Current accelerometer and gyro zero values.
int16_t ax, ay, az;
int16_t gx, gy, gz;

int ax0 = 60;
int ay0 = 100;
// adjustment of az is not useful.
int gx0 = 556;
int gy0 = -26;
int gz0 = 0;

int last_body_angle = 0;
int last_gxx = 0; // last X gyro reading

// Keep track of late control cycles. 
unsigned long servo_late = 0; // How many times am I late?
unsigned long max_servo_late = 0; // What was the worst one?

/**********************************************************************/
/**********************************************************************/

void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void motor_stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void motor_left_command( int speed )
{
    if ( speed >= 0 )
    {
      digitalWrite( AIN1, 1 );
      analogWrite( PWMA_LEFT, speed );
    }
  else
    {
      digitalWrite( AIN1, 0 );
      analogWrite( PWMA_LEFT, -speed );
    }
}

// reverses the sign of "speed"
void motor_right_command( int speed )
{
  if ( speed >= 0 )
    {
      digitalWrite( BIN1, 1 );
      analogWrite( PWMB_RIGHT, speed );
    }
  else
    {
      digitalWrite( BIN1, 0 );
      analogWrite( PWMB_RIGHT, -speed );
    }
}

/**********************************************************************/

void setup()
{

  Wire.begin();  // I2C (TWI) uses Wire library
  Serial.begin( 1000000 );  // run Serial fast to print out data
  Wire.setClock( 1000000UL ); // run I2C as fast as possible
  analogReference(INTERNAL1V1); // was INTERNAL on 328P, voltage

  while( !Serial ); // Wait for serial port to become ready
  Serial.println( "balance1 version 1" ); // print out what code is running
  delay(1000); // Delay to make sure above message prints out.

  motor_init(); // Initialize motors
  Serial.println( "motor_init done." );
  delay(1000); // Delay to make sure above message prints out.

  Encoder_init( &left_angle, &right_angle ); // Initialize (zero) encoders
  Serial.println( "Initialized encoders" );
  delay(1000); // Delay to make sure above message prints out.

  accelgyro.initialize(); // Initialize IMU
  Serial.println( "IMU initialized" );
  delay(1000); // Delay to make sure above message prints out.

  Serial.println( "Robot should be standing up with training wheels."  );
  Serial.println( "Type g <return> to run test, s <return> to stop."  );
  Serial.println( "Typing window is at the top of the Arduino serial monitor window." );
  Serial.println( "Type into the main window of a Putty serial monitor window." );

  // code starts disabled, so every time Arduino powers up nothing happens
  go = false; 
}

/******************************************************************/

// Take user input
void ProcessCommand()
{

  if ( Serial.available() <= 0 )
    return;

  int c = Serial.read();
  switch (c)
    {
      case 'S': case 's':
        Serial.println( "Stop!" );
        go = false;
        break;
      case 'G': case 'g':
      Serial.println( "Go!" );
        go = true;
        break;
      default:
        break;
    }
}

/**********************************************************************
Run a balancer with the given gains.
run time starts at the beginning and is a millisecond clock.
START_COLLECT   starts data collection START_COLLECT milliseconds after beginning
START_BALLANCE   starts balancer START_COLLECT + START_BALANCE milliseconds
                 after beginning.
DEBUG_PRINT_INTERVAL   milliseconds between debug printouts
/**********************************************************************/

void run_balancer( float body_angle_gain, float body_angular_velocity_gain,
     		   float some_kind_of_integral_gain, // to be determined
                   unsigned long run_time_limit, int collect_data )
{
  // Clocks and time management
  // used to keep track of time intervals in servo
  unsigned long last_micros = micros();
  unsigned long last_millis = millis();
  int servo_time = 0; // Servo clock: needs to be signed, can be int
  // how long have we been running? Safety feature
  unsigned long run_time = 0; // milliseconds
  // Clocks for printing and other stuff
  unsigned long debug_print_time = 0; // milliseconds

  run_time_limit += START_COLLECT; // account for startup time

  // state
  int started = 0;

  // integrators
  long integrator_state = 0;

  // keep track of voltage
  int voltage_raw = analogRead(VOL_MEASURE_PIN); //Read voltage value
  double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
  if ( collect_data )
    {
      Serial.print( "Current voltage " );
      Serial.print( voltage );
      Serial.print( " " );
      Serial.println( voltage_raw );
    }

  // print out the angle_gain
  if ( collect_data )
    {
      Serial.print( "Gains " );
      Serial.print( body_angle_gain );
      // Serial.print( " " );
      // Serial.print( body_angle_integral_gain );
      Serial.print( " " );
      Serial.println( body_angular_velocity_gain );
    }

  // print out IMU biases
  if ( collect_data )
    {
      Serial.print( "IMU biases " );
      Serial.print( ax0 );
      Serial.print( " " );
      Serial.print( ay0 );
      Serial.print( " " );
      Serial.print( gx0 );
      Serial.print( " " );
      Serial.print( gy0 );
      Serial.print( " " );
      Serial.println( gz0 );
    }

  // servo loop
  for( ; ; )
    {

/*****************************************
Timekeeping part */

      unsigned long current_micros = micros();
      if ( current_micros > last_micros )
        servo_time += current_micros - last_micros;
      else // rollover
        servo_time += (MAX_ULONG - last_micros) + current_micros;
      last_micros = current_micros;

      // It isn't time yet ...
      if ( servo_time < SERVO_INTERVAL )
        continue;

      // It is time! Reset the servo clock.
      servo_time -= SERVO_INTERVAL;

      // Keep track of maximum lateness
      if ( max_servo_late < servo_time )
        max_servo_late = servo_time;

      // Let's have some slop in counting late cycles.
      if ( servo_time > 50 )
        servo_late += 1;
    
      // handle the millisecond clocks
      unsigned long current_millis = millis();
      int millis_increment;
      if ( current_millis > last_millis )
        millis_increment = current_millis - last_millis;
      else // rollover
        millis_increment = (MAX_ULONG - last_millis) + current_millis;
      last_millis = current_millis;
  
      run_time += millis_increment;
      debug_print_time += millis_increment;

/*****************************************
Administrative part */

      // Turn off after a while to keep from running forever
      if ( ( run_time > run_time_limit ) || ( !go ) )
        {
          motor_stop();
          Serial.println( "Motor stopped." );
          Serial.println( "Wheels should be off the ground."  );
          Serial.println( "Type g <return> to run test, s <return> to stop." );
          return;
        }

/*****************************************
State estimation part */


      // Read the sensors
      Read_encoders( &left_angle, &right_angle );
/*
      float left_angle_radians = ENCODER_TO_RADIANS*left_angle;
      float right_angle_radians = ENCODER_TO_RADIANS*right_angle;

//     a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
//                 - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
      left_diff[2] = left_diff[1];
      left_diff[1] = left_diff[0];
      // this is radians/sec - diff/Ts
      left_diff[0] = 500*(left_angle_radians - past_left_angle_radians);
      past_left_angle_radians = left_angle_radians;
      float left_velocity = b_filter[0]*left_diff[0]
                      + b_filter[1]*left_diff[1]
                      + b_filter[2]*left_diff[2]
		      - a_filter[1]*past_left_velocity[0]
		      - a_filter[2]*past_left_velocity[1];
      past_left_velocity[1] = past_left_velocity[0];
      past_left_velocity[0] = left_velocity;

      right_diff[2] = right_diff[1];
      right_diff[1] = right_diff[0];
      // this is radians/sec - diff/Ts
      right_diff[0] = 500*(right_angle_radians - past_right_angle_radians);
      past_right_angle_radians = right_angle_radians;
      float right_velocity = b_filter[0]*right_diff[0]
                      + b_filter[1]*right_diff[1]
                      + b_filter[2]*right_diff[2]
		      - a_filter[1]*past_right_velocity[0]
		      - a_filter[2]*past_right_velocity[1];
      past_right_velocity[1] = past_right_velocity[0];
      past_right_velocity[0] = right_velocity;
*/
      // Current accelerometer and gyro zero values.
      int16_t ax, ay, az;
      int16_t gx, gy, gz;

      // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accelgyro.getAcceleration( &ax, &ay, &az );
      gx = accelgyro.getRotationX();
      // subtract zeros
      int axx = ax - ax0;
      int ayy = ay - ay0;
      // adjustment of az is not useful.
      int azz = az;
      int gxx = gx - gx0;
      // int gyy = gy - gy0;
      // int gzz = gz - gz0;

      float gscale = 0.001;
      float Kf_body = 0.1;

      // prediction step
      int body_angle = last_body_angle + gscale*(gxx + last_gxx);
      // measurement update
      body_angle = body_angle - Kf_body*(body_angle - ayy);
      last_body_angle = body_angle;
      last_gxx = gxx;

      int body_angular_velocity = gxx;

/*****************************************

//    if ( run_time >= move_time )

/*
      float left_error = left_angle_radians - angle_desired;
      float right_error = right_angle_radians - angle_desired;
*/  

      long command_long = 0;
    
      if ( run_time < START_COLLECT + START_BALANCE )
        { // Do nothing;
      	}
      else // Let's balance!
        {
          command_long = -body_angle_gain*body_angle
	                 
	                  -body_angular_velocity_gain*body_angular_velocity;
    	                 // + angle_error_integral_left;
          command_long = command_long >> 8;			 
        }

      int command = (int) command_long;

      if ( command > MAX_COMMAND )
        command = MAX_COMMAND;
      if ( command < -MAX_COMMAND )
        command = -MAX_COMMAND;

      motor_left_command( command );
      motor_right_command( command );

/*****************************************
Data printing part */

      // print out debugging info before data collection
      if ( ( debug_print_time > DEBUG_PRINT_INTERVAL )
           && ( ( run_time < START_COLLECT )
	        || ( run_time > run_time_limit ) ) )
        {
          Serial.print( servo_late );
          Serial.print( " " );
          Serial.print( max_servo_late );
          Serial.print( " " );
          Serial.print( ayy );
          Serial.print( " " );
          Serial.println( body_angle );

	  /*
          Serial.print( " " );
          Serial.print( left_error );
          Serial.print( " " );
          Serial.print( right_error );
          Serial.print( " " );
          Serial.println( ramp );
	  */
          ProcessCommand();
          debug_print_time -= DEBUG_PRINT_INTERVAL;
        }

      // print out data
      if ( ( run_time >= START_COLLECT ) && ( run_time <= run_time_limit )
           && collect_data )
        {
          if ( !started )
            {
              servo_late = 0;
              max_servo_late = 0;
              started = 1;
              Serial.println( "Data" );
            }
          Serial.print( current_micros );
          Serial.print( " " );
          Serial.print( left_angle );
          Serial.print( " " );
          Serial.print( right_angle );
          Serial.print( " " );
          Serial.print( command );
          Serial.print( " " );
          Serial.print( command );
          Serial.print( " " );
	        // Serial.print( ax );
	        // Serial.print( " " );
          Serial.print( ayy );    // positive is tipping forward
          Serial.print( " " );
          Serial.print( body_angle );
          Serial.print( " " );
          Serial.print( azz );    // positive is down
          Serial.print( " " );
          Serial.println( gxx );  // positive is body rolling forward
	        // Serial.print( " " );
	        // Serial.println( gy );
	        // Serial.print( " " );
	        // Serial.println( gz );
          // Serial.print( " " );
          // Serial.println( left_velocity);
          // Serial.print( " " );
          // Serial.println( right_velocity);
	        // Serial.print( " " );
          // Serial.print( left_angle_radians);
          // Serial.print( " " );        
          // Serial.println( left_error ); 
        }
    }
}

/**********************************************************************/

void loop()
{
  if ( !go )
    {
      motor_stop();
      delay(500);
      ProcessCommand();
      return;
    }


  Serial.println( "Please type the Y accelerometer zero: " );
  Serial.setTimeout( 100000 ); // 10 seconds
  ay0 = Serial.parseInt();
  Serial.print( "You typed: " );
  Serial.println( ay0 );
  if ( ay0 < -4000 || ay0 > 4000 )
    {
      Serial.println( "Bad value: Start over" );
      return;
    }

/*
  Serial.println( "Please type the X gyro zero: " );
  Serial.setTimeout( 100000 ); // 10 seconds
  gx0 = Serial.parseInt();
  Serial.print( "You typed: " );
  Serial.println( gx0 );
  if ( gx0 < -1000 || gx0 > 1000 )
    {
      Serial.println( "Bad value: Start over" );
      return;
    }
    */

  Serial.println( "Please type the desired body angle wrt vertical gain: " );
  Serial.setTimeout( 100000 ); // 10 seconds
  float body_angle_gain = Serial.parseInt();
  Serial.print( "You typed: " );
  Serial.println( body_angle_gain );
  if ( body_angle_gain < 0 || body_angle_gain > 10000 )
    {
      Serial.println( "Bad value: Start over" );
      return;
    }

  Serial.println( "Please type the desired body angular velocity gain: " );
  Serial.setTimeout( 100000 ); // 10 seconds
  float body_angular_velocity_gain = Serial.parseInt();
  Serial.print( "You typed: " );
  Serial.println( body_angular_velocity_gain );
  if ( body_angular_velocity_gain < 0 || body_angular_velocity_gain > 100 )
   {
      Serial.println( "Bad value: Start over" );
      return;
    }

  Encoder_init( &left_angle, &right_angle ); // set wheel encoders to zero

  // run an experiment
  run_balancer( body_angle_gain, body_angular_velocity_gain, 0.0, 15000, 1 );

  Serial.println( "Stop!" );
  go = false;
}
  
/**********************************************************************/
