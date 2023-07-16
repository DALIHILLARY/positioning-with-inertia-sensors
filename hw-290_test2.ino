#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#include "Kalman.h"
#include <HMC5883L_Simple.h>
#include <TinyGPS++.h>

Kalman kalmanX, kalmanY, kalmanZ, kalmanH; // Create the Kalman instances

MPU6050 accelgyro;
HMC5883L_Simple Compass;

/* IMU Data */
float accX, accY, accZ, gyroXrate, gyroYrate;

int magX, magY, magZ;

float HOME_LATITUDE = 0.0;
float HOME_LONGITUDE = 0.0;

/* Headings to determine movement direction
   Used to see if the direction user taking leads home or away
  */
float HIGHER_HEADING = 0.0;
float LOWER_HEADING = 0.0;
#define HEADING_DRIFT 30 // this will be added and subtracted from the current heading

float LAST_GPS_DISTANACE = 0.0; // for comparison with current /positive or negative direction

#define HIGHEST_GPS_SAMPLING_INTERVAL 300 // The lowest GPS is allowed to sample 5 min
#define LOWEST_GPS_SAMPLING_INTERVAL 30 // The highest GPS is allowed to sample 30 seconds
#define SAMPLING_FACTOR 2            // The multipling or decreasing factor for change in sampling

int currentGPSSamplingInterval = LOWEST_GPS_SAMPLING_INTERVAL; // begin off with the lowest sampling rate
unsigned long lastGPSSamplingTime = millis();                 // the last time the GPS was sampled

bool isOutsidePerimeter = false;
bool isMovingAway = false;

float currentLatitude = 0.0;
float currentLongitude = 0.0;

float HOME_RADIUS = 20.0; // in meters

TinyGPSPlus gps; // The TinyGPS++ object

// END GPS SECTION

// TRANSIMISSIONS SECTION
unsigned long previousIdleTransmission = millis();
unsigned long previousActiveTransmission = millis();

#define GPS_SWITCH 18
#define INTERRUPT_PIN 2
#define FIFO_BUFFER_SIZE 42 // maximum packet size plus 2 for the header bytes
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];

float roll, pitch; // Roll and pitch are calculated using the accelerometer

float gyroXangle, gyroYangle, gyroZangle;         // Angle calculate using the gyro only
float compAngleX, compAngleY, compAngleZ;         // Calculated angle using a complementary filter
float kalAngleX, kalAngleY, kalAngleZ, kalAngleH; // Calculated angle using a Kalman filter

float gyroHeading, compassHeading;

// #define SAMPLE_RATE_HZ 100 // Define the sampling rate in Hz

// double STEP_THRESHOLD = 1.0; // The threshold during step detection

// // Define constants for minimum and maximum time interval between maxima and minima
// #define MIN_INTERVAL 110 // Minimum interval time between maxima and minima in ms
// #define MAX_INTERVAL 400 // Maximum interval time between maxima and minima in ms

// // Define variables to store last valid minima and maxima values
// double last_minima = 0.0;
// double last_maxima = 0.0;

// // Initialize variables to store the time of the last minima and maxima
// unsigned long last_minima_time = 0;
// unsigned long last_maxima_time = 0;

// // nature of the step
// #define STEP_UP 1
// #define STEP_DOWN 0

#define STEP_LENGTH_CONSTANT 0.84434124     // Constant for step length calculation
#define STEP_LENGTH_COEFFICIENT 0.0407167 // Coefficient for step length calculation


float previous_step_length = 0.0; // Initialize previous step length to 0

struct Postion
{
  float x;
  float y;
};

Postion previous_position = {0.0, 0.0}; // Initialize position to (0,0)

uint32_t timer;
uint32_t timerx;

// WINDOWED ALGORITHM START HEADER

// Define window size
const int windowSize = 10;

// Define threshold multiplier (adjust this value as needed)
const float thresholdMultiplier = 1.5;

// Initialize pitch values and peak detection variables
float pitchValues[windowSize];
float baseline = 0;
float upperThreshold = 0;
float lowerThreshold = 0;

float lastPeak = -999;
float lastValley = -999;

bool isSlope = false;
bool isStart = true;

// END WINDOWED ALGORITHM END HEADER

void setup()
{
  delay(100); // Wait for sensors to get ready

  Serial.begin(9600);
  Serial2.begin(9600); // GPS baud rate
  Wire.begin();

  Serial.println("Initializing I2C devices..."); // initialize devices

  accelgyro.initialize(); // initialize mpu6050

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(GPS_SWITCH, OUTPUT); // use digital pin 18 to toggle the GPS

  // Load the DMP firmware onto the MPU6050
  uint8_t devStatus = accelgyro.dmpInitialize();
  if (devStatus == 0)
  {
    accelgyro.setDMPEnabled(true);
    Serial.println(F("DMP initialized."));
  }
  else
  {
    Serial.print(F("DMP initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L

  delay(100); // Wait for sensors to stabilize

  /* Set Kalman and gyro starting angle */
  updateMPU6050();
  updatePitchRoll();

  kalmanX.setAngle(roll); // First set roll starting angle
  gyroXangle = roll;
  compAngleX = roll;

  kalmanY.setAngle(pitch); // Then pitch
  gyroYangle = pitch;
  compAngleY = pitch;

  // kalmanH.setAngle(compassHeading); //heading from the compass

  timer = micros(); // Initialize the timer
  timerx = micros();

  // //LOOP UNTIL WE GET HOPE COORDINATES
  // while(HOME_LATITUDE == currentLatitude)  {
  //   // get current GPS coordinates
  //   readGPS();
  //   // assign the new readings as home
  //   HOME_LATITUDE = currentLatitude;
  //   HOME_LONGITUDE = currentLongitude;
  // }

}

void loop()
{

  /* Update all the IMU values */
  updateMPU6050();
  // updateHMC5883L();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  /* Roll and pitch estimation */
  updatePitchRoll();

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  kalAngleH = gyroHeading; // compassHeading was removed due to too much inteference

  /* Print Data */
#if 1

  // //GET CONSTANT VALUES,, FOR TESTING
  
  // Serial.print(kalAngleH); Serial.print("\t");
  // Serial.print(kalAngleY); Serial.print("\t");
  // Serial.print(currentLatitude,20); Serial.print("\t");
  // Serial.print(currentLongitude,20); Serial.println();
    
  // readGPS();

  // //END TEST
  
  if (detectStepWindowed())
  {
    previous_step_length = getStepLength();
    previous_position = getCurrentPosition();
    Serial.println("STEP DETECTED");
    Serial.print("STEP LENGTH: ");
    Serial.print(previous_step_length);
    
    // Check is user is already outside the perimeter
    if (isOutsidePerimeter)
    {
      // TODO: Check the current heading if it the same drift or off
      if (LOWER_HEADING <= kalAngleH && kalAngleH >= HIGHER_HEADING)
      {
        if(((millis() - lastGPSSamplingTime) / 1000) >= currentGPSSamplingInterval) {
          Serial.println("INFO: Next GPS read reached");
          sampleGPS();
        }
      }
      else
      {
        Serial.println("INFO: User made a turn");
        HIGHER_HEADING = int(kalAngleH + HEADING_DRIFT) % 360;
        LOWER_HEADING = int(kalAngleH - HEADING_DRIFT) % 360;

        sampleGPS();
      }
    }
    else
    {
      // Check distance from origin not to exceed HOME_RADIUS
      if (getDistanceFromOrigin() >= HOME_RADIUS)
      {
        Serial.println("Inertia out of perimeter");
        readGPS(); // Check the GPS on updated position
        float currentPosition = getGPSDistanceFromOrigin(); // Get the current position

        // Check if the new position is within the HOME_RADIUS
        if (currentPosition <= HOME_RADIUS)
        {
          Serial.println("WARNING: Was a false alarm");
          isOutsidePerimeter = false; // set flag is in perimeter
          isMovingAway = false;         
          // TODO: reset the postioning coordinates for inertia sensor
        }
        else
        {
          Serial.println("WARNING: User out of perimeter");   
          isMovingAway = true;
          isOutsidePerimeter = true; // set flag is outside perimeter
          HIGHER_HEADING = int(kalAngleH + HEADING_DRIFT) % 360;
          LOWER_HEADING = int (kalAngleH - HEADING_DRIFT) % 360;

          // TODO : Send distress signals to concerned people every 1 minute
          if (millis() - previousActiveTransmission > 60000)
          {
            previousActiveTransmission = millis();
            // TODO : Send distress signals to concerned people
          }
        }
        LAST_GPS_DISTANACE = currentPosition;
        
      }
      else
      {
        isOutsidePerimeter = false; // set the flag they are in perimeter
        // Serial.println("Within HOME_RADIUS");

        // TODO : Check if last idle transmission is more than 30 minutes
        if (millis() - previousIdleTransmission > 1800000)
        {
          // TODO : Send idle transmission to concerned people
          previousIdleTransmission = millis();
        }
      }
    }
  }

#endif
  delay(100); // Sleep before update from sensors
}

void updateMPU6050()
{
  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert the raw accelerometer values to G's
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  // Convert the raw gyroscope values to degrees per second
  gyroXrate = gx / 131.0;
  gyroYrate = gy / 131.0;

  // get the latest sensor readings from the MPU6050
  accelgyro.getFIFOBytes(fifoBuffer, sizeof(fifoBuffer)); // read the fifo data into the fifoBuffer

  // check if there is new data in the FIFO buffer
  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    // Get the quaternion values
    Quaternion q;
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);

    // Calculate the gyroHeading angle
    gyroHeading = atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
    gyroHeading = gyroHeading * 180 / M_PI; // Convert from radians to degrees
    if (gyroHeading < 0)
    {
      gyroHeading += 360; // Ensure that gyroHeading is between 0 and 360 degrees
    }
  }
}

void updatePitchRoll()
{
  // Calculate pitch and roll from the accelerometer data
  pitch = -atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / M_PI; // flip sign since device is taking max_pitch as negative
  roll = atan2(accY, accZ) * 180 / M_PI;
}

float getStepLength()
{
  // Serial.println("=======================================");
  // Serial.print("Last Peak: "); Serial.println(lastPeak);
  // Serial.print("Last Valley: "); Serial.println(lastValley);
  // Calculate the step length based on the pitch angle
  return (labs(lastPeak - lastValley) * STEP_LENGTH_COEFFICIENT + STEP_LENGTH_CONSTANT);
}

// Function to compute current position  {X, Y}
Postion getCurrentPosition()
{
  float heading = radians(kalAngleH); // convert to radians for actual values. Try cos(270)
  float x_new = previous_position.x + previous_step_length * cos(heading);
  float y_new = previous_position.y + previous_step_length * sin(heading);
  return {x_new, y_new};
}

// Function to compute distance between {0.0} and current position
float getDistanceFromOrigin()
{
  return sqrt(pow(previous_position.x, 2) + pow(previous_position.y, 2));
}

// get current GPS coordinates latitude and longitude
void readGPS()
{
  if(digitalRead(GPS_SWITCH) == 0) {
      digitalWrite(GPS_SWITCH, HIGH); // switch on gps, to get intial readings
      delay(100); // latch for operation
      Serial.println("GPS started");

  } 
  while (Serial2.available() > 0)
  {
    // Serial.println(Serial2.read());
    gps.encode(Serial2.read());
    if (gps.location.isValid() && gps.location.isUpdated())
    {
      // Serial.println("GPS Connected");     
      currentLatitude = gps.location.lat();
      currentLongitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.print(currentLatitude, 20);
      Serial.print(" Longitude: ");
      Serial.println(currentLongitude, 20);

      lastGPSSamplingTime = millis();

      digitalWrite(GPS_SWITCH, LOW); // Turn off GPS
      Serial.println("GPS OFF");
      break;
    }
  }
}
// Function to compute GPS distance from home/ origin
float getGPSDistanceFromOrigin()
{
  float R = 6371;                                              // Radius of the earth in km
  float dLat = (currentLatitude - HOME_LATITUDE) * M_PI / 180; // deg2rad below
  float dLon = (currentLongitude - HOME_LONGITUDE) * M_PI / 180;
  float a =
      sin(dLat / 2) * sin(dLat / 2) +
      cos(HOME_LATITUDE * M_PI / 180) * cos(currentLatitude * M_PI / 180) *
          sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c; // Distance in km
  d = d * 1000;
  return d;
}

// Function to compute distance between two points
float getDistanceBtnCoordinates(float lat1, float lon1, float lat2, float lon2)
{
  float R = 6371;                          // Radius of the earth in km
  float dLat = (lat2 - lat1) * M_PI / 180; // deg2rad below
  float dLon = (lon2 - lon1) * M_PI / 180;
  float a =
      sin(dLat / 2) * sin(dLat / 2) +
      cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
          sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c; // Distance in km
  d = d * 1000;
  return d;
}

bool detectStepWindowed()
{
  float pitch = kalAngleY;
  // Add pitch value to window
  for (int i = 0; i < windowSize - 1; i++)
  {
    pitchValues[i] = pitchValues[i + 1];
  } 
  pitchValues[windowSize - 1] = pitch;

  // Compute baseline and threshold
  baseline = computeBaseline(pitchValues, windowSize);
  upperThreshold = baseline + thresholdMultiplier * computeStdDev(pitchValues, windowSize);
  lowerThreshold = baseline - thresholdMultiplier * computeStdDev(pitchValues, windowSize);

  bool executePitch = false; //If we qualify it to be a step
  
  for (int i = 1; i < windowSize - 1; i++)
  {
    // Check for peaks within window
    if (pitchValues[i] > upperThreshold && pitchValues[i] > pitchValues[i - 1] && pitchValues[i] > pitchValues[i + 1])
    {
      if(!isSlope || isStart) {
        executePitch = true;
        isSlope = true;
        isStart = false;
        lastPeak = pitchValues[i];     
      }
    }else if (pitchValues[i] < lowerThreshold && pitchValues[i] < pitchValues[i - 1] && pitchValues[i] < pitchValues[i + 1])
    {
          // Check for valleys within window

      if(isSlope || isStart){
        executePitch = true;
        isSlope = false;
        isStart = false;
        lastValley =  pitchValues[i];       
      }
    }

    if (lastPeak != -999 && lastValley != -999 && executePitch) {
      return true;   
    }
  }


  return false;
}

// Helper function to compute baseline value
float computeBaseline(float *values, int numValues)
{
  float sum = 0;
  for (int i = 0; i < numValues; i++)
  {
    sum += values[i];
  }
  return sum / numValues;
}

// Helper function to compute standard deviation of values
float computeStdDev(float *values, int numValues)
{
  float sum = 0;
  float sumSquares = 0;
  for (int i = 0; i < numValues; i++)
  {
    sum += values[i];
    sumSquares += values[i] * values[i];
  }
  float mean = sum / numValues;
  float variance = (sumSquares / numValues) - (mean * mean);
  float sd = sqrt(variance);
  if(sd < 1) {
    return 1.0;
  }else {
    return sd;
  }
}


// Function to get GPS and determine next sampling rate
void sampleGPS()
{

        readGPS(); // Check the GPS on updated position
        float currentDistance = getGPSDistanceFromOrigin();        
        if (currentDistance > LAST_GPS_DISTANACE)
        {
          isMovingAway = true;
          // decrease the sampling rate since user is moving further away but go below the LOWEST_GPS_SAMPLING_INTERVAL
          decreaseGPSSamplingRate();
        }
        else
        {
          if(currentDistance <= HOME_RADIUS) {
            isOutsidePerimeter = false;
          }
          isMovingAway = false;
          // increase the sampling rate since user is coming back, no worries here
          increaseGPSSamplingRate();
        }
        LAST_GPS_DISTANACE = currentDistance;
}

// Function to decrease GPS sampling rate
void increaseGPSSamplingRate()
{
  // Serial.println("INFO: Increasing sampling rate");
  currentGPSSamplingInterval = currentGPSSamplingInterval / SAMPLING_FACTOR;
  currentGPSSamplingInterval = max(currentGPSSamplingInterval, LOWEST_GPS_SAMPLING_INTERVAL);
}

// Function to increase GPS sampling rate
void decreaseGPSSamplingRate()
{
  // Serial.println("INFO: Decreasing sampling rate");
  currentGPSSamplingInterval = currentGPSSamplingInterval * SAMPLING_FACTOR;
  currentGPSSamplingInterval = min(currentGPSSamplingInterval, HIGHEST_GPS_SAMPLING_INTERVAL);
}