#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#include "Kalman.h"
#include <HMC5883L_Simple.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>


Kalman kalmanX, kalmanY, kalmanZ, kalmanH; // Create the Kalman instances

MPU6050 accelgyro;
HMC5883L_Simple Compass;

/* IMU Data */
float accX, accY, accZ, gyroXrate, gyroYrate;

int magX, magY, magZ;

// START  GPS SECTION
#define RXPin 4;
#define TXPin 3;

#define HomeLat 40.442;
#define HomeLon -79.942;

float currentLatitude = 0.0;
float currentLongitude = 0.0;

#define HOME_RADIUS 20; //in meters

TinyGPSPlus gps; // The TinyGPS++ object

SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS module


//END GPS SECTION



#define INTERRUPT_PIN 2
#define FIFO_BUFFER_SIZE 42 // maximum packet size plus 2 for the header bytes
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];

double roll, pitch; // Roll and pitch are calculated using the accelerometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ, kalAngleH; // Calculated angle using a Kalman filter

float gyroHeading, compassHeading;

#define SAMPLE_RATE_HZ 100 // Define the sampling rate in Hz

double STEP_THRESHOLD = 1.0; // The threshold during step detection

// Define constants for minimum and maximum time interval between maxima and minima
#define MIN_INTERVAL 110 // Minimum interval time between maxima and minima in ms
#define MAX_INTERVAL 400 // Maximum interval time between maxima and minima in ms

// Define variables to store last valid minima and maxima values
double last_minima = 0.0;
double last_maxima = 0.0;

// Initialize variables to store the time of the last minima and maxima
unsigned long last_minima_time = 0;
unsigned long last_maxima_time = 0;

//nature of the step
#define STEP_UP 1
#define STEP_DOWN 0

#define STEP_LENGTH_CONSTANT 0.331433 // Constant for step length calculation
#define  STEP_LENGTH_COEFFICIENT 0.0407167 // Coefficient for step length calculation

int step_type = STEP_DOWN; // Initialize step type to down

float previous_step_length = 0.0; // Initialize previous step length to 0

struct Postion {
  float x;
  float y;
};

Postion previous_position = {0.0, 0.0}; // Initialize position to (0,0)

uint32_t timer;
uint32_t timerx;


void setup() {
  delay(100); // Wait for sensors to get ready
  
  Serial.begin(9600);
  ss.begin(9600); // GPS baud rate
  Wire.begin();
  Serial.println("Initializing I2C devices...");  // initialize devices

  accelgyro.initialize(); // initialize mpu6050
  pinMode(INTERRUPT_PIN, INPUT);

  // Load the DMP firmware onto the MPU6050
  uint8_t devStatus = accelgyro.dmpInitialize();
  if (devStatus == 0) {
    accelgyro.setDMPEnabled(true);
    Serial.println(F("DMP initialized."));
  } else {
    Serial.print(F("DMP initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
  
  // Compass.SetDeclination(23, 35, 'E');   // initialize hmc5883l
  // Compass.SetSamplingMode(COMPASS_SINGLE);
  // Compass.SetScale(COMPASS_SCALE_130);
  // Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

  delay(100); // Wait for sensors to stabilize

 /* Set Kalman and gyro starting angle */
  updateMPU6050();
  updateHMC5883L();
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

}

void loop() { 
 
  /* Update all the IMU values */
  updateMPU6050();
  // updateHMC5883L();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  /* Roll and pitch estimation */
  updatePitchRoll();

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  kalAngleH = gyroHeading; //compassHeading was removed due to too much inteference

  /* Print Data */
#if 1

  if(detectStep()) {
    previous_step_length = getStepLength();
    Serial.println();    
    Serial.print("Step Length:   "); Serial.print(previous_step_length); Serial.print("\t");

    previous_position = currentPosition();
    Serial.println();
    Serial.print("Coordinates:  ");    
    Serial.print(previous_position.x); Serial.print(" "); Serial.print(previous_position.y);
    Serial.print("\t");

  }

#endif
  delay(10); //Sleep before update from sensors
}

void updateMPU6050() {
    int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert the raw accelerometer values to G's
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  // Convert the raw gyroscope values to degrees per second
  gyroXrate = gx / 131.0;
  gyroYrate = gy / 131.0;

  //get the latest sensor readings from the MPU6050
  accelgyro.getFIFOBytes(fifoBuffer, sizeof(fifoBuffer)); //read the fifo data into the fifoBuffer
  
  //check if there is new data in the FIFO buffer
    if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Get the quaternion values
      Quaternion q;
      accelgyro.dmpGetQuaternion(&q, fifoBuffer);

      // Calculate the gyroHeading angle
      gyroHeading = atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
      gyroHeading = gyroHeading * 180 / M_PI; // Convert from radians to degrees
      if (gyroHeading < 0) {
        gyroHeading += 360; // Ensure that gyroHeading is between 0 and 360 degrees
      }
    }
}

void updateHMC5883L() {
  
  Compass.ReadAxes(&magX, &magY, &magZ);
  compassHeading = Compass.GetHeadingDegrees();
}

void updatePitchRoll() {
 // Calculate pitch and roll from the accelerometer data
  pitch = -atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / M_PI; //flip sign since device is taking max_pitch as negative
  roll = atan2(accY, accZ) * 180 / M_PI;

}

// Define function to detect a valid step
bool detectStep() {
    // Check if pitch value is above the defined threshold    
    //NB: kalAngleY is the kalman (sensor fusion) pitch
        
    double abs_pitch =  kalAngleY;
    Serial.print("Kalman Pitch: "); Serial.print(abs_pitch);
    Serial.print("     Step Threshold: "); Serial.println(STEP_THRESHOLD);
    if(abs_pitch > STEP_THRESHOLD) {
      // check the nature of the step (up or down)
      if(step_type == STEP_DOWN) {
        //set the threshold for the next step which is a step up
        step_type = STEP_UP;
        STEP_THRESHOLD += last_minima;
        last_minima = abs_pitch;
        last_minima_time = millis();
      }else {
        //set the threshold for the next step which is a step down
        step_type = STEP_DOWN;
        STEP_THRESHOLD = last_maxima - STEP_THRESHOLD;
        last_maxima = abs_pitch;
        last_maxima_time = millis();
      }
      // Check step validiity based on the time interval between the last two steps
      unsigned long step_interval = labs(last_minima_time - last_maxima_time);
      // Serial.print("  Time interval: "); Serial.println(step_interval);
      if (step_interval >= MIN_INTERVAL && step_interval <= MAX_INTERVAL) {
        return true;
      }
    }
    return false;
}

float getStepLength() {
  Serial.print(last_maxima); Serial.print("\t");
  Serial.print(last_minima);
  // Calculate the step length based on the pitch angle
  return (labs(last_maxima - last_minima) * STEP_LENGTH_COEFFICIENT + STEP_LENGTH_CONSTANT);
}

// Function to compute current position  {X, Y}
Postion currentPosition() {
  float heading = radians(kalAngleH); //convert to radians for actual values. Try cos(270)
  float x_new = previous_position.x + previous_step_length * cos(heading);
  float y_new = previous_position.y + previous_step_length * sin(heading);
  return {x_new, y_new};
}

//get current GPS coordinates latitude and longitude
void readGPS() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {

      currentLatitude = gps.location.lat();
      currentLongitude = gps.location.lng();
      Serial.print("Latitude: "); Serial.print(currentLatitude, 6);
      Serial.print(" Longitude: "); Serial.println(currentLongitude, 6);
      break;
    }
  }
}


// Function to compute distance between two points
float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // Radius of the earth in km
  float dLat = (lat2-lat1) * M_PI / 180;  // deg2rad below
  float dLon = (lon2-lon1) * M_PI / 180;
  float a = 
    sin(dLat/2) * sin(dLat/2) +
    cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * 
    sin(dLon/2) * sin(dLon/2)
    ; 
  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  float d = R * c; // Distance in km
  d = d * 1000;
  return d;
}

