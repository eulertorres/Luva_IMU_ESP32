/*============= Cration of a dataset using a sensored glove (5 fingers) with 20 MPU9250===============================*/
/*  Created by Euler torres to Glove aplication 20/06/2022
 *  Lybrary by Sebastian Madgwick [Madgwick] https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 *  
 Reading, Calculation and estimation 2 IMU angles
  Estimation of angles (Roll, Pitch, Yall) of 2 MPUs 9250 using Magnetic, Angular Rate and Gravity (MARG),
  Estimation of force of the grip applied by each finger using FSLP (Force Sensing Linear Potentiometer)
  Estimation of the position of the object using FSLP (Force Sensing Linear Potentiometer)
*/
 
#include "Wire.h"               //Biblioteca para comunicação I²C
#include "MPU9250.h"            //Header file wich includes all IMU communication
#include "MadgwickAHRS.h"       //Madwick Filter header file, for both IMU
#include "FSLP.h"               //FSLP header file, to get te measurements

#define SerialDebug 		    true     // Set true to get Serial output for debugging and dataset information
#define onlyAngles 			    true     // Debug only the angles (Roll, Pitch, Yall & Roll2, Pitch 2, Yall2)
#define calibrationMag9250 	false	   // Set true to calibrate the MPU920 magnetometer in a new enviorment
#define selftest_en 		    false     // Set true to see if the MPU9250s correponds to the factory integrity
#define gyro_accel_cal 		  true     // Set true to calibrate the biases and scales of each MPU9250

#define button 15       			   // Buton to start printing Dataset
#define Gled   2         			   // Extra Led to indicate printing
#define M_A0   17         			 // Digital output for mux selection
#define M_A1   16         			 // Digital output for mux selection
#define M_A2   4         			   // Digital output for mux selection
#define FD1    23                     // FSLP D1 (common for all FSLPs) [Digital OUTPUT]

#define TCA_addr 0x70         

char object[] = "Smartphone";          // Set the object to be printed in dataset

// MPU9250 Configuration. Specify sensor full scale
/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float 	aRes, gRes, mRes;      			 	   // scale resolutions per LSB for the sensors
int16_t MPU9250_1_data[7], MPU9250_2_data[7];  // used to read all 14 bytes at once from both MPU9250 accel/gyro of the specific finger
int16_t magCount1[3], magCount2[3];      	   // Stores the 16-bit signed magnetometer sensor output
float   magCalibration1[15];				   // Factory mag calibration and mag bias of all MPU1s
float   magCalibration2[15];				   // Factory mag calibration and mag bias of all MPU2s
float   SelfTest[6];                     	   // holds results of gyro and accelerometer self test

//------------------------------------------------ IMPORTANT-------------------------------------------------------------------------------------------------------------
// These can be measured once and entered here or can be calculated each time the device is powered on 
//Gyro biases for each axis to all 10 MPU9250_1 (proximal)
float   gyroBias1[15] = {0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12};

//Accelerometer biases for each axis to all MPU9250_1 (proximal)
float 	accelBias1[15] = {0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952};

//Gyro biases for each axis to all 10 MPU9250_2 (medium)
float   gyroBias2[15] = {0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12, 0.96, -0.21, 0.12};

//Accelerometer biases for each axis to all MPU9250_2 (medium)
float	accelBias2[15] = {0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952, 0.00299, -0.00916, 0.00952};

//Magnetometer biases for each axis to all MPU9250_1 (proximal)
float   magBias1[15] = {-95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83};

//Magnetometer scales for each axis to all MPU9250_1 (proximal)
float	magScale1[15]  = {0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22};

//Magnetometer biases for each axis to all MPU9250_2 (medium)
float   magBias2[15] = {-95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83};

//Magnetometer scales for each axis to all MPU9250_2 (medium)
float	magScale2[15]  = {0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22};

float pitch1[5], yaw1[5], roll1[5], pitch2[5], yaw2[5], roll2[5]; // absolute orientation
float a12, a22, a31, a32, a33;                                    // rotation matrix coefficients for Euler angles and gravity components MPU1
float AA12, AA22, AA31, AA32, AA33;                               // rotation matrix coefficients for Euler angles and gravity components MPU2
float deltat1 = 0.0f, deltat2 = 0.0f;   			                    // integration interval for both filter schemes
uint32_t lastupdate1[5] = {0, 0, 0, 0, 0}, lastupdate2[5] =  {0, 0, 0, 0, 0};                        // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                                      // used to calculate integration interval

float ax1[5], ay1[5], az1[5], gx1[5], gy1[5], gz1[5], mx1[5], my1[5], mz1[5]; // variables to hold latest sensor data values of the MPU_1 for all 5 fingers
float ax2[5], ay2[5], az2[5], gx2[5], gy2[5], gz2[5], mx2[5], my2[5], mz2[5]; // variables to hold latest sensor data values of the MPU_2 for all 5 fingers
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};             				  // vector to hold quaternion of the MPU_1 
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};             				  // vector to hold quaternion of the MPU_2

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)======================================================================
float pi = 3.141592653589793238462643383279502884f;	// Contant PI
float GyroMeasError = pi * (40.0f / 180.0f);        // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);        // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta1 = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;    // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// ESP32 GPIOs definitions and some variables ==========================================
uint8_t FD2[5], FSL[5], FR0[5];						// For assigning each FSLP pin to its respective finger.
uint8_t Finger[5];									      // Finger mux selection (Set buffer)
uint8_t Clear_buffer[5];							    // Finger mux selection (Clear buffer)
float pos_mm[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};	// Position buffer for each finger [mm]
float pressure[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Pressure buffer for each finger [Newtons]

// Variables for control ==============================================
bool newMagData = false;	  // Incomming data from magnetometers
bool buttonstate = false;   // Dataset printing state
bool lock = false;          // Avoid multiple entries by the button
uint8_t Closed_hand = 0;	  // If the hand is touching the object or not
gpio_config_t config_IO;    // Variable for ESP32 GPIO configurations

MPU9250 MPU9250; // instantiate MPU9250 class
FSLP FSLP;	     // Instantiate FSLP class

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(); 					    // set master mode, default on SDA/SCL   
  Wire.setClock(400000); 			// I2C frequency at 400 kHz
  delay(1000);
 
  // Finger 1 (Thumb [Dedão])--------------------------------------------------- (000)
  Finger[0] = 0;              // Select the finger via i2c
  FD2[0] = 25;                // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[0] = 36;                // FSLP Sense Line (Analaog INPUT)
  FR0[0] = 1;                 // Resistor 10 kOhms Digital INPUT/OUPUT
  
  // Finger 2 (Index [Indicador])----------------------------------------------- (001)
  Finger[1] =  6;             // Select the finger via i2c
  FD2[1] = 26;                // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[1] = 39;                // FSLP Sense Line (Analaog INPUT)
  FR0[1] = 3;                 // Resistor 10 kOhms Digital INPUT/OUPUT
  
  // Finger 3 (Middle [Meio])--------------------------------------------------- (011)
  Finger[2] =  3;         // Select the finger via i2c
  FD2[2] = 27;            // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[2] = 34;            // FSLP Sense Line (Analaog INPUT)
  FR0[2] = 19;            // Resistor 10 kOhms Digital INPUT/OUPUT
  
  // Finger 4 (Ring [Anelar])--------------------------------------------------- (010)
  Finger[3] =  2;         // Select the finger via i2c
  FD2[3] = 14;            // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[3] = 35;            // FSLP Sense Line (Analaog INPUT)
  FR0[3] = 18;            // Resistor 10 kOhms Digital INPUT/OUPUT
  
  // Finger 5 (Pinky [Mindinho])------------------------------------------------ (110)
  Finger[4] =  1;         // Select the finger via i2c
  FD2[4] = 12;            // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[4] = 32;            // FSLP Sense Line (Analaog INPUT)
  FR0[4] = 5;             // Resistor 10 kOhms Digital INPUT/OUPUT
    
  config_IO.mode = GPIO_MODE_INPUT;
  // Pins that are always inputs
  config_IO.pin_bit_mask = (1<<FSL[4])|(1<<FSL[3])|(1<<FSL[2])|(1<<FSL[1])|(1<<FSL[0])|(1<<button);
  gpio_config(&config_IO);

  config_IO.mode = GPIO_MODE_OUTPUT;
  // Pins that are always outputs
  config_IO.pin_bit_mask = (1<<Gled)|(1<<FD1)|(1<<M_A0)|(1<<M_A1)|(1<<M_A2);
  gpio_config(&config_IO);

  GPIO.out_w1tc = 0;  // Clear pins to select MUX (Set TCA address as 0x70)

  TCAscan();
  
  for(int i=0; i<5;i++){			      // Do this for each finger
	  MPU_select(Finger[i]);			  // Send selection via i2c to 0x70 address
	  Serial.println("-----------------------------------------------------------------------------\n");
	  Serial.print("Finger number: "); Serial.println(i);
	  vTaskDelay(100/portTICK_PERIOD_MS); // Wait 0.1 seconds
	  Serial.println(" ");	  
	  MPU9250.I2Cscan(); 			// should detect both MPU9250 at 0x75 and its magnetometers
	  Serial.println(" ");
	  
	  /* Configure the MPU9250 */
	  // Read the WHO_AM_I register, this is a good test of communication
	  Serial.println("Reading WHO_AM_I register...");
	  uint8_t c = MPU9250.getMPU9250ID(MPU1);		// MPU1 = address 0x68 | AD0 = 0
	  Serial.print("MPU9250_1 "); Serial.print("I AM: "); Serial.print(c, HEX); Serial.print(". I SHOULD BE: "); Serial.println(0x75, HEX);
	  uint8_t d = MPU9250.getMPU9250ID(MPU2);		// MPU2 = address 0x69 | AD0 = 1
	  Serial.print("MPU9250_2 "); Serial.print("I AM: "); Serial.print(d, HEX); Serial.print(". I SHOULD BE: "); Serial.println(0x75, HEX);
	  Serial.println(" ");
	  delay(250);
	  
	  if (c == 0x75 && d == 0x75 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x69 for MPU6050 
	  {  
		Serial.println("MPU9250_1 and MPU_9250_2 are online...");
		
		MPU9250.resetMPU9250(MPU1); // start by resetting MPU9250_1
		MPU9250.resetMPU9250(MPU2); // start by resetting MPU9250_2
		if(selftest_en){
			MPU9250.SelfTest(MPU1, SelfTest); // Start by performing self test and reporting values
			Serial.println("Self Test for MPU9250 #1:");
			Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
			MPU9250.SelfTest(MPU2, SelfTest); // Start by performing self test and reporting values
			Serial.println("Self Test for MPU9250 #2:");
			Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
			delay(200);
		}
		Serial.println(" ");
		
		if(gyro_accel_cal){ //If this variable is set, a new calibration will be made to both IMUs
			Serial.println("Gyro calibration = true");
			float gyroBias_temp[3], accelBias_temp[3];
			MPU9250.calibrateMPU9250(MPU1, gyroBias_temp, accelBias_temp); // Calibrate gyro and accelerometers, load biases in bias registers
			gyroBias1[i*3]=gyroBias_temp[0]; gyroBias1[i*3+1]=gyroBias_temp[1]; gyroBias1[i*3+2]=gyroBias_temp[2];
			accelBias1[i*3]=accelBias_temp[0]; accelBias1[i*3+1]=accelBias_temp[1]; accelBias1[i*3+2]=accelBias_temp[2]; 			
			
			MPU9250.calibrateMPU9250(MPU2, gyroBias_temp, accelBias_temp); // Calibrate gyro and accelerometers, load biases in bias registers
			gyroBias2[i*3]=gyroBias_temp[0]; gyroBias2[i*3+1]=gyroBias_temp[1]; gyroBias2[i*3+2]=gyroBias_temp[2];
			accelBias2[i*3]=accelBias_temp[0]; accelBias2[i*3+1]=accelBias_temp[1]; accelBias2[i*3+2]=accelBias_temp[2]; 
		}
		delay(200);
	  
		  MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 
		  MPU9250.initMPU9250(MPU2, Ascale, Gscale, sampleRate); 
		  Serial.println("MPU9250_1 e MPU9250_2 Initialized in read mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
		  
		  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		  byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
		  Serial.print("Magnetometer AK8963_1 "); Serial.print("I AM: "); Serial.print(e, HEX); Serial.print(" | I SHOULD BE: "); Serial.println(0x48, HEX);
		  byte f = MPU9250.getAK8963CID(MPU2);  // Read WHO_AM_I register for AK8963
		  Serial.print("AK8963 2 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" | I SHOULD BE: "); Serial.println(0x48, HEX);
		  delay(1000); 
		  
		  // Get magnetometer calibration from AK8963 ROM
		  float magCalibration_temp[3] = {0, 0, 0};		// Variable to store the recent calibration
		  MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration_temp); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
		  Serial.println("Calibration values for mag_1: ");
		  magCalibration1[i*3]   = magCalibration_temp[0];	// Store the calibration values into the buffer for all fingers
		  magCalibration1[i*3+1] = magCalibration_temp[1];
		  magCalibration1[i*3+2] = magCalibration_temp[2];
		  Serial.print("MPU1: X-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3], 2);
		  Serial.print("MPU1: Y-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3+1], 2);
		  Serial.print("MPU1: Z-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3+2], 2);

		  MPU9250.initAK8963Slave(MPU2, Mscale, Mmode, magCalibration_temp); Serial.println("AK8963 2 initialized for active data mode...."); // Initialize device 2 for active mode read of magnetometer
		  Serial.println("Calibration values for mag 2: ");
		  magCalibration2[i*3] = magCalibration_temp[0];
		  magCalibration2[i*3+1] = magCalibration_temp[1];
		  magCalibration2[3*i+2] = magCalibration_temp[2];
		  Serial.print("MPU2: X-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3], 2);
		  Serial.print("MPU2: Y-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3+1], 2);
		  Serial.print("MPU1: Z-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3+2], 2);
		  
		 // Comment out if using pre-measured, pre-stored offset biases
		  if(calibrationMag9250){
			  float magBias_temp[3],magScale_temp[3];
			  MPU9250.magcalMPU9250(MPU1, magBias_temp, magScale_temp); //Calibração com movimentos em 8
			  magBias1[i*3]=magBias_temp[0]; magBias1[i*3+1]=magBias_temp[1]; magBias1[i*3+2]=magBias_temp[2];
			  magScale1[i*3]=magScale_temp[0]; magScale1[i*3+1]=magScale_temp[1]; magScale1[i*3+2]=magScale_temp[2];
			  MPU9250.magcalMPU9250(MPU2, magBias_temp, magScale_temp);
			  magBias2[i*3]=magBias_temp[0]; magBias2[i*3+1]=magBias_temp[1]; magBias2[i*3+2]=magBias_temp[2];
			  magScale2[i*3]=magScale_temp[0]; magScale2[i*3+1]=magScale_temp[1]; magScale2[i*3+2]=magScale_temp[2];
		  } 
		  delay(200); // add delay to see results before serial spew of data
	  }
	  else
	  {
		Serial.print("MPU9250 1 Not recognized: 0x"); Serial.println(c, HEX);
		Serial.print("MPU9250 2 Not recognized: 0x"); Serial.println(d, HEX);
		Serial.print("Error on the finger number: "); Serial.println(i);
		while(1) ; // Loop forever if communication doesn't happen
	  }
	}
	
	// get sensor resolutions, only need to do this once, same for every MPU9250s
	aRes = MPU9250.getAres(Ascale);		// Accelerometer scale
	gRes = MPU9250.getGres(Gscale);		// Gyroscope scale
	mRes = MPU9250.getMres(Mscale);		// Magnetometer scale
	
	if(calibrationMag9250){
		Serial.print("Mag1: COPY THAT (Bias): {");
		for(int i=0; i<14; i++){
		  Serial.print(magBias1[i]); Serial.print(", ");
		} Serial.print(magBias1[14]); Serial.println("}");
		Serial.print("Mag1: COPY THAT (Scale): {");
		for(int i=0; i<14; i++){
		  Serial.print(magScale1[i]); Serial.print(", ");
		} Serial.print(magScale1[14]); Serial.println("}");
		Serial.print("Mag2: COPY THAT (Bias): {");
		for(int i=0; i<14; i++){
		  Serial.print(magBias2[i]); Serial.print(", ");
		} Serial.print(magBias2[14]); Serial.println("}");
		Serial.print("Mag2: COPY THAT (Scale): {");
		for(int i=0; i<14; i++){
		  Serial.print(magScale2[i]); Serial.print(", ");
		} Serial.print(magScale2[14]); Serial.println("}");
	}
	
	if(gyro_accel_cal){
		Serial.print("MPU1: COPY THAT (Gyro_bias) [dps]: {");
		for(int i=0; i<14; i++){
		  Serial.print(gyroBias1[i]); Serial.print(", ");
		} Serial.print(gyroBias1[14]); Serial.println("}");
		Serial.print("MPU1: COPY THAT (Accel_bias) [mg]: {");
		for(int i=0; i<14; i++){
		  Serial.print(1000*accelBias1[i]); Serial.print(", ");
		} Serial.print(1000*accelBias1[14]); Serial.println("}");
		Serial.print("MPU2: COPY THAT (Gyro_bias) [dps]: {");
		for(int i=0; i<14; i++){
		  Serial.print(gyroBias2[i]); Serial.print(", ");
		} Serial.print(gyroBias2[14]); Serial.println("}");
		Serial.print("MPU2: COPY THAT (Accel_bias) [mg]: {");
		for(int i=0; i<14; i++){
		  Serial.print(1000*accelBias2[i]); Serial.print(", ");
		} Serial.print(1000*accelBias2[14]); Serial.println("}");
	}
	
	Serial.println("----------------| Clean the Serial information and press the glove button to start the dataset |--------------------------");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{  
   
   // Check the button state -----------------------------------------------------------------------------
   int buttonstate_push = digitalRead(button);					// If the button pull-down is pressed
   if(buttonstate_push == HIGH && !lock){						// If it's set and not locked
     buttonstate = !buttonstate;								// Change the state
     lock = true;											    // Lock, to avoid multiple changes
   } else if(buttonstate_push == LOW && lock){lock = false;};	// Only unlock after releasing the button

     if (pressure[0] == 0 && pressure[1] == 0 && pressure[2] == 0 && pressure[3] == 0 && pressure[4] == 0){ // If there's no pressure in any finger
      Closed_hand = 0;											// Closing the hand
      pos_mm[0] = 0; pos_mm[1] = 0; pos_mm[2] = 0; pos_mm[3] = 0; pos_mm[4] = 0;				// There's no position either
      } else{Closed_hand = 1;}
    
	for(int iii=0; iii<5; iii++){								// Repeat for each finger
		vTaskDelay(100/portTICK_PERIOD_MS); // Wait 0.5 seconds
		MPU_select(Finger[iii]);
    //delay(100);
		MPU9250.readMPU9250Data(MPU1, MPU9250_1_data); 		// Read the first MPU data
		// Now we'll calculate the accleration value into actual g's
		 ax1[iii] = (float)MPU9250_1_data[0]*aRes - accelBias1[iii*3];  // get actual g value, this depends on scale being set
		 ay1[iii] = (float)MPU9250_1_data[1]*aRes - accelBias1[iii*3+1];   
		 az1[iii] = (float)MPU9250_1_data[2]*aRes - accelBias1[iii*3+2];  
		// Calculate the gyro value into actual degrees per second
		 gx1[iii] = (float)MPU9250_1_data[4]*gRes;			// get actual gyro value, this depends on scale being set
		 gy1[iii] = (float)MPU9250_1_data[5]*gRes;  
		 gz1[iii] = (float)MPU9250_1_data[6]*gRes; 
		if(MPU9250.checkNewMagData(MPU1) == true) {			// wait for magnetometer data ready bit to be set
		  MPU9250.readMagData(MPU1, magCount1);				// Read the x/y/z adc values
		}
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx1[iii] = (float)magCount1[0]*mRes*magCalibration1[iii*3] - magBias1[iii*3];  // get actual magnetometer value, this depends on scale being set
		my1[iii] = (float)magCount1[1]*mRes*magCalibration1[iii*3+1] - magBias1[iii*3+1];  
		mz1[iii] = (float)magCount1[2]*mRes*magCalibration1[iii*3+2] - magBias1[iii*3+2];  
		mx1[iii] *= magScale1[iii*3];
		my1[iii] *= magScale1[iii*3+1];
		mz1[iii] *= magScale1[iii*3+2];   
		for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
			Now1 = micros();
			deltat1 = ((Now1 - lastupdate1[iii])/1000000.0f); // set integration time by time elapsed since last filter update
			lastupdate1[iii] = Now1;
			MadgwickQuaternionUpdate1(-ax1[iii], ay1[iii], az1[iii], gx1[iii]*pi/180.0f, -gy1[iii]*pi/180.0f, -gz1[iii]*pi/180.0f,  my1[iii],  -mx1[iii], mz1[iii]);
		}
		a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
		a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
		a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
		a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
		a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
		pitch1[iii] = -asinf(a32);
		roll1[iii]  = atan2f(a31, a33);
		yaw1[iii]   = atan2f(a12, a22);
		pitch1[iii] *= 180.0f / pi;
		yaw1[iii]   *= 180.0f / pi; 
		yaw1[iii]   += 21.56f; // Declination at Sobradinho, Brasília at 21 degrees 56 minutes and 23 seconds on 31-01-2022
		if(yaw1[iii] < 0) yaw1[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
		roll1[iii]  *= 180.0f / pi;
		  
		MPU9250.readMPU9250Data(MPU2, MPU9250_2_data); // Read the data and store in MPU9250_2_data   
		// Now we'll calculate the accleration value into actual g's
		ax2[iii] = (float)MPU9250_2_data[0]*aRes - accelBias2[iii*3];  // get actual g value, this depends on scale being set
		ay2[iii] = (float)MPU9250_2_data[1]*aRes - accelBias2[iii*3+1];   
		az2[iii] = (float)MPU9250_2_data[2]*aRes - accelBias2[iii*3+2];  
		// Calculate the gyro value into actual degrees per second
		gx2[iii] = (float)MPU9250_2_data[4]*gRes - gyroBias2[iii*3];  // get actual gyro value, this depends on scale being set
		gy2[iii] = (float)MPU9250_2_data[5]*gRes - gyroBias2[iii*3+1];  
		gz2[iii] = (float)MPU9250_2_data[6]*gRes - gyroBias2[iii*3+2]; 
		if( MPU9250.checkNewMagData(MPU2) == true) { // wait for magnetometer data ready bit to be set
		MPU9250.readMagData(MPU2, magCount2);}  // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx2[iii] = (float)magCount2[0]*mRes*magCalibration2[0] - magBias2[iii*3];  // get actual magnetometer value, this depends on scale being set
	  my2[iii] = (float)magCount2[1]*mRes*magCalibration2[1] - magBias2[iii*3+1];  
    mz2[iii] = (float)magCount2[2]*mRes*magCalibration2[2] - magBias2[iii*3+2];  
    mx2[iii] *= magScale2[iii*3];
	  my2[iii] *= magScale2[iii*3+1];
	  mz2[iii] *= magScale2[iii*3+2];
		for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
		Now2 = micros();
		deltat2 = ((Now2 - lastupdate2[iii])/1000000.0f); // set integration time by time elapsed since last filter update
		lastupdate2[iii] = Now2;
		MadgwickQuaternionUpdate2(-ax2[iii], ay2[iii], az2[iii], gx2[iii]*pi/180.0f, -gy2[iii]*pi/180.0f, -gz2[iii]*pi/180.0f, my2[iii], -mx2[iii], mz2[iii]);
		//Serial.print(Q[0]);Serial.print(Q[1]);Serial.print(Q[2]);Serial.println(Q[3]);
		}             
	  AA12 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
		AA22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
		AA31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
		AA32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
		AA33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
		pitch2[iii] = -asinf(AA32);
		roll2[iii]  = atan2f(AA31, AA33);
		yaw2[iii]   = atan2f(AA12, AA22);
		pitch2[iii] *= 180.0f / pi;
		yaw2[iii]   *= 180.0f / pi; 
		yaw2[iii]   += 21.56f; // Declination at Sobradinho, Brasília at 21 degrees 56 minutes and 23 seconds on 31-01-2022
		if(yaw2[iii] < 0) yaw2[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
		roll2[iii]  *= 180.0f / pi;
	
	  pressure[iii] = FSLP.fslpGetPressure(FSL[iii], FD1, FD2[iii], FR0[iii]);
		if(pressure[iii] > 0){pressure[iii] = 0.01f*pressure[iii]*pressure[iii] + 0.5f*pressure[iii]+12;}
		int pos = FSLP.fslpGetPosition(FSL[iii], FD1, FD2[iii], FR0[iii]);
		pos_mm[iii] = pos*0.018315f;
	}

	/* ====================================================================================================                                   
	----------------------------------------|DATASET PRINTING|---------------------------------------------
	=======================================================================================================*/
	//}
	 
	//delay(65);
	if(SerialDebug && buttonstate) {	// If serial debug == true and the button is set
		digitalWrite(Gled, HIGH);		// Turn on the LED
		for(int i=0; i<5; i++){
			if(!onlyAngles){			// Only show the angles if set
				Serial.print((int)1000*ax1[i]);
				Serial.print(", "); Serial.print((int)1000*ay1[i]); 
				Serial.print(", "); Serial.print((int)1000*az1[i]);
				Serial.print(", "); Serial.print((int)1000*ax2[i]);  
				Serial.print(", "); Serial.print((int)1000*ay2[i]); 
				Serial.print(", "); Serial.print((int)1000*az2[i]);		
				Serial.print(", "); Serial.print( gx1[i], 2); 
				Serial.print(", "); Serial.print( gy1[i], 2); 
				Serial.print(", "); Serial.print( gz1[i], 2);
				Serial.print(", "); Serial.print( gx2[i], 2); 
				Serial.print(", "); Serial.print( gy2[i], 2); 
				Serial.print(", "); Serial.print( gz2[i], 2);		
				Serial.print(", "); Serial.print( (int)mx1[i] ); 
				Serial.print(", "); Serial.print( (int)my1[i] ); 
				Serial.print(", "); Serial.print( (int)mz1[i] );
				Serial.print(", "); Serial.print( (int)mx2[i] ); 
				Serial.print(", "); Serial.print( (int)my2[i] ); 
				Serial.print(", "); Serial.print( (int)mz2[i] );
        Serial.print(", ");
			}		
			Serial.print(roll1[i], 2);
			Serial.print(", ");Serial.print(pitch1[i], 2);
			Serial.print(", ");Serial.print(yaw1[i], 2);
			Serial.print(", ");Serial.print(roll2[i], 2);
			Serial.print(", ");Serial.print(pitch2[i], 2);		
			Serial.print(", ");Serial.print(yaw2[i], 2);
			if(!onlyAngles){			// Only show the angles if set
				Serial.print(", ");Serial.print(pressure[i]);
				Serial.print(", ");Serial.print(pos_mm[i]);			
			}
		}
			if(!onlyAngles){
				Serial.print(", ");Serial.print(Closed_hand);
				Serial.print(", ");Serial.println(object);
			} else {Serial.println(" ");}
    } else {digitalWrite(Gled, LOW);}	// Else, turn off the LED and stop printing the dataset
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void MPU_select(uint8_t i){
  if(i>7 || i<0) return;
  Wire.beginTransmission(TCA_addr);
  Wire.write(1 << i);
  Wire.endTransmission();
  }

void TCAscan(){
  Serial.println("\nTCAScanner ready!");
  for (uint8_t t=0; t<8; t++) {
    MPU_select(t);
    Serial.print("TCA Port #"); Serial.println(t);
    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCA_addr) continue;
        Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  Serial.println("\ndone");
}

void MadgwickQuaternionUpdate1(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta1 * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta1 * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta1 * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta1 * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat1;
            q2 += qDot2 * deltat1;
            q3 += qDot3 * deltat1;
            q4 += qDot4 * deltat1;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            
            // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

void MadgwickQuaternionUpdate2(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = Q[0], q2 = Q[1], q3 = Q[2], q4 = Q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat2;
            q2 += qDot2 * deltat2;
            q3 += qDot3 * deltat2;
            q4 += qDot4 * deltat2;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            Q[0] = q1 * norm;
            Q[1] = q2 * norm;
            Q[2] = q3 * norm;
            Q[3] = q4 * norm;
        }

// Wrap an angle in the range [-limit,+limit]
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}
