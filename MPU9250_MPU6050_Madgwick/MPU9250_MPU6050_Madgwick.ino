/*============= Cration of a dataset using a sensored glove (5 fingers) with 20 MPU9250===============================*/
/*  Created by Euler torres to Glove aplication 20/06/2022
 *  Lybrary by Sebastian Madgwick [Madgwick] https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 *  
 Reading, Calculation and estimation 2 IMU angles
  Estimation of angles (Roll, Pitch, Yall) of 2 MPUs 9250 using Magnetic, Angular Rate and Gravity (MARG),
  Estimation of force of the grip applied by each finger using FSLP (Force Sensing Linear Potentiometer)
  Estimation of the position of the object using FSLP (Force Sensing Linear Potentiometer)
*/
//
//#include "BluetoothSerial.h"
//
//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif
//
//BluetoothSerial SerialBT;

#include <SPI.h>                 // SPI communication library (not used)
#include "Wire.h"                // I²C communication library
#include "MPU9250.h"             // Header file wich includes all IMU communication
#include "MadgwickAHRS.h"        // Madwick Filter header file, for both IMU
#include "FSLP.h"                // FSLP header file, to get te measurements
#include <Adafruit_GFX.h>        // Adafruit library for OLED displays
#include <Adafruit_SSD1306.h>    // SSD1306 Display library

//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"

#define SerialDebug true         // Set true to get Serial output for debugging and dataset printing
#define onlyAngles  false         // Debug only the angles (Roll, Pitch, Yall & Roll2, Pitch 2, Yall2)

#define num_obj 21				         // Number of objects that is going to be tested					

#define stability_th 200         // Samples ignored before start dataset printing

#define button 15       			   // Buton to start printing Dataset
#define Gled   2         			   // Extra Led to indicate printing
#define M_A0   17         			 // Digital output for mux address selection (not used)
#define M_A1   16         			 // Digital output for mux address selection (not used)
#define M_A2   4         			   // Digital output for mux address selection (not used)
#define FD1    23                // FSLP D1 (common for all FSLPs) [Digital OUTPUT]

#define SCREEN_WIDTH 128         // OLED display width, in pixels
#define SCREEN_HEIGHT 64         // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1            // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define DEBOUNCETIME 10          // Maximum time for button bounce (ms)
#define confirmTime  1600        // Maximum time for confirm a choice

#define TCA_addr 0x70 			// Address for the i²c mux 

String object[num_obj] = {"NONE", "Laranja", "Cubo_magico", "Cartao", "Violao", "Controle_videogame", "Smartphone", "Garrafa", "Caneta", "Mouse", "Espremer_laranja", "Manipular_cubo_magico", "Manipular_cartao", "Violao_Do", "Violao_Re", "Violao_Mi", "Usar_Smartphone", "Manipular_controle", "Encher_garrafa", "Assinatura", "Cliques_mouse"};        // Set the object to be printed in dataset
String current_finger[5] = {"Dedo 1 - Polegar", "Dedo 2 - indicador", "Dedo 3 - Médio", "Dedo 4 - Anelar", "Dedo 5 - Mindinho"};

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
float 	aRes, gRes, mRes;      			 	        // scale resolutions per LSB for the sensors
int16_t MPU9250_1_data[7], MPU9250_2_data[7]; // used to read all 14 bytes at once from both MPU9250 accel/gyro/temperature of the specific finger
//int16_t magCount1[3], magCount2[3];      	    // Stores the 16-bit signed magnetometer sensor output
//float   magCalibration1[15];                  // Factory mag calibration and mag bias of all MPU1s
//float   magCalibration2[15];                  // Factory mag calibration and mag bias of all MPU2s
float   SelfTest[6];                     	    // holds results of gyro and accelerometer self test

//bool calibrationMag9250 = false;               // Set true to calibrate the MPU920 magnetometer in a new enviorment
bool selftest_en        = false;               // Set true to see if the MPU9250s correponds to the factory integrity
bool gyro_accel_cal     = false;               // Set true to calibrate the biases and scales of each MPU9250

//------------------------------------------------ IMPORTANT-------------------------------------------------------------------------------------------------------------
// These can be measured once and entered here or can be calculated each time the device is powered on 

// MPU1: Gyroscope Bias------------------------------------
float   gyroBias1[15] = {-2.60, -2.54, 0.15, 4.11, -2.04, -0.95, -13.37, -11.54, -0.92, 0.37, 0.13, 0.03, -1.16, -2.47, 0.61};

// MPU1: Accelerometer Bias------------------------------
float   accelBias1[15] = {0.37, -0.02, -0.12, 0.05, -0.01, 0.16, 0.02, 0.08, 0.05, 0.14, 0.04, 0.02, -0.30, 0.08, -0.11};

// MPU2: Gyroscope Bias-----------------------------------
float   gyroBias2[15] = {-0.60, 0.60, -0.43, -13.86, -2.63, 1.15, -5.40, -14.02, -0.14, 3.82, 1.14, -0.05, 16.65, -8.64, 1.01};

// MPU2: Accelerometer Bias------------------------------
float   accelBias2[15] = {0.06, 0.03, 0.01, -0.04, 0.00, -0.02, 0.20, -0.00, 0.02, -0.19, -0.08, -0.03, -0.13, 0.05, -0.35};


//Magnetometer biases for each axis to all MPU9250_1 (proximal)
//float   magBias1[15] = {-95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83};

//Magnetometer scales for each axis to all MPU9250_1 (proximal)
//float	magScale1[15]  = {0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22};

//Magnetometer biases for each axis to all MPU9250_2 (medium)
//float   magBias2[15] = {-95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83, -95.96, 296.88, -428.83};

//Magnetometer scales for each axis to all MPU9250_2 (medium)
//float	magScale2[15]  = {0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22, 0.86, 0.98, 1.22};

float pitch1[5], yaw1[5], roll1[5], pitch2[5], yaw2[5], roll2[5]; // absolute orientation
float a12, a22, a31, a32, a33;                                    // rotation matrix coefficients for Euler angles and gravity components MPU1
float AA12, AA22, AA31, AA32, AA33;                               // rotation matrix coefficients for Euler angles and gravity components MPU2

float ax1[5], ay1[5], az1[5], gx1[5], gy1[5], gz1[5];             // variables to hold latest sensor data values of the MPU_1 for all 5 fingers
//float mx1[5], my1[5], mz1[5]
float ax2[5], ay2[5], az2[5], gx2[5], gy2[5], gz2[5];			// variables to hold latest sensor data values of the MPU_2 for all 5 fingers
//float mx2[5], my2[5], mz2[5];

float q[5][4] = {{1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}}; // vector to hold all quaternions of the MPU_1 of each finger
float Q[5][4] = {{1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}};    // vector to hold all quaternions of the MPU_2 of each finger

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)======================================================================
float pi = 3.141592653589793238462643383279502884f;	// Contant PI

// ESP32 GPIOs definitions and some variables ==========================================
uint8_t FD2[5], FSL[5], FR0[5];						          // For assigning each FSLP pin to its respective finger.
uint8_t Finger[5];									                // Finger mux selection (Set buffer)
uint8_t Clear_buffer[5];							              // Finger mux selection (Clear buffer)
float pos_mm[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};	  // Position buffer for each finger [mm]
float pressure[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Pressure buffer for each finger [Newtons]

// Variables for control ==============================================
//bool newMagData = false;	                  // Incomming data from magnetometers
bool buttonstate = false;                   // Dataset printing state
bool lock = false;                          // Avoid multiple entries by the button
bool sel = true;                            // Selection of the user menu
bool currentState = false;                  // If the button is currently pressed or not (after debounce)
bool saveLastState;                         // 
bool menu_sel = false;						// Choice of the user
bool matlab_sim = false;                       // If the dataset is printed to Matlab simulation or not
bool led_status = false;
uint8_t Closed_hand = 0;	                  // If the hand is touching the object or not
gpio_config_t config_IO;                    // Variable for ESP32 GPIO configurations
volatile int numberOfButtonInterrupts = 0;  // Number of interrupts detected
volatile bool lastState;                    // Last state when the button was pressed
volatile uint32_t debounceTimeout = 0;      // Store debounce time
uint32_t saveDebounceTimeout;               // Time when the bounce stopped
uint8_t sample_count = 0;                   // Number of samples aquired (for estabilization matters)
int save;                                   // 
uint8_t obj_sel = 0;						// Object that is currently being held
unsigned long millisTime, setupTime;		// For printing the time of the data aquisition (matlab only)

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

MPU9250 MPU9250; // instantiate MPU9250 class
FSLP FSLP;	     // Instantiate FSLP class

void IRAM_ATTR handleButtonInterrupt() {
    portENTER_CRITICAL_ISR(&mux); 
      numberOfButtonInterrupts++;                 // Increasse the number of detected interrupts
      lastState = digitalRead(button);  
      debounceTimeout = xTaskGetTickCount();      //version of millis() that works from interrupt
    portEXIT_CRITICAL_ISR(&mux);
}

void setup()
{
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
  Serial.begin(115200);
  //SerialBT.begin("Luva ESP32"); //Bluetooth device name
//  Serial.println("The device started, now you can pair it with bluetooth!");
  delay(1000);
  Wire.begin(); 					// set master mode, default on SDA/SCL   
  Wire.setClock(400000); 			// I2C frequency at 400 kHz
  delay(1000);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Finger 1 (Thumb [Dedão])--------------------------------------------------- (000)
  Finger[0] = 0;              // Select the finger via i2c
  FD2[0] = 25;                // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[0] = 36;                // FSLP Sense Line (Analaog INPUT)
  FR0[0] = 19;                 // Resistor 10 kOhms Digital INPUT/OUPUT
  
  // Finger 2 (Index [Indicador])----------------------------------------------- (001)
  Finger[1] =  7;             // Select the finger via i2c
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
  FD2[4] = 33;            // FSLP D2 (Analog INPUT/OUTPUT)
  FSL[4] = 13;            // FSLP Sense Line (Analaog INPUT)
  FR0[4] = 19;            // Resistor 10 kOhms Digital INPUT/OUPUT

  pinMode(FD1, OUTPUT);
    
  config_IO.mode = GPIO_MODE_INPUT;
  // Pins that are always inputs
  //config_IO.pin_bit_mask = (1<<FSL[4])|(1<<FSL[3])|(1<<FSL[2])|(1<<FSL[1])|(1<<FSL[0])|(1<<button);
  config_IO.pin_bit_mask = (1<<button);
  gpio_config(&config_IO);

  attachInterrupt(digitalPinToInterrupt(button), handleButtonInterrupt, CHANGE);   // Will interrupt the program in every change (incluiding the bounce)

  config_IO.mode = GPIO_MODE_OUTPUT;
  // Pins that are always outputs
  config_IO.pin_bit_mask = (1<<Gled);//|(1<<FD1)|(1<<M_A0)|(1<<M_A1)|(1<<M_A2);
  gpio_config(&config_IO);

  GPIO.out_w1tc = 0;              // Clear pins to select MUX (Set TCA address as 0x70)

  TCAscan();                      // Show every device coneccted to the TCA board

  // Configuration menu, where the user needs to confirm which parameters will be calibrated

  MPU_select(5);      // Select the Display in the mux
  delay(100);
  display.clearDisplay();
  menu_sel = choose(" CONFIGURAR LUVA ?");
  digitalWrite(Gled, HIGH);    // Turn on the LED
  delay(1500);
  digitalWrite(Gled, LOW);     // Turn off the LED
  if (menu_sel){
	  selftest_en = choose("    INTEGRIDADE             DO CHIP");
	  digitalWrite(Gled, HIGH);    // Turn on the LED
	  delay(1500);
	  digitalWrite(Gled, LOW);     // Turn off the LED
	  gyro_accel_cal = choose("     GIROSCOPIO E         ACELEROMETRO");
	  digitalWrite(Gled, HIGH);    // Turn on the LED
	  delay(1500);
	  digitalWrite(Gled, LOW);     // Turn off the LED  
	  matlab_sim = choose("     SIMULACAO               MATLAB");
	  digitalWrite(Gled, HIGH);    // Turn on the LED
	  delay(1500);
	  digitalWrite(Gled, LOW);     // Turn off the LED
  }
  drawcalib_config("ESPERANDO CONEXÃO SERIAL", "Configuração inicial", false);
  display.display();
  while (!Serial);
  drawcalib_config("CHECANDO CONEXOES", "Configuração inicial", false);
  display.display();
  
  for(int i=0; i<5;i++){			    // Do this for each finger
	  MPU_select(Finger[i]);			  // Send selection via i2c to 0x70 address
	  Serial.println("-----------------------------------------------------------------------------\n");
	  Serial.print("Finger number: "); Serial.println(i);
    //drawcalib_config("Dedo numero ", "Configuracao das IMUs", false);
	  vTaskDelay(100/portTICK_PERIOD_MS); // Wait 0.1 seconds
	  Serial.println(" ");	  
	  MPU9250.I2Cscan(); 			      // should detect both MPU9250 at 0x75 and its magnetometers
	  Serial.println(" ");
	  
	  /* Configure the MPU9250 */
	  // Read the WHO_AM_I register, this is a good test of communication
	  Serial.println("Reading WHO_AM_I register...");
	  uint8_t c = MPU9250.getMPU9250ID(MPU1);		// MPU1 = address 0x68 | AD0 = 0
	  Serial.print("MPU9250_1 "); Serial.print("I AM: "); Serial.print(c, HEX); Serial.print(". I SHOULD BE: "); Serial.println(0x75, HEX);
	  uint8_t d = MPU9250.getMPU9250ID(MPU2);		// MPU2 = address 0x69 | AD0 = 1
	  Serial.print("MPU9250_2 "); Serial.print("I AM: "); Serial.print(d, HEX); Serial.print(". I SHOULD BE: "); Serial.println(0x75, HEX);
	  Serial.println(" ");
	  delay(10);
	  
	  if (c == 0x75 && d == 0x75 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x69 for MPU6050 
	  {  
		Serial.println("MPU9250_1 and MPU_9250_2 are online...");
		
		MPU9250.resetMPU9250(MPU1);               // start by resetting MPU9250_1
		MPU9250.resetMPU9250(MPU2);               // start by resetting MPU9250_2
		if(selftest_en){
			MPU9250.SelfTest(MPU1, SelfTest);       // Start by performing self test and reporting values
			Serial.println("Self Test for MPU9250 #1:");
			Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
			MPU9250.SelfTest(MPU2, SelfTest);       // Start by performing self test and reporting values
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
		
		if(gyro_accel_cal){                       //If this variable is set, a new calibration will be made to both IMUs    
			
			Serial.println("Gyro calibration = true");
			MPU_select(5);      // Select the Display in the mux
			drawcalib_config("ACELEROMETRO E GIROSCOPIO", current_finger[i].c_str(), false);
      display.display();
      
			MPU_select(Finger[i]);        // Send selection via i2c to 0x70 address
			float gyroBias_temp[3], accelBias_temp[3];
			
			MPU9250.calibrateMPU9250(MPU1, gyroBias_temp, accelBias_temp); // Calibrate gyro and accelerometers, load biases in bias registers
			gyroBias1[i*3]=gyroBias_temp[0]; gyroBias1[i*3+1]=gyroBias_temp[1]; gyroBias1[i*3+2]=gyroBias_temp[2];
			accelBias1[i*3]=accelBias_temp[0]; accelBias1[i*3+1]=accelBias_temp[1]; accelBias1[i*3+2]=accelBias_temp[2]; 			
			
			MPU9250.calibrateMPU9250(MPU2, gyroBias_temp, accelBias_temp); // Calibrate gyro and accelerometers, load biases in bias registers
			gyroBias2[i*3]=gyroBias_temp[0]; gyroBias2[i*3+1]=gyroBias_temp[1]; gyroBias2[i*3+2]=gyroBias_temp[2];
			accelBias2[i*3]=accelBias_temp[0]; accelBias2[i*3+1]=accelBias_temp[1]; accelBias2[i*3+2]=accelBias_temp[2]; 

		}
		delay(10);
	  
		  MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 
		  MPU9250.initMPU9250(MPU2, Ascale, Gscale, sampleRate); 
		  Serial.println("MPU9250_1 e MPU9250_2 Initialized in read mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
		  
		  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		  //byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
		  //Serial.print("Magnetometer AK8963_1 "); Serial.print("I AM: "); Serial.print(e, HEX); Serial.print(" | I SHOULD BE: "); Serial.println(0x48, HEX);
		  //byte f = MPU9250.getAK8963CID(MPU2);  // Read WHO_AM_I register for AK8963
		  // Serial.print("AK8963 2 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" | I SHOULD BE: "); Serial.println(0x48, HEX);
		  //delay(1000); 
		  
		  // Get magnetometer calibration from AK8963 ROM
		  //float magCalibration_temp[3] = {0, 0, 0};		// Variable to store the recent calibration
		  //MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration_temp); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
		  //Serial.println("Calibration values for mag_1: ");
		  //magCalibration1[i*3]   = magCalibration_temp[0];	// Store the calibration values into the buffer for all fingers
		  //magCalibration1[i*3+1] = magCalibration_temp[1];
		  //magCalibration1[i*3+2] = magCalibration_temp[2];
		  //Serial.print("MPU1: X-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3], 2);
		  //Serial.print("MPU1: Y-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3+1], 2);
		  //Serial.print("MPU1: Z-Axis sensitivity adjustment value for this finger "); Serial.println(magCalibration1[i*3+2], 2);

		  ////MPU9250.initAK8963Slave(MPU2, Mscale, Mmode, magCalibration_temp); Serial.println("AK8963 2 initialized for active data mode...."); // Initialize device 2 for active mode read of magnetometer
		  //Serial.println("Calibration values for mag 2: ");
		  //magCalibration2[i*3] = magCalibration_temp[0];
		  //magCalibration2[i*3+1] = magCalibration_temp[1];
		  //magCalibration2[3*i+2] = magCalibration_temp[2];
		  //Serial.print("MPU2: X-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3], 2);
		  //Serial.print("MPU2: Y-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3+1], 2);
		  //Serial.print("MPU1: Z-Axis sensitivity adjustment value "); Serial.println(magCalibration2[i*3+2], 2);
		  
		 // Comment out if using pre-measured, pre-stored offset biases
		//  if(calibrationMag9250){
        //MPU_select(5);      // Select the Display in the mux
        //drawcalib_config("Movimento em 8 - IMU 1", current_finger[i].c_str(), false);
        // display.display();
		//	  MPU_select(Finger[i]);        // Send selection via i2c to 0x70 address
		//	  float magBias_temp[3],magScale_temp[3];
		//	  MPU9250.magcalMPU9250(MPU1, magBias_temp, magScale_temp); //Calibração com movimentos em 8
		//	  magBias1[i*3]=magBias_temp[0]; magBias1[i*3+1]=magBias_temp[1]; magBias1[i*3+2]=magBias_temp[2];
		//	  magScale1[i*3]=magScale_temp[0]; magScale1[i*3+1]=magScale_temp[1]; magScale1[i*3+2]=magScale_temp[2];
		//	  
		//	  MPU_select(5);      // Select the Display in the mux
        //drawcalib_config("Movimento em 8 - IMU 2", current_finger[i].c_str(), false);
        //display.display();
        //MPU_select(Finger[i]);        // Send selection via i2c to 0x70 address
		//	  MPU9250.magcalMPU9250(MPU2, magBias_temp, magScale_temp);
		//	  magBias2[i*3]=magBias_temp[0]; magBias2[i*3+1]=magBias_temp[1]; magBias2[i*3+2]=magBias_temp[2];
		//	  magScale2[i*3]=magScale_temp[0]; magScale2[i*3+1]=magScale_temp[1]; magScale2[i*3+2]=magScale_temp[2];
		  //} 
		  //delay(15); // add delay to see results before serial spew of data
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

	Serial.print("Resolution Ascale:   ");
	Serial.println(aRes);
	Serial.print("Resolution Gscale:   ");
	Serial.println(gRes);
	//mRes = MPU9250.getMres(Mscale);		// Magnetometer scale
	
	//if(calibrationMag9250){
	//	Serial.print("Mag1: COPY THAT (Bias): {");
	//	for(int i=0; i<14; i++){
	//	  Serial.print(magBias1[i]); Serial.print(", ");
	//	} Serial.print(magBias1[14]); Serial.println("}");
	//	Serial.print("Mag1: COPY THAT (Scale): {");
	//	for(int i=0; i<14; i++){
	//	  Serial.print(magScale1[i]); Serial.print(", ");
	//	} Serial.print(magScale1[14]); Serial.println("}");
	//	Serial.print("Mag2: COPY THAT (Bias): {");
	//	for(int i=0; i<14; i++){
	//	  Serial.print(magBias2[i]); Serial.print(", ");
	//	} Serial.print(magBias2[14]); Serial.println("}");
	//	Serial.print("Mag2: COPY THAT (Scale): {");
	//	for(int i=0; i<14; i++){
	//	  Serial.print(magScale2[i]); Serial.print(", ");
	//	} Serial.print(magScale2[14]); Serial.println("}");
	//}
	
	if(gyro_accel_cal){
		Serial.println("COPY THAT:");
    Serial.println("// MPU1: Gyroscope Bias------------------------------------");
		Serial.print("float   gyroBias1[15] = {");
		for(int i=0; i<14; i++){
		  Serial.print(gyroBias1[i]); Serial.print(", ");
		} Serial.print(gyroBias1[14]); Serial.println("};");
    Serial.println("\n// MPU1: Accelerometer Bias------------------------------");
		Serial.print("float   accelBias1[15] = {");
		for(int i=0; i<14; i++){
		  Serial.print(accelBias1[i]); Serial.print(", ");
		} Serial.print(accelBias1[14]); Serial.println("};");
		//Serial.println("MPU1: COPY THAT:");
   Serial.println("\n// MPU2: Gyroscope Bias-----------------------------------");
		Serial.print("float   gyroBias2[15] = {");
		for(int i=0; i<14; i++){
		  Serial.print(gyroBias2[i]); Serial.print(", ");
		} Serial.print(gyroBias2[14]); Serial.println("};");
    Serial.println("\n// MPU2: Accelerometer Bias------------------------------");
		Serial.print("float   accelBias2[15] = {");
		for(int i=0; i<14; i++){
		  Serial.print(accelBias2[i]); Serial.print(", ");
		} Serial.print(accelBias2[14]); Serial.println("};");
	}
	
	Serial.println("----------------| Clean the Serial information select the object to be held |--------------------------");
	MPU_select(5);      // Select the Display in the mux
	drawcalib_config(object[obj_sel].c_str(), "Object selection", false);
  display.display();
	setupTime = millis();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{  
   
   // Check the button state -----------------------------------------------------------------------------
    portENTER_CRITICAL_ISR(&mux); // início da seção crítica
      save  = numberOfButtonInterrupts;
      saveDebounceTimeout = debounceTimeout;
      saveLastState  = lastState;
    portEXIT_CRITICAL_ISR(&mux); // fim da seção crítica
    currentState = digitalRead(button); //recupera o estado atual do botão

    //Update debounce time if the button has change state
    if(currentState != saveLastState)
    {
      saveDebounceTimeout = millis();
    } 

    if( (millis() - saveDebounceTimeout) > DEBOUNCETIME && (save != 0))
    {
         if(!currentState && !buttonstate) {              
              if(obj_sel<num_obj-1){
				        obj_sel++;			// Next object				 
			        }else{obj_sel = 0;}
				
				MPU_select(5);      // Select the Display in the mux
				drawcalib_config(object[obj_sel].c_str(), "Object selection", false);
        display.display();
        }else if (currentState && buttonstate){buttonstate = false;}
        //Serial.printf("Button Interrupt Triggered %d times, current State=%u, time since last trigger %dms\n", save, currentState, millis() - saveDebounceTimeout);
        portENTER_CRITICAL_ISR(&mux);  //início da seção crítica
        numberOfButtonInterrupts = 0; // reconhece que o botão foi pressionado e reseta o contador de interrupção //acknowledge keypress and reset interrupt counter
        portEXIT_CRITICAL_ISR(&mux); 	//fim da seção crítica
    }

	if(currentState && (millis() - saveDebounceTimeout) > confirmTime){
      buttonstate = true;			// Start calculations and printing
      digitalWrite(Gled, HIGH);    // Turn on the LED
      delay(1500);
      digitalWrite(Gled, LOW);    // Turn off the LED
    }

     if (pressure[0] == 0 && pressure[1] == 0 && pressure[2] == 0 && pressure[3] == 0 && pressure[4] == 0){ // If there's no pressure in any finger
      Closed_hand = 0;											// Closing the hand
      pos_mm[0] = 0; pos_mm[1] = 0; pos_mm[2] = 0; pos_mm[3] = 0; pos_mm[4] = 0;				// There's no position either
      } else{Closed_hand = 1;}
    	
	if(buttonstate) {
   
		millisTime = millis() - setupTime;
		
		for(int iii=0; iii<5; iii++){								// Repeat for each finger
			
			MPU_select(Finger[iii]);
		  
			MPU9250.readMPU9250Data(MPU1, MPU9250_1_data); 		// Read the first MPU data
			// Now we'll calculate the accleration value into actual g's
			 ax1[iii] = (float)MPU9250_1_data[0]*aRes - accelBias1[iii*3];  // get actual g value, this depends on scale being set
			 ay1[iii] = (float)MPU9250_1_data[1]*aRes - accelBias1[iii*3+1];   
			 az1[iii] = (float)MPU9250_1_data[2]*aRes - accelBias1[iii*3+2];  

			// Calculate the gyro value into actual degrees per second
			 gx1[iii] = (float)MPU9250_1_data[4]*gRes - gyroBias1[iii*3];			// get actual gyro value, this depends on scale being set
			 gy1[iii] = (float)MPU9250_1_data[5]*gRes - gyroBias1[iii*3+1];  
			 gz1[iii] = (float)MPU9250_1_data[6]*gRes - gyroBias1[iii*3+2];
			//if(MPU9250.checkNewMagData(MPU1) == true) {			// wait for magnetometer data ready bit to be set
			//  MPU9250.readMagData(MPU1, magCount1);				// Read the x/y/z adc values
			//}
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections
			//mx1[iii] = (float)magCount1[0]*mRes*magCalibration1[iii*3] - magBias1[iii*3];  // get actual magnetometer value, this depends on scale being set
			//my1[iii] = (float)magCount1[1]*mRes*magCalibration1[iii*3+1] - magBias1[iii*3+1];  
			//mz1[iii] = (float)magCount1[2]*mRes*magCalibration1[iii*3+2] - magBias1[iii*3+2];  
			//mx1[iii] *= magScale1[iii*3];
			//my1[iii] *= magScale1[iii*3+1];
			//mz1[iii] *= magScale1[iii*3+2]; 
			
			for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
				
				MadgwickAHRSupdateIMU(gx1[iii]*pi/180.0f, gy1[iii]*pi/180.0f, gz1[iii]*pi/180.0f, ax1[iii], ay1[iii], az1[iii], (float*)q, iii);
			}
     
			a12 =   2.0f * (q[iii][1] * q[iii][2] + q[iii][0] * q[iii][3]);
			//a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
			a31 =   2.0f * (q[iii][0] * q[iii][1] + q[iii][2] * q[iii][3]);
			a32 =   2.0f * (q[iii][1] * q[iii][3] - q[iii][0] * q[iii][2]);
			//a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
			a33 = 1 - 2*(q[iii][1]*q[iii][1]+q[iii][2]*q[iii][2]);
			a22 = 1 - 2*(q[iii][2]*q[iii][2]+q[iii][3]*q[iii][3]); 
			
			pitch1[iii] = asinf(a32);
			roll1[iii]  = atan2f(a31, a33);
			yaw1[iii]   = atan2f(a12, a22);
			pitch1[iii] *= 180.0f / pi;
			yaw1[iii]   *= 180.0f / pi;
			roll1[iii]  *= 180.0f / pi;
//      pitch1[iii] *= 2;
//      yaw1[iii]   *= 2;
//      roll1[iii]  *= 2;     
			//if(yaw1[iii] < 0) yaw1[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
//      if(pitch1[iii] < 0) pitch1[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
//      if(roll1[iii] < 0) roll1[iii]   += 360.0f; // Ensure yaw stays between 0 and 360

			  
			MPU9250.readMPU9250Data(MPU2, MPU9250_2_data); // Read the data and store in MPU9250_2_data   
			// Now we'll calculate the accleration value into actual g's
			ax2[iii] = (float)MPU9250_2_data[0]*aRes - accelBias2[iii*3];  // get actual g value, this depends on scale being set
			ay2[iii] = (float)MPU9250_2_data[1]*aRes - accelBias2[iii*3+1];   
			az2[iii] = (float)MPU9250_2_data[2]*aRes - accelBias2[iii*3+2];  
			// Calculate the gyro value into actual degrees per second
			gx2[iii] = (float)MPU9250_2_data[4]*gRes - gyroBias2[iii*3];  // get actual gyro value, this depends on scale being set
			gy2[iii] = (float)MPU9250_2_data[5]*gRes - gyroBias2[iii*3+1];  
			gz2[iii] = (float)MPU9250_2_data[6]*gRes - gyroBias2[iii*3+2]; 
			
			//if( MPU9250.checkNewMagData(MPU2) == true) { // wait for magnetometer data ready bit to be set
			//MPU9250.readMagData(MPU2, magCount2);}  // Read the x/y/z adc values
			//// Calculate the magnetometer values in milliGauss
			//// Include factory calibration per data sheet and user environmental corrections
			//mx2[iii] = (float)magCount2[0]*mRes*magCalibration2[0] - magBias2[iii*3];  // get actual magnetometer value, this depends on scale being set
			//my2[iii] = (float)magCount2[1]*mRes*magCalibration2[1] - magBias2[iii*3+1];  
			//mz2[iii] = (float)magCount2[2]*mRes*magCalibration2[2] - magBias2[iii*3+2];  
			//mx2[iii] *= magScale2[iii*3];
			//my2[iii] *= magScale2[iii*3+1];
			//mz2[iii] *= magScale2[iii*3+2];
			
			for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
				MadgwickAHRSupdateIMU(gx2[iii]*pi/180.0f, gy2[iii]*pi/180.0f, gz2[iii]*pi/180.0f, ax2[iii], ay2[iii], az2[iii], (float*)Q, iii);
			}             
			
			AA12 =   2.0f * (Q[iii][1] * Q[iii][2] + Q[iii][0] * Q[iii][3]);
			//AA22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
			AA31 =   2.0f * (Q[iii][0] * Q[iii][1] + Q[iii][2] * Q[iii][3]);
			AA32 =   2.0f * (Q[iii][1] * Q[iii][3] - Q[iii][0] * Q[iii][2]);
			//AA33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];

      AA33 = 1 - 2*(Q[iii][1]*Q[iii][1]+Q[iii][2]*Q[iii][2]);
      AA22 = 1 - 2*(Q[iii][2]*Q[iii][2]+Q[iii][3]*Q[iii][3]); 
     
			pitch2[iii] = asinf(AA32);
			roll2[iii]  = atan2f(AA31, AA33);
			yaw2[iii]   = atan2f(AA12, AA22);
			
			pitch2[iii] *= 180.0f / pi;
			yaw2[iii]   *= 180.0f / pi; 		
      roll2[iii]  *= 180.0f / pi;
//      pitch2[iii] *= 2;
//      yaw2[iii]   *= 2;
//      roll2[iii]  *= 2;
			//if(yaw2[iii] < 0) yaw2[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
//      if(pitch2[iii] < 0) pitch2[iii]   += 360.0f; // Ensure yaw stays between 0 and 360
//      if(roll2[iii] < 0) roll2[iii]   += 360.0f; // Ensure yaw stays between 0 and 360			
	  
		}
   
    //  ---------------------Pressure aquisition-----------------------------------------
    
    for (int k = 0; k < 5; k++){
      // Step 1 - Set up the appropriate drive line voltages.
      digitalWrite(FD1, HIGH);
  
      pinMode(FR0[k], OUTPUT);
      digitalWrite(FR0[k], LOW);
  
      pinMode(FSL[k], INPUT);
  
      pinMode(FD2[k], INPUT);

      // Step 2 - Wait for the voltage to stabilize.
      delayMicroseconds(10);

        // Step 3 - Take two measurements.
      int v1 = analogRead(FD2[k]);
      int v2 = analogRead(FSL[k]);

      if(v2 != v1){
        pressure[k] = 64.0 * v2 / (v1 - v2);
      }else {pressure[k] = 0;}

      if(k==4){pressure[k] = 64 * v2 /(4095 - v2);} // Damaged sensor

      // Conversion from Siemens to Newtons by interpolation (Calibration Program)
      pressure[k] = 0.00438449*pressure[k]*pressure[k] + 0.09705429*pressure[k] + 0.13164553;
    }

    //  ---------------------Position aquisition-----------------------------------------
    
    for (int k = 0; k < 5; k++){
      // Step 1 - Clear all charges 
       pinMode(FSL[k], OUTPUT);
      digitalWrite(FSL[k], LOW);     
      
      pinMode(FD1, OUTPUT);
      digitalWrite(FD1, LOW);
    
      pinMode(FD2[k], OUTPUT);
      digitalWrite(FD2[k], LOW);
    
      pinMode(FR0[k], OUTPUT);
      digitalWrite(FR0[k], LOW);

      // Step 2 - Set up appropriate drive line voltages.
      digitalWrite(FD1, HIGH);
      pinMode(FR0[k], INPUT);
      pinMode(FSL[k], INPUT);

      // Step 3 - Wait for the voltage to stabilize.
      delayMicroseconds(10);
    
      // Step 4 - Take the measurement.
      if(pressure[k]==0){pos_mm[k] = 0;} else{ 
        pos_mm[k] = analogRead(FSL[k]);
      }
      if(k==4){pos_mm[4] = pos_mm[3]*0.01221;}else{pos_mm[k] = pos_mm[k]*0.018315;} // faulty sensor counter measure ***** REMOVE WHEN FIXED *****
     
      
    }
    
	}
 
	/* ====================================================================================================                                   
	----------------------------------------|DATASET PRINTING|---------------------------------------------
	=======================================================================================================*/

  // Each iteration is aprxoimately 59ms. Therefore the sample frequency is 17 samples/s
  // For estabilization of the filtering, its recommended a 10 second treshold to start printing the effective data. that aprroximately 170 samples
  // 200 samples is not printed at the beggining (blinking LED) 

	if(buttonstate) {					// If the button is set
    if(sample_count>200){
      if(!matlab_sim){
    		for(int i=0; i<5; i++){
    			if(!onlyAngles){			// Only show the angles if set
    									          Serial.print(ax1[i]);
    				Serial.print(", "); Serial.print(ay1[i]); 
    				Serial.print(", "); Serial.print(az1[i]);
    				Serial.print(", "); Serial.print( gx1[i], 2); 
    				Serial.print(", "); Serial.print( gy1[i], 2); 
    				Serial.print(", "); Serial.print( gz1[i], 2);
    				Serial.print(", "); Serial.print(ax2[i]);  
    				Serial.print(", "); Serial.print(ay2[i]); 
    				Serial.print(", "); Serial.print(az2[i]);		
    				Serial.print(", "); Serial.print( gx2[i], 2); 
    				Serial.print(", "); Serial.print( gy2[i], 2); 
    				Serial.print(", "); Serial.print( gz2[i], 2);		
    				//Serial.print(", "); Serial.print( (int)mx1[i] ); 
    				//Serial.print(", "); Serial.print( (int)my1[i] ); 
    				//Serial.print(", "); Serial.print( (int)mz1[i] );
    				//Serial.print(", "); Serial.print( (int)mx2[i] ); 
    				//Serial.print(", "); Serial.print( (int)my2[i] ); 
    				//Serial.print(", "); Serial.print( (int)mz2[i] );
    			  Serial.print(", ");
    			}		
    			                    Serial.print(roll1[i], 2);
    			Serial.print(", "); Serial.print(pitch1[i], 2);
    			Serial.print(", "); Serial.print(yaw1[i], 2);
    			Serial.print(", "); Serial.print(roll2[i], 2);
    			Serial.print(", "); Serial.print(pitch2[i], 2);		
    			Serial.print(", "); Serial.print(yaw2[i], 2);
    			if(!onlyAngles){			// Only show the angles if set
    				Serial.print(", ");Serial.print(pressure[i]);
    				Serial.print(", ");Serial.print(pos_mm[i]);
           	Serial.print(", ");
    			}
    		}
    			if(!onlyAngles){
    				                   Serial.print(Closed_hand);
    				//Serial.print(", ");Serial.print(millisTime/1000);
    				Serial.print(", ");Serial.println(object[obj_sel]);
    			} else {Serial.println(" ");}
      } else {
            for(int i=0; i<5; i++){
            if(!onlyAngles){      // Only show the angles if set
                                Serial.print( ax1[i], 2);
            Serial.print("\t"); Serial.print( ay1[i], 2); 
            Serial.print("\t"); Serial.print( az1[i], 2);
            Serial.print("\t"); Serial.print( gx1[i], 2); 
            Serial.print("\t"); Serial.print( gy1[i], 2); 
            Serial.print("\t"); Serial.print( gz1[i], 2);
            Serial.print("\t"); Serial.print( ax2[i], 2);  
            Serial.print("\t"); Serial.print( ay2[i], 2); 
            Serial.print("\t"); Serial.print( az2[i], 2);   
            Serial.print("\t"); Serial.print( gx2[i], 2); 
            Serial.print("\t"); Serial.print( gy2[i], 2); 
            Serial.print("\t"); Serial.print( gz2[i], 2);   
  //          Serial.print(", "); Serial.print(mx1[i] ); 
  //          Serial.print(", "); Serial.print(my1[i] ); 
  //          Serial.print(", "); Serial.print(mz1[i] );
  //          Serial.print(", "); Serial.print(mx2[i] ); // The mpu used doesnt have a magnetometer (yes, it was a scam :T )
  //          Serial.print(", "); Serial.print(my2[i] ); 
  //          Serial.print(", "); Serial.print(mz2[i] );
            Serial.print("\t");
          }   
                              Serial.print(roll1[i], 2);
          Serial.print("\t"); Serial.print(pitch1[i], 2);
          Serial.print("\t"); Serial.print(yaw1[i], 2);
          Serial.print("\t"); Serial.print(roll2[i], 2);
          Serial.print("\t"); Serial.print(pitch2[i], 2);   
          Serial.print("\t"); Serial.print(yaw2[i], 2);
          if(!onlyAngles){      // Only show the angles if set
            Serial.print("\t");Serial.print(pressure[i]);
            Serial.print("\t");Serial.print(pos_mm[i]);
            Serial.print("\t");
          }
        } //Serial.println(" ");
          if(!onlyAngles){
                               Serial.print(Closed_hand);
            Serial.print("\t");Serial.println(millisTime);
          } else {Serial.println(" ");}
      }
    }else {if(sample_count % 15 == 0){digitalWrite(Gled, led_status); led_status = !led_status;}
          sample_count++; 
          //Serial.print(sample_count); Serial.print("\t"); Serial.print(sample_count % 15);Serial.print("\t"); Serial.println(digitalRead(Gled));
          } // Blink the LED to wait for filter estabilization 
  } else {digitalWrite(Gled, LOW); sample_count = 0;}	// Else, turn off the LED and stop printing the dataset
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

void drawcalib_config(const char msg[], const char title[], bool options){
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.write(title);
  
  display.setCursor(0,19);
  display.write(msg);
  
  if(options){
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setCursor(20,45);
    display.write("SIM");
    display.setCursor(84,45);
    display.write("NAO");
  }
  //display.display();
}

void checkbutton(){
 
}

bool choose(const char msg2[]){
  for(;;){
    portENTER_CRITICAL_ISR(&mux); // início da seção crítica
      save  = numberOfButtonInterrupts;
      saveDebounceTimeout = debounceTimeout;
      saveLastState  = lastState;
    portEXIT_CRITICAL_ISR(&mux); // fim da seção crítica

    currentState = digitalRead(button); //recupera o estado atual do botão

    //Update debounce time if the button has change state
    if(currentState != saveLastState)
    {
      saveDebounceTimeout = millis();
    } 
    //Serial.println("escolha");
    //se o tempo passado foi maior que o configurado para o debounce e o número de interrupções ocorridas é maior que ZERO (ou seja, ocorreu alguma), realiza os procedimentos
    if( (millis() - saveDebounceTimeout) > DEBOUNCETIME && (save != 0) )
    {
           if(!currentState) {              
              sel = !sel;
            }
            //Serial.printf("Button Interrupt Triggered %d times, current State=%u, time since last trigger %dms\n", save, currentState, millis() - saveDebounceTimeout);
            portENTER_CRITICAL_ISR(&mux);  //início da seção crítica
              numberOfButtonInterrupts = 0; // reconhece que o botão foi pressionado e reseta o contador de interrupção //acknowledge keypress and reset interrupt counter
            portEXIT_CRITICAL_ISR(&mux); //fim da seção crítica
    }
    
    if(sel){
      drawcalib_config(msg2, "Calibracao inicial  ", true);
      display.fillTriangle(5, 54, 5, 47, 15, 51, WHITE);//YES
      display.display();
    } else {
      drawcalib_config(msg2, "Calibracao inicial  ", true);
      display.fillTriangle(70, 55, 70, 47, 80, 51, WHITE);//NO
      display.display();
    }
    if(currentState && (millis() - saveDebounceTimeout) > confirmTime){
      return sel;
    }
  delay(5);
  }
}

// Wrap an angle in the range [-limit,+limit]
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}
