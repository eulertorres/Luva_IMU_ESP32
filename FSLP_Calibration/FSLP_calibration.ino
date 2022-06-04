// To measure position, the sense line must be connected to a
// pin capable of reading analog voltages.  For pressure,
// the sense line and drive line 2 must be connected to pins
// capable of reading analog voltages.  See the FSLP guide for
// more information.
const int fslpSenseLine = 25;
const int fslpDriveLine1 = 32;
const int fslpDriveLine2 = 33;
const int fslpBotR0 = 26;
const int button = 5;   // Buton to start calibration
const int Gled = 2;     // Extra Led to indicate printing
bool debug = true;      // Print values constantly ()
bool buttonstate = false; // printing stateno dataset
bool lock = false;      // Avoid multiple entries by the button
const int N = 16;
int n = 0;
int i = 0;
float str[N];
int sample = 0;

void setup()
{
  Serial.begin(9600);
  delay(250);
  pinMode(button,  INPUT);      // Button to start printing the measurements
  pinMode(Gled, OUTPUT);
}

void loop()
{
  if (!debug){
   int buttonstate_push = digitalRead(button);
   if(buttonstate_push == HIGH && i == 0){
     buttonstate = true;
     digitalWrite(Gled, HIGH);
     n = 0;
   } else if(i == N){
      buttonstate = false;
      i = 0;
      digitalWrite(Gled, LOW);
      Serial.print("X.append([");
      for (n = 0; n < (sizeof (str)/sizeof (str[0])) - 1; n++){
        Serial.print(str[n]); Serial.print(", ");
        }
      Serial.print(str[N-1]); Serial.print("]) #Medida (gramas agua): "); Serial.println(sample*50);
      sample++;
   }
  } else{
      float pressure;
      int position;
      pressure = fslpGetPressure();

      if (pressure == 0){
          position = 0;} else{
          position = fslpGetPosition();  // Raw reading, from 0 to 1023.
  }

      char report[80];
      sprintf(report, "pressure: %.5f   position: %5d\n",
      pressure, position);
      Serial.print(report);

  delay(20);
    }
   
  if (buttonstate){  
    float pressure; int position;
    str[n] = fslpGetPressure();
//    if (pressure == 0.0)
//    {
//      position = 0;
//    }
//    else
//    {
//      position = fslpGetPosition();  // Raw reading, from 0 to 1023.
//    }
    delay(1000);
    i++; n++;
  }
}

// This function follows the steps described in the FSLP
// integration guide to measure the position of a force on the
// sensor.  The return value of this function is proportional to
// the physical distance from drive line 2, and it is between
// 0 and 1023.  This function does not give meaningful results
// if fslpGetPressure is returning 0.
int fslpGetPosition()
{
  // Step 1 - Clear the charge on the sensor.
  pinMode(fslpSenseLine, OUTPUT);
  digitalWrite(fslpSenseLine, LOW);

  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, LOW);

  pinMode(fslpDriveLine2, OUTPUT);
  digitalWrite(fslpDriveLine2, LOW);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  // Step 2 - Set up appropriate drive line voltages.
  digitalWrite(fslpDriveLine1, HIGH);
  pinMode(fslpBotR0, INPUT);
  pinMode(fslpSenseLine, INPUT);

  // Step 3 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 4 - Take the measurement.
  return analogRead(fslpSenseLine);
}

// This function follows the steps described in the FSLP
// integration guide to measure the pressure on the sensor.
// The value returned is usually between 0 (no pressure)
// and 500 (very high pressure), but could be as high as
// 32736.
float fslpGetPressure()
{
  // Step 1 - Set up the appropriate drive line voltages.
  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, HIGH);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  pinMode(fslpSenseLine, INPUT);

  pinMode(fslpDriveLine2, INPUT);

  // Step 2 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 3 - Take two measurements.
  int v1 = analogRead(fslpDriveLine2);
  int v2 = analogRead(fslpSenseLine);

  // Step 4 - Calculate the pressure.
  // Detailed information about this formula can be found in the
  // FSLP Integration Guide.
  if (v1 == v2)
  {
    // Avoid dividing by zero, and return maximum reading.
    return 64 * 1023;
  }
  return 64.0 * v2 / (v1 - v2);
}
