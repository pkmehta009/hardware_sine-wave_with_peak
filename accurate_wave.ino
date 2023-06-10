const int analogPin = 39;  // Analog input pin
const int analogPin1 = 36;
const int analogPin2 = 34;

unsigned long currentMillis1;
unsigned long currentMillis2;
unsigned long currentMillis3;

int threshold = 100;          // Threshold value for zero-crossing detection
int previousAnalogValue = 0;  // Previous analog value
int previousAnalogValueY = 0;
int previousAnalogValueB = 0;
bool zeroCrossingDetected = false;  // Flag for zero-crossing detection
bool zeroCrossingDetectedY = false;
bool zeroCrossingDetectedB = false;
int freqncyR, freqncyY, freqncyB;
int time1;
int time2;
int time3;
int analogValue, analogValue1, analogValue2;

unsigned long previousMillis = 0;
const long interval = 1000;


////////////////////////////////////////////////////////////////////////////////////////
const int numReadings = 20;  // Number of readings to average
int readings[numReadings];   // Array to store the readings
int currentIndex = 0;        // Current index in the readings array
int peakValue = 0;           // Variable to store the peak value
int averageReading;
const int numReadingsY = 10;  // Number of readings to average
int readingsY[numReadingsY];  // Array to store the readings
int currentIndexY = 0;        // Current index in the readings array
int peakValueY = 0;           // Variable to store the peak value
int averageReadingY;




const int numReadingsB = 10;  // Number of readings to average
int readingsB[numReadingsB];  // Array to store the readings
int currentIndexB = 0;        // Current index in the readings array
int peakValueB = 0;           // Variable to store the peak value
int averageReadingB;



#define SAMPLING 300
#define VOFFSET 0.0
#define AMPLITUDE 1023  //300.6
#define REAL_VAC 270.5
float adc_max, adc_min;
float adc_vpp, adc_vppY, adc_vppB;
float R, Y, B;
float xvalue;
float yvalue;

#define mvalue 0.01178  // y = 68.14011*x + 30.57023
#define cvalue -0.2411

void setup() {
  Serial.begin(115200);       // Initialize serial communication
  pinMode(analogPin, INPUT);  // Set the analog pin as input
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
}

void loop() {

  //  averageReading = getAverageReading();
  // Print average and peak values
  // Serial.print("Average Reading: ");

  read_VAC();
  read_VACY();
  read_VACB();

  analogValue = analogRead(analogPin);
  analogValue1 = analogRead(analogPin1);
  analogValue2 = analogRead(analogPin2);


  if ((analogValue > threshold && previousAnalogValue < threshold) || (analogValue < threshold && previousAnalogValue > threshold)) {
    zeroCrossingDetected = true;
    freqncyR++;
    if (analogValue > threshold) {
      currentMillis1 = millis();
    }
    if (analogValue < threshold) {
      unsigned long currentMillis = millis();
      time1 = currentMillis - currentMillis1;
      currentMillis1 = 0;
    }
  }



  if ((analogValue1 > threshold && previousAnalogValueY < threshold) || (analogValue1 < threshold && previousAnalogValueY > threshold)) {
    zeroCrossingDetectedY = true;
    freqncyY++;
    if (analogValue1 > threshold) {
      currentMillis2 = millis();
    }
    if (analogValue1 < threshold) {
      unsigned long currentMillis = millis();
      time2 = currentMillis - currentMillis2;
      currentMillis2 = 0;
    }
  }



  if ((analogValue2 > threshold && previousAnalogValueB < threshold) || (analogValue2 < threshold && previousAnalogValueB > threshold)) {
    zeroCrossingDetectedB = true;
    freqncyB++;
    if (analogValue2 > threshold) {
      currentMillis3 = millis();
    }
    if (analogValue2 < threshold) {
      unsigned long currentMillis = millis();
      time3 = currentMillis - currentMillis3;
      currentMillis3 = 0;
    }
  }

  previousAnalogValue = analogValue;
  previousAnalogValueY = analogValue1;
  previousAnalogValueB = analogValue2;


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print(freqncyR / 2);
    Serial.print(",");
    Serial.print(freqncyY / 2);
    Serial.print(",");
    Serial.println(freqncyB / 2);
    time1 = 0;
    time2 = 0;
    time3 = 0;
    freqncyR = 0;
    freqncyY = 0;
    freqncyB = 0;
  }



  // Serial.print(getAverageReading());
  // Serial.print(",");
  // Serial.print(getAverageReadingY());
  // Serial.print(",");
  // Serial.println(getAverageReadingB());

  R = ((adc_vpp * 4.9 / 1023.0));
  Y = ((adc_vppY * 4.9 / 1023.0));
  B = ((adc_vppB * 4.9 / 1023.0));

  // Serial.print(R);
  // Serial.print(",");
  // Serial.print(Y);
  // Serial.print(",");
  // Serial.println(B);

  float real_voltageR = map(adc_vpp, 0, 603, 0, 233);
  float real_voltageY = map(adc_vppY, 0, 603, 0, 233);
  float real_voltageB = map(adc_vppB, 0, 603, 0, 233);

  // Serial.print(adc_vpp);
  // Serial.print(",");
  // Serial.print(adc_vppY);
  // Serial.print(",");
  // Serial.println(adc_vppB);

  // Serial.print(real_voltageY);
  // Serial.print(",");
  // Serial.println(real_voltageB);



  //  float temp1 = mvalue*x+cvalue;

  float vrms = (R * 261.3 / 3.05);
  float x2 = (x2 + (vrms * 15 / 100));
  // Serial.print(x);
  // Serial.print(",");
  // Serial.println(adc_vpp);
}

int getAverageReading() {
  int total = 0;
  // Shift readings in the array
  for (int i = 0; i < numReadings - 1; i++) {
    readings[i] = readings[i + 1];
    total += readings[i];
  }

  // Read new value and add it to the total
  readings[numReadings - 1] = analogValue;  //analogValue;
  total += readings[numReadings - 1];

  // Calculate average
  int average = total / numReadings;

  // Update peak value
  if (average > peakValue) {
    peakValue = average;
  }

  return average;
}

void read_VAC() {
  int cnt, cnt1;
  adc_max = 0;
  adc_min = 1023;

  for (cnt = 0; cnt < SAMPLING; cnt++) {
    // analogReadResolution(10);
    int adc = getAverageReading();
    // analogValue = adc;
    // Serial.println(analgpin0);
    if (adc > adc_max) {
      adc_max = adc;
    }
    if (adc < adc_min) {
      adc_min = adc;
    }
  }
  adc_vpp = adc_max - adc_min;
}




void read_VACY() {
  int cnt, cnt1;
  adc_max = 0;
  adc_min = 1023;

  for (cnt = 0; cnt < SAMPLING; cnt++) {
    // analogReadResolution(10);
    float adc = getAverageReadingY();

    // Serial.println(analgpin0);
    if (adc > adc_max) {
      adc_max = adc;
    }
    if (adc < adc_min) {
      adc_min = adc;
    }
  }
  adc_vppY = adc_max - adc_min;
}






void read_VACB() {
  int cnt, cnt1;
  adc_max = 0;
  adc_min = 1023;

  for (cnt = 0; cnt < SAMPLING; cnt++) {
    // analogReadResolution(10);
    float adc = getAverageReadingB();
    // Serial.println(analgpin0);
    if (adc > adc_max) {
      adc_max = adc;
    }
    if (adc < adc_min) {
      adc_min = adc;
    }
  }
  adc_vppB = adc_max - adc_min;
}

int getAverageReadingY() {
  int total = 0;

  // Shift readings in the array
  for (int i = 0; i < numReadingsY - 1; i++) {
    readingsY[i] = readingsY[i + 1];
    total += readingsY[i];
  }

  // Read new value and add it to the total
  readingsY[numReadingsY - 1] = analogValue1;
  total += readingsY[numReadingsY - 1];

  // Calculate average
  int average = total / numReadingsY;

  // Update peak value
  if (average > peakValueY) {
    peakValueY = average;
  }

  return average;
}

int getAverageReadingB() {
  int total = 0;

  // Shift readings in the array
  for (int i = 0; i < numReadingsB - 1; i++) {
    readingsB[i] = readingsB[i + 1];
    total += readingsB[i];
  }

  // Read new value and add it to the total
  readingsB[numReadingsB - 1] = analogValue2;
  total += readingsB[numReadingsB - 1];

  // Calculate average
  int average = total / numReadingsB;
  // Update peak value
  if (average > peakValueB) {
    peakValueB = average;
  }

  return average;
}