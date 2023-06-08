int analogPin = 39;              // Analog input pin
int threshold = 100;            // Threshold value for zero-crossing detection
int previousAnalogValue = 0;    // Previous analog value
bool zeroCrossingDetected = false;  // Flag for zero-crossing detection
int freqncy;
unsigned long previousMillis = 0;
const long interval = 1000;

unsigned long currentMillis1;
int time1;


const int numReadings = 20;  // Number of readings to average
int readings[numReadings];   // Array to store the readings
int currentIndex = 0;        // Current index in the readings array
int peakValue = 0;           // Variable to store the peak value
int averageReading;




void setup() {
  pinMode(analogPin, INPUT);    // Initialize analog input pin
  Serial.begin(115200);           // Initialize serial communication
}

void loop() {
  
  int analogValue = getAverageReading();

  // Detect zero-crossings
  if ((analogValue > threshold && previousAnalogValue < threshold) ||
      (analogValue < threshold && previousAnalogValue > threshold)) {
    zeroCrossingDetected = true;
    freqncy++;
    if(analogValue > threshold){
    currentMillis1 = millis();
    }
    if(analogValue < threshold)
    {
      unsigned long currentMillis = millis();
      time1 = currentMillis-currentMillis1;
      currentMillis1=0;
    }
  }


  // Update previous analog value
  previousAnalogValue = analogValue;

  // Delay for stability
  // delay(10);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print((freqncy/2)+1);
    Serial.print(",");
    Serial.println(time1);
    time1=0;
    freqncy =0;
  }
}


int getAverageReading() {
  int total = 0;
  // Shift readings in the array
  for (int i = 0; i < numReadings - 1; i++) {
    readings[i] = readings[i + 1];
    total += readings[i];
  }

  // Read new value and add it to the total
  // analogReadResolution(10);
  readings[numReadings - 1] = analogRead(analogPin);
  total += readings[numReadings - 1];

  // Calculate average
  int average = total / numReadings;

  // Update peak value
  if (average > peakValue) {
    peakValue = average;
  }

  return average;
}
