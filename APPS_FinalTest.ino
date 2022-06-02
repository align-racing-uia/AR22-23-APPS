// Old sensors had 15-360 degrees, so values must be: 315 to 1023 | 1 to 708
// Accelerator Pedal Position Sensor
int sg1 = A0;                       // declare pin for inverted signal
int sg2 = A1;                       // declare pin for non-inverted signal
const int outPin = 12;              // declare pin for output
int out = 0;                        // declare reading for output
int sensor1;                        // declare reading for inverted signal
int sensor2;                        // declare reading for non-inverted signal
unsigned long previousTime = 0;     // declare time before each iteration
long interval = 99;                 // declare interval (100 ms)
bool error = false;                 // declare ERROR value to default = false
unsigned long  currentTime = 0;     // declare timer
const int light = 13;               // declare error light
const double offset2 = 2.3668;      // offset for non-inverted signal
const double offset1 = 1.1105;      // offset for inverted signal
const double offset2_2 = 2.7222;    // offset for non-inverted signal
const double offset1_2 = 1.6667;    // offset for inverted signal
const double gain = 2.5;            // GAIN

void setup() {
//pinMode(outPin, OUTPUT);       // set output to right mode
Serial.begin(9600);              // start monitor for values
pinMode(light, OUTPUT);          // set output for error light
digitalWrite(light, LOW);        // set light to low
}

void loop() {
sensor1 = (analogRead(sg1)*(5/1023))/gain + offset1_2;  // read inverted signal
sensor2 = (analogRead(sg2)*(5/1023))/gain + offset2_2;  // read non-inverted signal

double sg2_percent = (sensor2-offset2_2)*163;              // convert to percent for non-inverted
double sg1_percent = 100-((sensor1-offset1_2)*90);         // convert to percent for inverted

Serial.print("Percent: ");
Serial.print(sg1_percent);                          // print percent for inverted signal
Serial.print(" and ");
Serial.print(sg2_percent);                          // print percent for non-inverted signal

Serial.print("          ");

Serial.print(1023-sensor1);                          // print inverted signal, inverted to normal
Serial.print(" and ");
Serial.print(sensor2);                               // print non-inverted signal

Serial.print("          ");

Serial.print(sensor1);
Serial.print(" and ");
Serial.println(sensor2);

if (error == false) {
  if ((abs(sg2_percent - sg1_percent) > 10)) {               // check if difference between signals is more than 10%, do this:
    currentTime = millis();                                  // timer
    if (currentTime  > interval) {                           // if they deviate for more than 100 ms:
    error = true;                                            // set ERROR to true
    }
    } 

  if (sg2_percent < 0 || sg2_percent > 100 || sg1_percent < 0|| sg1_percent > 100)     // check if the signals are too low/high
    { error = true; }                                                                  // set ERROR to true
    
  else {                                                                               // if nothing is wrong:
    if (sg1_percent > 1 ) {
      out = sg1_percent;              // sends signal from 0-100% to CANbus
      analogWrite(outPin, out);       // FIX THIS IS NOT CANBUS
    }
  }
}

else {                                                                       // if error is true
  Serial.println(" ERROR ");                                               // print ERROR to monitor
  digitalWrite(light, HIGH);                                                 // error light
  }
}
