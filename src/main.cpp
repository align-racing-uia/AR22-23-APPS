
// Rele 1 - Pin 5 : Bremselys
// Rele 2 - Pin 4 : Vifte
// Rele 3 - Pin 3 : Pumpe
// Rele 4 - Pin 2 : AMS test relay

#include "Arduino.h"
#include "lib/mcp2515.h"

// Definitions
#define SHUTDOWN_CIRCUIT_PIN A3
#define READY_TO_DRIVE_INPUT A2
#define BREAK_SENSOR_PIN A4
#define AMS_LIGHT_RELAY 2
#define PUMP_PIN 3
#define FAN_PIN 4
#define BRAKELIGHT_PIN 5
#define BUZZER_OUTPUT_PIN 7
#define SENSOR_1_PIN A0 // Non inverted signal
#define SENSOR_2_PIN A1 // Inverted signal
#define SIGNAL_1_MAX 480
#define SIGNAL_1_MIN 260
#define SIGNAL_2_MAX 650
#define SIGNAL_2_MIN 440
#define MAX_TORQUE 1200
#define BITS_TO_BAR 0.122070313
#define BITS_OFFSET -100
#define APPS_CAN_ID 0x179
// Define canbus frame

struct can_frame commandedInverterMessage; // To send for one peace of data. Can be duplicated for other ID's and data
struct can_frame APPSCanMessage;
struct can_frame clearFaultsCanFrame;
struct can_frame canReceive;
MCP2515 can0(10); // Chip select

const int outPin = 12;          // declare pin for output
int out = 0;                    // declare reading for output
int sensor1;                    // declare reading for inverted signal
int sensor2;                    // declare reading for non-inverted signal
unsigned long previousTime = 0; // declare time before each iteration
unsigned long firstDeviadeTime = 0;
bool deviation = false;            // declare ERROR value to default = false
unsigned long currentTime = 0; // declare timer
const int light = 13;          // declare deviation light
int torque = 0;
unsigned long tempSendTime, reset_timer = 0, r2dSoundStartTime, brakeImplausibilityTime;
int shutdown_circuit_toggle, shutdown_circuit = 0, ready_to_drive_toggle, ready_to_drive, R2DS_toggled = 0, VSM_state = 0, VSM_toggled = 0;
int brakeImplausibility = 0;
void clearFaults()
{
  // Cascadia motion CAN Protocol page 39
  clearFaultsCanFrame.can_id = 0x0C1; // Parameter write
  clearFaultsCanFrame.can_dlc = 8;    // Length of the message

  //---CAN DATA START --- //
  clearFaultsCanFrame.data[0] = 0x14; // Fault clear parameter, Address 20 = 0x14
  clearFaultsCanFrame.data[1] = 0x00; // Commanded torque, last
  clearFaultsCanFrame.data[2] = 0x01; // Clear faults
  clearFaultsCanFrame.data[3] = 0x00; // Commanded speed, last(not used)
  clearFaultsCanFrame.data[4] = 0x00; // Not in use
  clearFaultsCanFrame.data[5] = 0x00; // Not in use
  clearFaultsCanFrame.data[6] = 0x00; // Not in use
  clearFaultsCanFrame.data[7] = 0x00; // Not in use
  //---CAN DATA END --- //
  can0.sendMessage(&clearFaultsCanFrame);
  // Serial.print("Faults cleared");
}
float getBrakePressure()
{
  int bits = analogRead(BREAK_SENSOR_PIN) + BITS_OFFSET;
  return bits * BITS_TO_BAR;
}
void setup()
{

  commandedInverterMessage.can_id = 0x0C0; // CANBUS ID
  commandedInverterMessage.can_dlc = 8;    // Length of the message
  //---CAN DATA START --- //
  commandedInverterMessage.data[0] = 0x00; // Commanded torque, first
  commandedInverterMessage.data[1] = 0x00; // Commanded torque, last
  commandedInverterMessage.data[2] = 0x00; // Commanded speed, first(not used)
  commandedInverterMessage.data[3] = 0x00; // Commanded speed, last(not used)
  commandedInverterMessage.data[4] = 0x01; // Commanded direction, 0 = reverse, 1 = forward
  commandedInverterMessage.data[5] = 0x00; // 5.0 Inverter enable(0 off, 1 on)
  commandedInverterMessage.data[6] = 0x00; // Not in use
  commandedInverterMessage.data[7] = 0x00; // Not in use
  //---CAN DATA END --- //

  APPSCanMessage.can_id = APPS_CAN_ID; // CANBUS ID
  APPSCanMessage.can_dlc = 8;          // Length of the message
  //---CAN DATA START --- //
  APPSCanMessage.data[0] = 0x00; // Commanded torque, first
  APPSCanMessage.data[1] = 0x00; // Commanded torque, last
  APPSCanMessage.data[2] = 0x00; // Commanded speed, first(not used)
  APPSCanMessage.data[3] = 0x00; // Commanded speed, last(not used)
  APPSCanMessage.data[4] = 0x00; // Commanded direction, 0 = reverse, 1 = forward
  APPSCanMessage.data[5] = 0x00; // 5.0 Inverter enable(0 off, 1 on)
  APPSCanMessage.data[6] = 0x00; // Not in use
  APPSCanMessage.data[7] = 0x00; // Not in use
  //---CAN DATA END --- //

  SPI.begin();

  can0.reset();
  can0.setBitrate(CAN_500KBPS); // Rate of CANBUS 500kbps for normal usage

  // Configure CAN-bus masks
  can0.setConfigMode();
  can0.setFilterMask(MCP2515::MASK0, false, 0xFFF);
  can0.setFilterMask(MCP2515::MASK1, false, 0xFFF);
  can0.setFilter(MCP2515::RXF0, false, 0x0AA);
  can0.setFilter(MCP2515::RXF1, false, 0x0A0);
  can0.setFilter(MCP2515::RXF2, false, 0x0AA);
  can0.setFilter(MCP2515::RXF3, false, 0x0AA);
  can0.setFilter(MCP2515::RXF4, false, 0x0AA);
  can0.setFilter(MCP2515::RXF5, false, 0x0AA);

  can0.setNormalMode();
  Serial.begin(9600); // start monitor for values
  Serial.println("");
  Serial.println((String) "SensorData,Sensor1,Sensor2,Deviation,Signal 1 Percent,Signal 2 Percent,Signal 1 value,Signal 2 Value,Brake signal,Brake pressure, V1, V2");
  Serial.println("Control,Torque,VSM_State,Shutdown Circuit,Ready to Drive,Buzzer,Deviation,Brake implosibility,R2DS");

  pinMode(SHUTDOWN_CIRCUIT_PIN, INPUT);
  pinMode(READY_TO_DRIVE_INPUT, INPUT);
  pinMode(BUZZER_OUTPUT_PIN, OUTPUT);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(AMS_LIGHT_RELAY, OUTPUT);
}

void loop()
{
  float brakePressure = getBrakePressure();
  shutdown_circuit_toggle = (shutdown_circuit != digitalRead(SHUTDOWN_CIRCUIT_PIN) ? 1 : 0);
  shutdown_circuit = digitalRead(SHUTDOWN_CIRCUIT_PIN);

  ready_to_drive_toggle = (ready_to_drive != digitalRead(READY_TO_DRIVE_INPUT) ? 1 : 0);
  ready_to_drive = digitalRead(READY_TO_DRIVE_INPUT);

  // --- CANBUS read ---
  // TODO If in VSM state below 4, set R2DS_toggled 0
  if (can0.readMessage(&canReceive) == MCP2515::ERROR_OK)
  {
    // frame contains received message
    if (canReceive.can_id == 0x0AA)
    {
      VSM_toggled = (VSM_state != canReceive.data[0] ? 1 : 0);
      VSM_state = canReceive.data[0];
    }
  }

  if (VSM_state < 4 || ready_to_drive == 0 || shutdown_circuit == 0)
    R2DS_toggled = 0;
  sensor1 = analogRead(SENSOR_1_PIN); // read inverted signal
  sensor2 = analogRead(SENSOR_2_PIN); // read non-inverted signal
  int sg1_val = map(constrain(sensor1, SIGNAL_1_MIN, SIGNAL_1_MAX), SIGNAL_1_MIN, SIGNAL_1_MAX, 0, MAX_TORQUE);
  int sg2_val = map(constrain(sensor2, SIGNAL_2_MIN, SIGNAL_2_MAX), SIGNAL_2_MIN, SIGNAL_2_MAX, MAX_TORQUE, 0);
  int sg1_percent = map(constrain(sensor1, SIGNAL_1_MIN, SIGNAL_1_MAX), SIGNAL_1_MIN, SIGNAL_1_MAX, 0, 100);
  int sg2_percent = map(constrain(sensor2, SIGNAL_2_MIN, SIGNAL_2_MAX), SIGNAL_2_MIN, SIGNAL_2_MAX, 100, 0);
  // Print APPS state to serial console
  if (millis() - tempSendTime > 100)
  {
    Serial.println((String) "SensorData," + sensor1 + "," + sensor2 + "," + abs(sg1_percent - sg2_percent) + "," + sg1_percent + "," + sg2_percent + "," + sg1_val + "," + sg2_val + "," + analogRead(BREAK_SENSOR_PIN) + "," + brakePressure + "," + sensor1 * 0.00488758553274682 + "," + sensor2 * 0.00488758553274682);
  }
  // Check if APPS is deviating more than 10%
  if (deviation == false)
  {
    if ((abs(sg2_percent - sg1_percent) > 10))
    { // check if difference between signals is more than 10%, do this:
      if (firstDeviadeTime != 0)
        firstDeviadeTime = millis();
      if (millis() - firstDeviadeTime > 100)
      {               // if they deviate for more than 100 ms:
        deviation = true; // set ERROR to true
      }
    }
    else
    {
      firstDeviadeTime = 0;
    }
  }
  int avgPercent = (sg1_percent + sg2_percent) / 2;

  // Reset APPS
  if (ready_to_drive_toggle && ready_to_drive == 0)
  {
    deviation = false;
    brakeImplausibility = 0;
  }
  // --- Inverter ---
  // Turn off relays if TSMS is toggled off.
  // Reset relays if deviation has occured during precharge
  if ((shutdown_circuit_toggle || ready_to_drive_toggle) && shutdown_circuit == 1 && ready_to_drive == 0 && VSM_state == 7)
  {
    // Disable inverter
    commandedInverterMessage.data[5] = 0x00;
    can0.sendMessage(&commandedInverterMessage);
    delay(200);
    clearFaults();
  }
  // Enable or disable inverter based on switch
  if (ready_to_drive == 0 || shutdown_circuit == 0)
    commandedInverterMessage.data[5] = 0x00;
  else if (ready_to_drive == 1 && shutdown_circuit == 1)
    commandedInverterMessage.data[5] = 0x01;

  // Turn on ready to drive sound
  if (R2DS_toggled == 0 && ready_to_drive == 1 && avgPercent == 0 && shutdown_circuit == 1 && VSM_state >= 4 && ready_to_drive_toggle && brakePressure >= 10)
  {
    r2dSoundStartTime = millis();
    R2DS_toggled = 1;
  }
  if ((millis() - r2dSoundStartTime >= 500) % 2 < 1)
    digitalWrite(BUZZER_OUTPUT_PIN, HIGH);

  else if ((millis() - r2dSoundStartTime >= 500) % 2 > 1)
    digitalWrite(BUZZER_OUTPUT_PIN, LOW);

  // Turn off ready to drive sound
  if ((millis() - r2dSoundStartTime >= 2000) || ready_to_drive == 0)
  {
    digitalWrite(BUZZER_OUTPUT_PIN, LOW);
  }

  // Set speed
  // Take the average of both sensors and use as torque
  torque = (sg1_val + sg2_val) / 2;
  commandedInverterMessage.data[0] = (uint8_t)(torque & 0x00FF);      // Commanded torque, first
  commandedInverterMessage.data[1] = (uint8_t)((torque >> 8) & 0xFF); // Commanded torque, last


  // Toggle off
  shutdown_circuit_toggle = 0;
  ready_to_drive_toggle = 0;
  VSM_toggled = 0;

  // TODO Bremsetrykk
  // Hvis over 30 bar, torque = 0
  // Hvis out of range, short eller VCC, torque = 0
  if (brakePressure >= 30 && avgPercent >= 25)
  {
    if (brakeImplausibilityTime == 0)
      brakeImplausibilityTime = millis();
    else if (brakeImplausibilityTime - millis() >= 500)
      brakeImplausibility = 1;
  }
  else
  {
    if (brakeImplausibility == 1 && brakeImplausibilityTime != 0 && torque == 0)
    {
      brakeImplausibilityTime = 0;
      brakeImplausibility = 0;
    }
  }
  // Brakelight
  if (brakePressure >= 4)
  {
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(5, LOW);
  }

  // Temp converter send R2D state
  uint8_t states = ready_to_drive;
  states = (states << 1 ) + shutdown_circuit;
  states = (states << 1 ) + brakeImplausibility;
  states = (states << 1 ) + deviation;
  states = (states << 4);
  APPSCanMessage.data[0] = states;
  

  APPSCanMessage.data[1] = (uint8_t)((int)(brakePressure * 10) & 0x00FF);      // Brake pressure, first
  APPSCanMessage.data[2] = (uint8_t)(((int)(brakePressure * 10) >> 8) & 0xFF); // Brake pressure, last

  APPSCanMessage.data[3] = (uint8_t)((int)(sensor1)&0x00FF);        // Analog input sensor 1, first
  APPSCanMessage.data[4] = (uint8_t)(((int)(sensor1) >> 8) & 0xFF); // Analog input sensor 1, last

  APPSCanMessage.data[5] = (uint8_t)((int)(sensor2)&0x00FF);        // Analog input sensor 2, first
  APPSCanMessage.data[6] = (uint8_t)(((int)(sensor2) >> 8) & 0xFF); // Analog input sensor 2, last


  // AMS light working test
  if(millis() <= 2000){
    digitalWrite(AMS_LIGHT_RELAY, HIGH);
  }
  else{
    digitalWrite(AMS_LIGHT_RELAY, LOW);
  }

  // Fan controller
  if(ready_to_drive  == 1) {
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(PUMP_PIN, LOW);
  }
  else{
    digitalWrite(FAN_PIN, HIGH);
    digitalWrite(PUMP_PIN, HIGH);

  }
  // TODO add brake sensor above 25% shutdown everything
  if (deviation == true || shutdown_circuit == 0 || ready_to_drive == 0 || brakeImplausibility == 1 || R2DS_toggled == 0 || millis() - r2dSoundStartTime <= 2000)
  {
    commandedInverterMessage.data[0] = 0x00; // Commanded torque, first
    commandedInverterMessage.data[1] = 0x00; // Commanded torque, last
    commandedInverterMessage.data[5] = 0x00; // 5.0 Inverter enable(0 off, 1 on)
    // Serial.print("Error somewhere");
  }
  // --- CAN-BUS ---
  if (millis() - tempSendTime > 100)
  {
    Serial.println((String) "Control," + torque + "," + VSM_state + "," + shutdown_circuit + "," + ready_to_drive + "," + digitalRead(BUZZER_OUTPUT_PIN) + "," + deviation + "," + brakeImplausibility + "," + R2DS_toggled);
    can0.sendMessage(&commandedInverterMessage);
    can0.sendMessage(&APPSCanMessage);
    tempSendTime = millis();
  }
}
