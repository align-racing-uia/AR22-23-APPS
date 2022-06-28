#include "Arduino.h"
#include "lib/mcp2515.h"

// Definitions
#define SHUTDOWN_CIRCUIT_PIN A3
#define READY_TO_DRIVE_INPUT A2
#define BUZZER_OUTPUT_PIN A4
#define SENSOR_1_PIN A0 // Non inverted signal
#define SENSOR_2_PIN A1 // Inverted signal
#define SIGNAL_1_MAX 492
#define SIGNAL_1_MIN 440
#define SIGNAL_2_MAX 558
#define SIGNAL_2_MIN 510
#define MAX_TORQUE 700
enum RELAY_RESET_MODES
{
  RELAY_RESET_OFF,
  RELAY_RESET_FIRST,
  RELAY_RESET_ENABLE,
  RELAY_RESET_DISABLE
};
int RELAY_RESET_STATE = RELAY_RESET_OFF;

// Define canbus frame

struct can_frame commandedInverterMessage; // To send for one peace of data. Can be duplicated for other ID's and data
struct can_frame clearFaultsCanFrame;
struct can_frame canReceive;
MCP2515 can0(10); // Chip select

const int outPin = 12;          // declare pin for output
int out = 0;                    // declare reading for output
int sensor1;                    // declare reading for inverted signal
int sensor2;                    // declare reading for non-inverted signal
unsigned long previousTime = 0; // declare time before each iteration
unsigned long firstDeviadeTime = 0;
bool error = false;            // declare ERROR value to default = false
unsigned long currentTime = 0; // declare timer
const int light = 13;          // declare error light
int torque = 0;
unsigned long tempSendTime, reset_timer = 0, r2dSoundStartTime;
int shutdown_circuit_toggle, shutdown_circuit = 0, ready_to_drive_toggle, ready_to_drive, R2DS_toggled = 0, VSM_state = 0, VSM_toggled = 0;

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
  Serial.print("Faults cleared");
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

  SPI.begin();

  can0.reset();
  can0.setBitrate(CAN_500KBPS); // Rate of CANBUS 500kbps for normal usage

  // Configure CAN-bus masks
  can0.setConfigMode();
  can0.setFilterMask(MCP2515::MASK0, false, 0xFFF);
  can0.setFilterMask(MCP2515::MASK1, false, 0xFFF);
  can0.setFilter(MCP2515::RXF0, false, 0x0AA);
  can0.setFilter(MCP2515::RXF1, false, 0x0AA);
  can0.setFilter(MCP2515::RXF2, false, 0x0AA);
  can0.setFilter(MCP2515::RXF3, false, 0x0AA);
  can0.setFilter(MCP2515::RXF4, false, 0x0AA);
  can0.setFilter(MCP2515::RXF5, false, 0x0AA);

  can0.setNormalMode();
  // pinMode(outPin, OUTPUT);       // set output to right mode
  Serial.begin(9600);       // start monitor for values
  pinMode(light, OUTPUT);   // set output for error light
  digitalWrite(light, LOW); // set light to low
  pinMode(SHUTDOWN_CIRCUIT_PIN, INPUT);
  pinMode(READY_TO_DRIVE_INPUT, INPUT);
  pinMode(BUZZER_OUTPUT_PIN, OUTPUT);
}

void loop()
{
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
      R2DS_toggled = (VSM_state < 4 ? 0 : R2DS_toggled);
    }
  }

  sensor1 = analogRead(SENSOR_1_PIN); // read inverted signal
  sensor2 = analogRead(SENSOR_2_PIN); // read non-inverted signal
  int sg1_val = map(constrain(sensor1, SIGNAL_1_MIN, SIGNAL_1_MAX), SIGNAL_1_MIN, SIGNAL_1_MAX, 0, MAX_TORQUE);
  int sg2_val = map(constrain(sensor2, SIGNAL_2_MIN, SIGNAL_2_MAX), SIGNAL_2_MIN, SIGNAL_2_MAX, MAX_TORQUE, 0);
  int sg1_percent = map(constrain(sensor1, SIGNAL_1_MIN, SIGNAL_1_MAX), SIGNAL_1_MIN, SIGNAL_1_MAX, 0, 100);
  int sg2_percent = map(constrain(sensor2, SIGNAL_2_MIN, SIGNAL_2_MAX), SIGNAL_2_MIN, SIGNAL_2_MAX, 100, 0);
  // Print APPS state to serial console
  if (millis() - tempSendTime > 100)
  {
    Serial.print(analogRead(SENSOR_1_PIN));
    Serial.print(";");
    Serial.print(analogRead(SENSOR_2_PIN));
    Serial.println("");
    Serial.print("Percent total: ");
    Serial.print(abs(sg1_percent - sg2_percent)); // print percent for inverted signal
    Serial.print(" ");
    Serial.print(sg1_percent); // print percent for inverted signal
    Serial.print(" ");
    Serial.print(sg2_percent); // print percent for inverted signal

    Serial.print("          ");

    Serial.print(sg1_val); // print inverted signal, inverted to normal
    Serial.print(" and ");
    Serial.print(sg2_val); // print non-inverted signal

    Serial.print("          ");

    Serial.print(sensor1);
    Serial.print(" and ");
    Serial.println(sensor2);
  }
  // Check if APPS is deviating more than 10%
  if (error == false)
  {
    if ((abs(sg2_percent - sg1_percent) > 10))
    { // check if difference between signals is more than 10%, do this:
      if (firstDeviadeTime != 0)
        firstDeviadeTime = millis();
      if (millis() - firstDeviadeTime > 100)
      {               // if they deviate for more than 100 ms:
        error = true; // set ERROR to true
      }
    }
    else
    {
      firstDeviadeTime = 0;
    }

    if (sg2_percent < 0 || sg2_percent > 100 || sg1_percent < 0 || sg1_percent > 100) // check if the signals are too low/high
    {
      error = true;
    } // set ERROR to true

    else
    { // if nothing is wrong:
      if (sg1_percent > 1)
      {
        out = sg1_percent;        // sends signal from 0-100% to CANbus
        analogWrite(outPin, out); // FIX THIS IS NOT CANBUS
      }
    }
  }

  else
  { // if error is true
    // TODO Serial.println(" ERROR "); // print ERROR to monitor
    digitalWrite(light, HIGH); // error light
  }

  // --- Inverter ---
  // Turn off relays if TSMS is toggled off.
  // Reset relays if error has occured during precharge
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
  if (R2DS_toggled == 0 && VSM_state >= 4 && shutdown_circuit == 1)
  {
    digitalWrite(BUZZER_OUTPUT_PIN, HIGH);
    r2dSoundStartTime = millis();
    R2DS_toggled = 1;
  }
  // Turn off ready to drive sound
  if (millis() - r2dSoundStartTime >= 2000)
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

  // TODO add brake sensor above 25% shutdown everything
  if (error == true || shutdown_circuit == 0 || ready_to_drive == 0)
  {
    commandedInverterMessage.data[0] = 0x00; // Commanded torque, first
    commandedInverterMessage.data[1] = 0x00; // Commanded torque, last
    commandedInverterMessage.data[5] = 0x00; // 5.0 Inverter enable(0 off, 1 on)
    // Serial.print("Error somewhere");
  }
  // --- CAN-BUS ---
  if (millis() - tempSendTime > 100)
  {
    Serial.print(commandedInverterMessage.data[0], HEX);
    Serial.print(";");
    Serial.print(commandedInverterMessage.data[1], HEX);
    Serial.println("");
    Serial.print("VSM State: ");
    Serial.print(VSM_state);
    Serial.println("");
    can0.sendMessage(&commandedInverterMessage);
    tempSendTime = millis();
  }
}
