#include "Arduino.h"
#include "lib/mcp2515.h"

// Definitions
#define SHUTDOWN_CIRCUIT_PIN A3
#define READY_TO_DRIVE_INPUT A2
#define READY_TO_DRIVE_OUTPUT A4
enum RELAY_RESET_MODES
{
  RELAY_RESET_OFF,
  RELAY_RESET_FIRST,
  RELAY_RESET_ENABLE,
  RELAY_RESET_DISABLE
};
int RELAY_RESET_STATE = RELAY_RESET_OFF;

// Define canbus frames
struct can_frame commandedInverterMessage; // To send for one peace of data. Can be duplicated for other ID's and data
struct can_frame clearFaultsCanFrame;
struct can_frame canReceive;
MCP2515 can0(10); // Chip select

// Sensors have 15-360 degrees, so values must be: 315 to 1023 | 1 to 708
// Accelerator Pedal Position Sensor

int sg1 = A0;                    // declare pin for inverted signal
int sg2 = A1;                    // declare pin for non-inverted signal
const int outPin = 12;           // declare pin for output
int out = 0;                     // declare reading for output
int sensor1;                     // declare reading for inverted signal
int sensor2;                     // declare reading for non-inverted signal
unsigned long previousTime = 0;  // declare time before each iteration
unsigned long firstDeviadeTime = 0;
bool error = false;              // declare ERROR value to default = false
unsigned long currentTime = 0;   // declare timer
const int light = 13;            // declare error light
const double offset2 = 2.3668;   // offset for non-inverted signal
const double offset1 = 1.1105;   // offset for inverted signal
const double offset2_2 = 2.7222; // offset for non-inverted signal
const double offset1_2 = 1.6667; // offset for inverted signal
const double gain = 2.5;         // GAIN

void clearFaults()
{
  // TODO update comments
  clearFaultsCanFrame.can_id = 0x0C1; // CANBUS ID
  clearFaultsCanFrame.can_dlc = 8;    // Length of the message
  //---CAN DATA START --- //
  clearFaultsCanFrame.data[0] = 0x14; // Commanded torque, first
  clearFaultsCanFrame.data[1] = 0x00; // Commanded torque, last
  clearFaultsCanFrame.data[2] = 0x01; // Commanded speed, first(not used)
  clearFaultsCanFrame.data[3] = 0x00; // Commanded speed, last(not used)
  clearFaultsCanFrame.data[4] = 0x01; // Commanded direction, 0 = reverse, 1 = forward
  clearFaultsCanFrame.data[5] = 0x00; // 5.0 Inverter enable(0 off, 1 on)
  clearFaultsCanFrame.data[6] = 0x00; // Not in use
  clearFaultsCanFrame.data[7] = 0x00; // Not in use
  //---CAN DATA END --- //
  can0.sendMessage(&clearFaultsCanFrame);
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
  can0.setNormalMode();
  can0.setFilterMask(MCP2515::MASK0, true, 0x0AA);
  // pinMode(outPin, OUTPUT);       // set output to right mode
  Serial.begin(9600);       // start monitor for values
  pinMode(light, OUTPUT);   // set output for error light
  digitalWrite(light, LOW); // set light to low
  pinMode(SHUTDOWN_CIRCUIT_PIN, INPUT);
  pinMode(READY_TO_DRIVE_INPUT, INPUT);
  pinMode(READY_TO_DRIVE_OUTPUT, OUTPUT);
}

unsigned long tempSendTime, reset_timer = 0, r2dSoundStartTime;
int shutdown_circuit_toggle, shutdown_circuit = 0, ready_to_drive_toggle, ready_to_drive, R2D_toggled = 0, VSM_state = 0, VSM_toggled = 0;

void loop()
{
  shutdown_circuit_toggle = (shutdown_circuit != digitalRead(SHUTDOWN_CIRCUIT_PIN) ? 1 : 0);
  shutdown_circuit = digitalRead(SHUTDOWN_CIRCUIT_PIN);

  ready_to_drive_toggle = (ready_to_drive != digitalRead(READY_TO_DRIVE_INPUT) ? 1 : 0);
  ready_to_drive = digitalRead(READY_TO_DRIVE_INPUT);

  // --- CANBUS read ---
  // TODO If in VSM state below 4, set R2D_toggled 0
  if (can0.readMessage(&canReceive) == MCP2515::ERROR_OK)
  {
    // frame contains received message
    if (canReceive.can_id == 0x0AA)
    {
      VSM_toggled = (VSM_state != canReceive.data[0] ? 1 : 0);
      VSM_state = canReceive.data[0];
      VSM_toggled = (VSM_state < 4 ? 0 : VSM_toggled);
    }
  }
  sensor1 = (analogRead(sg1) * (5 / 1023)) / gain + offset2_2; // read inverted signal
  sensor2 = (analogRead(sg2) * (5 / 1023)) / gain + offset1_2; // read non-inverted signal

  double sg2_percent = (sensor2 - offset2_2) * 163;        // convert to percent for non-inverted
  double sg1_percent = 100 - ((sensor1 - offset1_2) * 90); // convert to percent for inverted

  Serial.print("Percent: ");
  Serial.print(sg1_percent); // print percent for inverted signal
  Serial.print(" and ");
  Serial.print(sg2_percent); // print percent for non-inverted signal

  Serial.print("          ");

  Serial.print(1023 - sensor1); // print inverted signal, inverted to normal
  Serial.print(" and ");
  Serial.print(sensor2); // print non-inverted signal

  Serial.print("          ");

  Serial.print(sensor1);
  Serial.print(" and ");
  Serial.println(sensor2);
  if (error == false)
  {
    if ((abs(sg2_percent - sg1_percent) > 10))
    {                         // check if difference between signals is more than 10%, do this:
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
  {                            // if error is true
    Serial.println(" ERROR "); // print ERROR to monitor
    digitalWrite(light, HIGH); // error light
  }

  // --- Inverter ---
  // Turn off relays if TSMS is toggled off.

  if (shutdown_circuit_toggle && shutdown_circuit == 1)
  {
    commandedInverterMessage.data[4] = 0x00;
    can0.sendMessage(&commandedInverterMessage);
    reset_timer = millis();
    RELAY_RESET_STATE = RELAY_RESET_ENABLE;
  }

  else if (RELAY_RESET_STATE == RELAY_RESET_ENABLE && (millis() - reset_timer >= 200))
  {
    commandedInverterMessage.data[4] = 0x01;
    can0.sendMessage(&commandedInverterMessage);
    reset_timer = millis();
    RELAY_RESET_STATE = RELAY_RESET_DISABLE;
  }
  else if (RELAY_RESET_STATE == RELAY_RESET_DISABLE && (millis() - reset_timer >= 200))
  {
    commandedInverterMessage.data[4] = 0x00;
    can0.sendMessage(&commandedInverterMessage);
    reset_timer = millis();
    RELAY_RESET_STATE = RELAY_RESET_OFF;
  }

  if ((shutdown_circuit_toggle || ready_to_drive_toggle) && shutdown_circuit == 1 && ready_to_drive == 0)
  {
    clearFaults();
    commandedInverterMessage.data[4] = 0x00; // Disable inverter
    can0.sendMessage(&commandedInverterMessage);
  }

  // Enable or disable inverter based on switch
  if (ready_to_drive == 0)
    commandedInverterMessage.data[4] = 0x00;
  else if (ready_to_drive == 1)
    commandedInverterMessage.data[4] = 0x01;

  // Turn on ready to drive sound
  if (R2D_toggled == 0 && VSM_state >= 4)
  {
    digitalWrite(READY_TO_DRIVE_OUTPUT, HIGH);
    r2dSoundStartTime = millis();
    R2D_toggled = 1;
  }
  // Turn off ready to drive sound
  if (millis() - r2dSoundStartTime >= 2000)
  {
    digitalWrite(READY_TO_DRIVE_OUTPUT, LOW);
  }

  // --- CAN-BUS ---
  if (millis() - tempSendTime > 100)
    can0.sendMessage(&commandedInverterMessage);
  shutdown_circuit_toggle = 0;
  ready_to_drive_toggle = 0;
  VSM_toggled = 0;
}
