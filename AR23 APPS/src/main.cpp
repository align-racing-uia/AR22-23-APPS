#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>


#define SDC A6
#define CAN_CS 10
#define CAN_INT 2
#define SENSOR1_CS 7
#define SENSOR2_CS 8
#define APPS_TRAVEL 210.0 // Degrees x 10^(-1)

// ID for default broadcasting of R2D state, and pedal pressures.
#define APPS_BROADCAST_ID 0x0E1

// ID for cascadia command
#define INV_CMD_ID 0x0C0

// ID for cascadia Relay and clear fault control.
#define INV_RLY_ID 0x0C1
#define INV_CLF_ID 0x0C1

// "Emergency" channels, to very quickly open SDC
#define SDC_OPEN_ID1 0x0B0
#define SDC_OPEN_ID2 0x0B1

#define MAX_TORQUE 2150 // Nm * 10
#define BUZZER_DURATION 3 // seconds

MCP_CAN CAN0(CAN_CS);

// SPI settings for the APPS sensors:
// SPI mode 0
// Frame size 8 bit
// Data rate 2Mhz
// The most significant bit comes first.
SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

// Sensor 1 is a non inverted signal.
uint16_t sensor1_min = 0xFFFF;

// Sensor 2 is a inverted signal.
uint16_t sensor2_max = 0x0000;


// Bitflags:
bool deviation_error = false;
bool ready_to_drive = false;
bool ready_to_drive_switch = false;
bool precharge_button = false;
bool shutdown_circuit = false;
bool brakelight = false;
bool buzzer = false;

// Buzzer timestamp

// Deviation timestamp, to check if it lasts more than 100ms
int deviation_timestamp = 0;

// The arduino should sample more often than it sends canbus messages / serial output
// Therefore we need a timestamp for the last sent message.
int rly_timestamp = 0;
int broadcast_timestamp = 0;
int command_timestamp = 0;

// CANBUS read variables
long unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];

// Function to reset the apps sensor on startup
// It expects a signal on SPI with 0x00 and 0x70 with a gap of 3 microseconds between.
void reset_apps_sensor(int cs) {
  digitalWrite(cs, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(0x00);
  delayMicroseconds(3);
  SPI.transfer(0x70);
  SPI.endTransaction();
  digitalWrite(cs, HIGH);
}

// Function to reset the apps sensor on startup
// It expects a signal on SPI with 0x00 and 0x00 with a gap of 3 microseconds between.
// Decoding is showed below.
uint32_t read_apps_sensor(int cs) {

  uint8_t val1 = 0;
  uint8_t val2 = 0;
  uint8_t val1_mask = 0x3F; 

  digitalWrite(cs, LOW);
  SPI.beginTransaction(settings);
  val1 = SPI.transfer(0x00);
  delayMicroseconds(3);
  val2 = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(cs, HIGH);

  val1 &= val1_mask;

  uint16_t val = 0x00;
  val |= val2;
  val |= (val1 << 8);
  val >>= 2;

  return val;
}

void can_tx(int id, int length, byte data[]){
  digitalWrite(CAN_CS, LOW);
  byte sendFrame = CAN0.sendMsgBuf(id, 0, length, data);
  digitalWrite(CAN_CS, HIGH);
}

void can_rx() {
  if(!digitalRead(CAN_INT)){
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
  }else{
    return;
  }

  switch (rxId)
  {
  case 0x0E0:
    
    ready_to_drive_switch = 0x1 & rxBuf[0];
    precharge_button = 0x2 & rxBuf[0];
  
  default:
    return;
  }
}

// This is important!
void inverter_command(int throttle) {
  byte commandData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // First 2 bytes are used for torque.
  if (ready_to_drive){
    commandData[0] = (throttle * MAX_TORQUE) & 0x00FF;
    commandData[1] = ((throttle * MAX_TORQUE) >> 8) & 0xFF00;
  }
  

  // Byte 2 and 3 are speed command, if we are using speed mode. We are not, so these stay at zero.

  // Byte 4 is commanded direction. We can only drive forward according to rules, hence the direction is Forward (0x1).
  commandData[4] = 0x1;

  // Byte 5, bit 1 is inverter enable, which it should be if we are ready to drive.
  commandData[5] |= ready_to_drive;
  
  // Byte 6 and 7 is Commanded Torque limit, this is set in eeprom, so its no need to send new values.


  can_tx(INV_CMD_ID, 8, commandData);
}

void inverter_clear_faults() {
  // For this one, just read the documentation. Cascadia CAN Protocol, Page 39.
  byte clearFaultData[8] = {0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  can_tx(INV_CLF_ID, 8, clearFaultData);
}

void inverter_relay_states() {
  // Initalizing default data according to cascadia documentation. Cascadia CAN Protocol, Page 40.
  byte rlyData[8] = {0x01, 0x00, 0x01, 0x00, 0x00, 0x55, 0x00, 0x00};

  rlyData[4] |= brakelight << 4;
  rlyData[4] |= buzzer << 5;

  can_tx(INV_RLY_ID, 8, rlyData);

}



void setup() {

  // Debugging pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Assigning the chip-select CS-BARRED for APPS sensors and the CANBUS pcb.
  pinMode(SENSOR1_CS, OUTPUT);
  pinMode(SENSOR2_CS, OUTPUT);
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(SENSOR1_CS, HIGH);
  digitalWrite(SENSOR2_CS, HIGH);
  digitalWrite(CAN_CS, HIGH);

  // Setting the interrupt pin for the mcp2515
  pinMode(CAN_INT, INPUT);

  // Shutdown circuit
  pinMode(SDC, INPUT);

  Serial.begin(9600);
  SPI.begin();

  // Reseting both sensors
  reset_apps_sensor(SENSOR1_CS);
  reset_apps_sensor(SENSOR2_CS);

  // Initializing CANBUS
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
    Serial.println("MCP2515 Initialized at 500kbps (16MHz)");
  }else{
    Serial.println("MCP2515 Error...");
  }

  // Standard id range is from 0x0000 - 0x03FF, First mask enables all bits in the filters, meaning only
  // set ids will pass through.

  // Since most of the logging and monitoring is done on the dashboard module, we only care about dashboard buttons.
  // e.g. R2D. This puts less strain on the arduino.
  CAN0.init_Mask(0, 0x03FF);
  CAN0.init_Filt(0, 0x00E0);

  // Set the MCP2515 into normal mode.
  CAN0.setMode(MCP_NORMAL);

  
}

void loop() {

  // Shutdown circuit status, as this is quite important, this goes first.
  shutdown_circuit = digitalRead(SDC);

  // Reading sensor input  
  uint16_t sensor1 = read_apps_sensor(SENSOR1_CS);
  uint16_t sensor2 = read_apps_sensor(SENSOR2_CS);

  
  // Handle incoming canbus messages
  can_rx();

  // Calibrating the sensors as we go.
  if(sensor1 < sensor1_min) {
    sensor1_min = sensor1;
  }

  // Actual pedal angle, sensor 1:
  sensor1 -= sensor1_min;

  if(sensor2 > sensor2_max) {
    sensor2_max = sensor2;
  }

  // Actual pedal angle, sensor 2:
  sensor2 = abs(sensor2_max - sensor2);

  // Calculating percentage of full pedal travel
  int sg_percentage1 = sensor1 / APPS_TRAVEL * 100;
  int sg_percentage2 = sensor2 / APPS_TRAVEL * 100;

  // To make sure we do not have a misread on one sensor, and that the car drives, when the pedal is in the zero position,
  // We take the minimum value of the pedal percentages.
  uint8_t throttle_signal = min(sg_percentage1, sg_percentage2);
  // As we are using unsigned integers for this, check if value is above 150%, and set to zero if this is the case
  // As it more than likely means that both of the integer values of the sensor wrapped around.
  if(throttle_signal > 150){
    throttle_signal = 0;
  }

  // Calculates the deviation percentage and checks how long an eventual fault has lasted
  if(abs(sg_percentage1 - sg_percentage2) > 10){
    if(deviation_timestamp == 0){
      deviation_timestamp = millis();
    }
    if(millis() - deviation_timestamp > 50){
      deviation_error = true;
    }
    
  }else{
    deviation_timestamp = 0;
  }

  // Check if ready to drive should be enabled:
  if (!ready_to_drive){
    if(throttle_signal < 5 || shutdown_circuit || ready_to_drive_switch){

    }
  }

  // Before sending messages to the inverter, we should make sure that the car is still in a good state:
  // TODO: Add more flags!
  if (deviation_error) {
    ready_to_drive = false;
  }

  if(millis() - rly_timestamp > 50) {
    inverter_relay_states();
    rly_timestamp = millis();
  } 

  if(millis() - command_timestamp > 10) {
    inverter_command(throttle_signal);
    command_timestamp = millis();
  }
  

  if(millis() - broadcast_timestamp > 100) {

    // Prepare canbus frame:
    // Byte 1 is the pedal position, in a percentage
    // Byte 4 holds some states, with each bit meaning this: 0bABCDEFGH
    // F - Ready to drive state
    // H - Deviation Error
    byte canFrame[4] = {0x0, 0x0, 0x0, 0x0};
    canFrame[0] = throttle_signal;
    canFrame[3] = ready_to_drive << 1;
    canFrame[3] = deviation_error;


    // Send canbus frame:
    can_tx(APPS_BROADCAST_ID, 4, canFrame);

    broadcast_timestamp = millis();
    //Serial.println("Sensor 1: " + String(sensor1) + ", Sensor 2: " + String(sensor2));
    // Serial.println("Throttle %: " + String(canbus_signal));
  }

  // The whole loop should have a minimum delay of 5ms.
  delay(5);

}
