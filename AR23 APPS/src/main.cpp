#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>


#define SDC A5
#define CAN_CS 10
#define CAN_INT 2
#define SENSOR1_CS 7
#define SENSOR2_CS 8
#define SENSOR1_MAX 3225
#define SENSOR1_MIN 3015
#define SENSOR2_MAX 1263
#define SENSOR2_MIN 1053

#define APPS_TRAVEL 210.0 // Degrees x 10^(-1)

// ID for default broadcasting of R2D state, and pedal pressures.
#define APPS_BROADCAST_ID 0x0E1

// ID for cascadia command
#define INV_CMD_ID 0x0C0

// ID for cascadia Relay and clear fault control.
#define INV_RLY_ID 0x0C1
#define INV_CLF_ID 0x0C1

// ID of the dashboard brain, for spoofing.
#define DB_ID 0x0E0

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
bool precharge_button_released = true;

// Watchdog timer for R2D switch
unsigned long r2d_timestamp = 0;

// Deviation timestamp, to check if it lasts more than 100ms
unsigned long deviation_timestamp = 0;

// The arduino should sample more often than it sends canbus messages / serial output
// Therefore we need a timestamp for the last sent message.
unsigned long rly_timestamp = 0;
unsigned long broadcast_timestamp = 0;

unsigned long command_timestamp = 0;

// CANBUS read variables
long unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];

// Tracking the current state of the inverter.
int cascadia_state = 0;

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
  // Needs a three microsecond delay after pulling CS low.
  delayMicroseconds(3);
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
    
    r2d_timestamp = millis(); // This is a check for the watchdog.

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
    int torque = (int) round(((double)throttle)/100.0 * MAX_TORQUE);
    commandData[0] = torque & 0x00FF;
    commandData[1] = (torque >> 8) & 0xFF;
  }
  

  // Byte 2 and 3 are speed command, if we are using speed mode. We are not, so these stay at zero.

  // Byte 4 is commanded direction. We can only drive forward according to rules, hence the direction is Forward (0x1).
  commandData[4] = 0x1;

  // Byte 5, bit 1 is inverter enable, which it should be if we are ready to drive.
  commandData[5] |= ready_to_drive;

  commandData[6] = MAX_TORQUE & 0x00FF;
  commandData[7] = (MAX_TORQUE >> 8) & 0xFF;
  
  // Byte 6 and 7 is Commanded Torque limit, this is set in eeprom, so its no need to send new values.

  // Serial.println("Sent inverter command!");
  can_tx(INV_CMD_ID, 8, commandData);
}

void inverter_clear_faults() {
  // For this one, just read the documentation. Cascadia CAN Protocol, Page 39.
  // Byte order is: #0 #1 #2 #3 #4 #5 #6 #7
  // Byte #0 and #1 is the parameter adress, in the following order #1#0. E.g. {0x14, 0x00} becomes 0x0014.
  // Byte #2 is R/W commands. 0x0 for read, 0x01 for write.
  // Byte #3, 6 and 7 is reserved and should be zero.
  // Byte #4 and #5 is the data, which is explained in the documentation.
  byte clearFaultData[8] = {0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  can_tx(INV_CLF_ID, 8, clearFaultData);
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
  int raw_sensor1 = read_apps_sensor(SENSOR1_CS);
  int raw_sensor2 = read_apps_sensor(SENSOR2_CS);

  
  // Handle incoming canbus messages
  can_rx();

  // Actual pedal angle, sensor 1:
  // Setting the zero point of the sensor
  if(raw_sensor1 < sensor1_min){
    sensor1_min = raw_sensor1;
  }
  int sensor1 = raw_sensor1 - sensor1_min;
  if(sensor1 < 0){
    sensor1 = 0;
  }

  // Actual pedal angle, sensor 2:
  // Setting the zero point of the sensor
  if(raw_sensor2 > sensor2_max){
    sensor2_max = raw_sensor2;
  }
  int sensor2 = sensor2_max - raw_sensor2;
  if(sensor2 < 0){
    sensor2 = 0;
  }

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
    if(millis() - deviation_timestamp > 1000){
      deviation_error = true;
    }
    
  }else{
    deviation_timestamp = 0;
  }

  // If we havent heard anything from the dashboard in over a second, 
  // we should have a failstate which assumes the switch is off.
  if(millis() - r2d_timestamp > 2000){
    ready_to_drive_switch = false;
  }

  // Check if ready to drive should be enabled:
  if (!ready_to_drive){
    if(throttle_signal < 5 && shutdown_circuit && ready_to_drive_switch){
      inverter_clear_faults();
      ready_to_drive = true;
    }
  }

  if (precharge_button && precharge_button_released){
    precharge_button_released = false;
  }
  
  if (!precharge_button){
    precharge_button_released = true;
  }

  if (!shutdown_circuit || !ready_to_drive_switch){
    ready_to_drive = false;
  }

  // Before sending messages to the inverter, we should make sure that the car is still in a good state:
  // TODO: Add more flags!
  // if (deviation_error) {
  //   ready_to_drive = false;
  // }


  if(millis() - command_timestamp > 10) {
    inverter_command(throttle_signal);
    command_timestamp = millis();
  }
  

  if(millis() - broadcast_timestamp > 100) {

    // Prepare canbus frame:
    // Byte 1 is the pedal position, in a percentage
    // Byte 4 holds some states, with each bit meaning this: 0bABCDEFGH
    // E - Shutdown circuit
    // F - Ready to drive switch
    // G - Ready to drive state
    // H - Deviation Error
    byte canFrame[6] = {0x0, 0x0, 0x0, 0x00, 0x0, 0x0};
    canFrame[0] = throttle_signal;

    if(ready_to_drive){
      canFrame[1] = (int) round((((double) throttle_signal / 100.0) * MAX_TORQUE) / 10.0);
      
    }

    canFrame[4] = sensor1;
    canFrame[5] = sensor2;

    canFrame[3] = shutdown_circuit << 3;
    canFrame[3] |= ready_to_drive_switch << 2;
    canFrame[3] |= ready_to_drive << 1;
    canFrame[3] |= deviation_error;


    // Send canbus frame:
    can_tx(APPS_BROADCAST_ID, 6, canFrame);

    broadcast_timestamp = millis();
    Serial.println("Sensor 1: " + String(raw_sensor1) + ", Sensor 2: " + String(raw_sensor2));
    Serial.println("Sensor 1%: " + String(sg_percentage1) + ", Sensor 2%: " + String(sg_percentage2));
    // Serial.println("Throttle %: " + String(canbus_signal));
  }


}
