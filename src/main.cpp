#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <AMT22_lib.h>


#define SDC A5
#define CAN_CS 10
#define CAN_INT 2
#define ENC1_CS 7
#define ENC2_CS 8

#define BP1 A0
#define BP2 A1

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13

#define APPS_TRAVEL 105.0 // Resolution of 1024 bit ADC

// ID for default broadcasting of R2D state, and pedal pressures.
#define APPS_BROADCAST_ID 0x0E1

// ID for cascadia command
#define INV_CMD_ID 0x0C0

// ID for cascadia Relay and clear fault control.
#define INV_RLY_ID 0x0C1
#define INV_CLF_ID 0x0C1

#define SENSOR1_PIN A7
#define SENSOR2_PIN A3

// ID of the dashboard brain, for spoofing.
#define DB_ID 0x0E0

// "Emergency" channels, to very quickly open SDC
#define SDC_OPEN_ID1 0x0B0
#define SDC_OPEN_ID2 0x0B1

#define MAX_TORQUE 2220 // Nm * 10
#define BUZZER_DURATION 3 // seconds

MCP_CAN CAN0(CAN_CS);

uint16_t sensorPosition1;
uint16_t sensorPosition2;

uint16_t sensor1_throttle;
uint16_t sensor2_throttle;

uint16_t throttle;


// CONFIG VARIABLES:
float apps_deadzone = 0.04;


// SPI settings for the APPS sensors:
// SPI mode 0
// Frame size 8 bit
// Data rate 2Mhz
// The most significant bit comes first.
SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

// Encoder 1 is a non inverted signal.
uint16_t sensor1_min = 100;

// Encoder 2 is a inverted signal.
uint16_t sensor2_max = 500;

// Brake pressure variables:
float brakePressure1, brakePressure2;


// Bitflags:
bool deviation_error = false;
bool ready_to_drive = false;
bool ready_to_drive_switch = false;
bool shutdown_circuit = false;
bool encoder_fault = false;
bool brakelight = false;
bool brakeImplausibility = false;
bool inverter_enable_lockout = false;

// VSM State
uint16_t vsm_state = 0;

// Watchdog timer for R2D switch
unsigned long r2d_timestamp = 0;

// Lockout timestamp for reset.
unsigned long lockout_timestamp = 0;

// Deviation timestamp, to check if it lasts more than 100ms
unsigned long deviation_timestamp = 0;

// Brake Implausibility timestamp
unsigned long brakeImplausibility_timestamp = 0;

// The arduino should sample more often than it sends canbus messages / serial output
// Therefore we need a timestamp for the last sent message.
unsigned long rly_timestamp = 0;
unsigned long broadcast_timestamp = 0;

unsigned long command_timestamp = 0;

// Motor speed, for torque modulation
unsigned int motor_speed = 0; // RPM

unsigned int motor_feedback_torque = 0; // TODO: Implement


// CANBUS read variables
long unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];


int get_max_torque(){

  if(motor_speed < 3100) {
    return MAX_TORQUE;
  }

  if(motor_speed < 4200) {
    return MAX_TORQUE - (int) (600.0 * ((float)(motor_speed % 3100) / 1100.0));
  }

  return 170 - (int) (450.0 * ((float)(motor_speed % 4200) / 1300.0));

}


float get_brake_pressure(int analogPin) {
  float pressurePSI, pressureMBAR, pressureVDC;
  int pressure;
  float NullVDC = 0.5;
  float Sensitivity = 4000 / 2000; // 4000 mV / 2000 psi

  pressure = analogRead(analogPin);
  if(pressure < 5/1024 * 0.4){
    pressureMBAR = 100;
    encoder_fault = true;
  }
  pressureVDC = (float)pressure * 0.0048828125; // (5/1024 = 0.0048...)
  pressureVDC = pressureVDC - NullVDC;
  pressurePSI = pressureVDC / Sensitivity * 1000; // (VDC*1000 = mV)  / (mV / PSI)
  pressureMBAR = pressurePSI * 0.068948;

  if(pressureMBAR < 0){
    pressureMBAR = 0;
  }

  return pressureMBAR;
}


// Uses a library provided by AMT to re0ad the APPS sensors.
// Library sets encoder position to 0xFFFF if there is an error in the checksum value.
// We let up to 3 faults happen each reading, and if there is more than that, we enable a error flag.
void read_apps_sensors() {

  //Serial.println(analogRead(SENSOR1_PIN));
  //Serial.println(analogRead(SENSOR2_PIN));

  sensorPosition1 = analogRead(SENSOR1_PIN);
  sensorPosition2 = analogRead(SENSOR2_PIN);
  
  if(sensorPosition1 < 10 || sensorPosition2 < 10){
    deviation_error = true;
  }

  sensor1_throttle = constrain((int)(((double)(sensorPosition1-sensor1_min))/APPS_TRAVEL*100.0),0,100);
  sensor2_throttle = constrain((int)(((double)(sensor2_max-sensorPosition2))/APPS_TRAVEL*100.0),0,100);

  // Sanity Check 
  if(sensorPosition1 < sensor1_min){
    sensor1_throttle = 0;
  }
  if(sensorPosition2 > sensor2_max){
    sensor2_throttle = 0;
  }

  

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
  digitalWrite(CAN_CS, HIGH);
  switch (rxId)
  {
  case 0x0E0:
    
    r2d_timestamp = millis(); // This is a check for the watchdog
    ready_to_drive_switch = 0x1 & rxBuf[0];
    break;
  
  case 0x0AA:
    vsm_state = rxBuf[0];
    inverter_enable_lockout = 0x80 & rxBuf[6];
    break;
  case 0x0A5:
    motor_speed = rxBuf[3] << 8;
    motor_speed |= rxBuf[2];
    break;
  
  default:
    return;
  }
}


// This is important!
void inverter_command(int throttle) {
  byte commandData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // First 2 bytes are used for torque.
  if (ready_to_drive){
    int torque = (int) round(((double)throttle)/100.0 * get_max_torque());
    commandData[0] = torque & 0x00FF;
    commandData[1] = (torque >> 8) & 0xFF;
  }
  

  // Byte 2 and 3 are speed command, if we are using speed mode. We are not, so these stay at zero.

  // Byte 4 is commanded direction. We can only drive forward according to rules, hence the direction is Forward (0x1).
  commandData[4] = 0x1;

  // Byte 5, bit 1 is inverter enable, which it should be if we are ready to drive.
  commandData[5] = ready_to_drive;

  // Kind of a bugfix.
  // The current inverter sometimes goes into Inverter Enable lockout mode after power cycling.
  // And it wont reset unless we cycle inverter enable at least once.
  if(inverter_enable_lockout){
    if(lockout_timestamp == 0){
      lockout_timestamp = millis();
    }

    if(millis() - lockout_timestamp < 100){
      commandData[5] = true;
    }else if(millis() - lockout_timestamp < 300){
      commandData[5] = false;
    }else{
      lockout_timestamp = 0;
      inverter_enable_lockout = false;
    }

  }else{
    lockout_timestamp = 0;
  }

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

  // Assigning the chip-select CS-BARRED the CANBUS pcb.
  pinMode(CAN_CS, OUTPUT);
  pinMode(ENC1_CS, OUTPUT);
  pinMode(ENC2_CS, OUTPUT);
  pinMode(BP1, INPUT);
  pinMode(BP2, INPUT);
  digitalWrite(CAN_CS, HIGH);

  // Setting the interrupt pin for the mcp2515
  pinMode(CAN_INT, INPUT);

  // Shutdown circuit
  pinMode(SDC, INPUT);

  Serial.begin(9600);



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
  CAN0.init_Filt(0, 0x00AA);

  // Set the MCP2515 into normal mode.
  CAN0.setMode(MCP_NORMAL);

  
}

void loop() {

  // Shutdown circuit status, as this is quite important, this goes first.
  shutdown_circuit = digitalRead(SDC);

  brakePressure1 = get_brake_pressure(BP1);
  brakePressure2 = get_brake_pressure(BP2); // TODO: CHANGE TO BP2

  if(brakePressure1 > 4 || brakePressure2 > 4) {
    brakelight = true;
  }else{
    brakelight = false;
  }

  // Reads the current position data from the encoders.
  read_apps_sensors();

  // Handle incoming canbus messages
  can_rx();


  // To make sure we do not have a misread on one sensor, and that the car drives, when the pedal is in the zero position,
  // We take the minimum value of the pedal percentages.
  uint8_t throttle_signal = min(sensor1_throttle, sensor2_throttle);
  // As we are using unsigned integers for this, check if value is above 150%, and set to zero if this is the case
  // As it more than likely means that both of the integer values of the sensor wrapped around.
  if(throttle_signal > 150){
    throttle_signal = 0;
  }

  // /sensor1_throttlehe deviation percentage and checks how long an eventual fault has lasted
  if(abs(sensor1_throttle - sensor2_throttle) > 10){
    if(deviation_timestamp == 0){
      deviation_timestamp = millis();
    }
    if(millis() - deviation_timestamp > 100){
      deviation_error = true;
    }
    
  }else{
    deviation_timestamp = 0;
  }


  if((brakePressure1 >= 30 || brakePressure2 >= 30) && throttle_signal >= 25) {
    if(brakeImplausibility_timestamp == 0){
      brakeImplausibility_timestamp = millis();
    }else if(brakeImplausibility_timestamp - millis() >= 500){
      brakeImplausibility = true;
      throttle_signal = 0;
    }
    
  }else{
    if(brakeImplausibility == true && brakeImplausibility_timestamp != 0 && throttle_signal == 0){
      brakeImplausibility = false;
      brakeImplausibility_timestamp = 0;
    }
  }

  // If we havent heard anything from the dashboard in over a second, 
  // we should have a failstate which assumes the switch is off.
  if(millis() - r2d_timestamp > 2000){
    ready_to_drive_switch = false;
  }

  // Check if ready to drive should be enabled:
  if (!ready_to_drive){
    if(throttle_signal < 5 && shutdown_circuit && ready_to_drive_switch && (brakePressure1 > 10 || brakePressure2 > 10)){
      if(vsm_state < 4 || vsm_state > 6){
        inverter_clear_faults();
      }else{
        ready_to_drive = true;
      }
    }
  }

  if (!shutdown_circuit || !ready_to_drive_switch || encoder_fault || vsm_state < 4 || vsm_state > 6){
    ready_to_drive = false;
  }

  // Before sending messages to the inverter, we should make sure that the car is still in a good state:
  // TODO: Add more flags!
  if (deviation_error) {
     ready_to_drive = false;
  }


  if(millis() - command_timestamp > 10) {
    inverter_command(throttle_signal);
    command_timestamp = millis();
  }
  

  if(millis() - broadcast_timestamp > 100) {

    // Prepare canbus frame:
    // Byte 1 is the pedal position, in a percentage
    // Byte 2 is the commanded torque as a integer
    // Byte 3 is VSM state, from the APPS perspective
    // Byte 4 holds some states, with each bit meaning this: 0bABCDEFGH
    // B - Implausibility 
    // C - Encoder fault
    // D - Brakelight
    // E - Shutdown circuit
    // F - Ready to drive switch
    // G - Ready to drive state
    // H - Deviation Error

    // Byte 5 and 6 is the encoder positions of the APPS sensors.
    // Byte 7 and 8 is the brake pressures in MBAR.
    byte canFrame[8] = {0x0, 0x0, 0x0, 0x00, 0x0, 0x0, 0x0, 0x0};
    canFrame[0] = throttle_signal;

    if(ready_to_drive){
      canFrame[1] = (int) round((((double) throttle_signal / 100.0) * MAX_TORQUE) / 10.0);
      
    }

    canFrame[2] = vsm_state;

    canFrame[4] = sensorPosition1;
    canFrame[5] = sensorPosition2;

    canFrame[6] = (int) brakePressure1;
    canFrame[7] = (int) brakePressure2;

    canFrame[3] = brakeImplausibility << 6;
    canFrame[3] |= encoder_fault << 5;
    canFrame[3] |= brakelight << 4;
    canFrame[3] |= shutdown_circuit << 3;
    canFrame[3] |= ready_to_drive_switch << 2;
    canFrame[3] |= ready_to_drive << 1;
    canFrame[3] |= deviation_error;


    // Send canbus frame:
    can_tx(APPS_BROADCAST_ID, 8 , canFrame);

    broadcast_timestamp = millis();
    // Serial.println("Throttle % " + String(sensor1_throttle) + ", sensorPosition: " + String(sensorPosition1) + ", Raw throttle: " + String(sensorPosition1 - sensor1_min));
    // Serial.println("Brake pressure 1: " + String(brakePressure1) + ", Brake pressure 2: " + String(brakePressure2));
    // Serial.println("Sensor 1%: " + String(sg_percentage1) + ", Sensor 2%: " + String(sg_percentage2));
    // Serial.println("Throttle %: " + String(canbus_signal));
  }
}
