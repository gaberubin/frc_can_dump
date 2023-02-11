#include <frc_CAN.h>
#include <frc_can_core.h>
#include <frc_mcp2515.h>

#include <Wire.h>
#include <stdio.h>
#include <SerialUSB.h>
#include <Adafruit_BNO055.h>

#include <Adafruit_ST7789.h>

const int8_t TFT_CS_PIN = 5;
const int8_t TFT_RST_PIN = 17;
const int8_t TFT_DC_PIN = 16;
const int8_t TFT_MOSI_PIN = 23;
const int8_t TFT_SCLK_PIN = 18;

Adafruit_BNO055 bno(55, BNO055_ADDRESS_A, &Wire);
double xPos = 0, yPos = 0, headingVel = 0;
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;  //how often to read data from the board
const uint16_t PRINT_DELAY_MS = 500;             // how often to print the data
uint16_t printCount = 0;       

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
const double DEG_2_RAD = 0.01745329251;  //trig functions require radians, BNO055 outputs degrees

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



Adafruit_ST7789 tft( TFT_CS_PIN, TFT_DC_PIN, TFT_MOSI_PIN, TFT_SCLK_PIN, TFT_RST_PIN);

// Define the CS pin and the interrupt pin
const uint8_t CAN_SPI_BUS = HSPI;
const int8_t CAN_SCLK_PIN = 14; // selected by HSPI
const int8_t CAN_MOSI_PIN = 15; // selected by HSPI
const int8_t CAN_MISO_PIN = 12; // selected by HSPI
const int8_t CAN_CS_PIN = 33;
const int8_t CAN_INT_PIN = 32;

#define CAN_INTERRUPT 2

// Create an MCP2515 device. Only need to create 1 of these
SPIClass SPI2( HSPI );
frc::MCP2515 canTransceiver{ CAN_CS_PIN, SPI2 };

// Create an FRC CAN Device. You can create up to 16 of these in 1 progam
// Any more will overflow a global array
frc::CAN frcCANDevice{ 1 };

// Callback function. This will be called any time a new message is received
// Matching one of the enabled devices.
void CANCallback( frc::CAN *can, int apiId, bool rtr, const frc::CANData &data ) {
  Serial.print( rtr ? "Received request for API " : "Received message for API " );
  Serial.print( apiId, HEX );
  if (!rtr) {
    data.
  }
    
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback( uint32_t id, const frc::CANData &data ) {

}


void setup() {
    // Initialize the MCP2515. If any error values are set, initialization failed
    auto err = canTransceiver.reset();
    // CAN rate must be 1000KBPS to work with the FRC Ecosystem
    // Clock rate must match clock rate of CAN Board.
    err = canTransceiver.setBitrate( frc::CAN_1000KBPS, frc::CAN_CLOCK::MCP_8MHZ );

    // Set up to normal CAN mode
    err = canTransceiver.setNormalMode();

    // Prepare our interrupt pin
    pinMode( CAN_INT_PIN, INPUT );
    
    // Set up FRC CAN to be able to use the CAN Impl and callbacks
    // Last parameter can be set to nullptr if unknown messages should be skipped
    frc::CAN::SetCANImpl( &canTransceiver, CAN_INT_PIN, CANCallback, UnknownMessageCallback );

    // All CAN Devices must be added to the read list. Otherwise they will not be handled correctly.
    frcCANDevice.AddToReadList();
  
  /* Initialise the sensor */
  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
    delay(500);
  }
  Serial.print("Setting BNO055 to use external crystal\n");

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  Serial.print("Getting BNO055 details\n");

  /* Display some basic information on this sensor */
  displaySensorDetails();


}

void loop() {
   // Update must be called every loop in order to receive messages
    frc::CAN::Update();

    uint8_t data[8];

  auto tnow = micros();
  if ((tnow - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {

    //delayMicroseconds(BNO055_SAMPLERATE_DELAY_MS * 1000 - (tnow - tStart));
  }

}

void printImu() {
  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount == 5) {  //* BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(orientationData.orientation.x);
    Serial.print("Position: ");
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
    Serial.print("Speed: ");
    Serial.println(headingVel);
    Serial.println("-------");

    printCount = 0;
  } else {
    printCount = printCount + 1;
  }
}

void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}
