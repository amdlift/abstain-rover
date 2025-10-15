#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

#include <Wire.h>

/*
BNO055 Code
  This Code is to read sensor data from
  a BNO055

*/
// Board 4 Setup and Initiation Code

// Initialization of Const Variables
const int I2CSDA = 2;
const int I2CSCL = 3;
const int dataRX = 1;
const int dataTX = 0;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;


void setup() {
    Serial1.setRX(dataRX);
    Serial1.setTX(dataTX);
    Serial.begin(9600);
    Wire1.setSDA(I2CSDA);
    Wire1.setSCL(I2CSCL);
  // If error in startup

  if (!bmp.begin_I2C(0x77, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
    // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
   
}

void loop() {

 if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(100);
}
