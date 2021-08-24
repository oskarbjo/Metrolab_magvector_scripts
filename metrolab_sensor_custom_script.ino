// Custom code for Metrolab magvector sensor

#include <SPI.h>

///////////////// SELECT ANALOG/DIGITAL: //////////////////
bool analogDigital = 1; //DIGITAL MODE = 1, ANALOG MODE = 0
///////////////////////////////////////////////////////////


#define MV2_SPI_CLK_FREQ 1000000

#define chipSelectPin 10
#define D_SPI_MOSI 11
#define D_SPI_MISO 12
#define D_SPI_CLK 13

// COMMON
#define NDIGITAL_ANALOG_PIN     8
#define VDD_PIN          A5 

// DIGITAL MODE
#define D_DR_PIN        2
#define D_INIT_PIN        7
#define D_SPI_MOSI        11
#define D_SPI_MISO        12
#define D_SPI_CLK               13

//ANALOG MODE
#define ANALOG_SHIFT      6
#define ANALOG_OFFSET      0x200
#define A_BX_PIN        A0
#define A_BY_PIN        A1
#define A_BZ_PIN        A2
#define A_TEMPERATURE_PIN   A3
#define A_REF_PIN       A4
#define A_EMR_PIN               2
#define A_INV_PIN       4
#define A_LP_PIN        7
#define A_MA0_PIN        10
#define A_MA1_PIN       11
#define A_RA0_PIN       12
#define A_RA1_PIN       13



uint16_t initSettingRegister00 = 0b1110110000110000;
uint16_t initSettingRegister01 = 0b1110110100000010;
uint16_t initSettingRegister10 = 0b1110111000001000;

//uint16_t settingX_readwrite = 0b1111110000110000;
//uint16_t settingY_readwrite = 0b1111110000110001;
//uint16_t settingZ_readwrite = 0b1111110000110010;
//uint16_t settingT_readwrite = 0b1111110000110011;

uint16_t settingX_write = 0b1110110000110000;
uint16_t settingY_write = 0b1110110000110001;
uint16_t settingZ_write = 0b1110110000110010;
uint16_t settingT_write = 0b1110110000110011;

uint16_t settingX_read = 0b1101110000110000;
uint16_t settingY_read = 0b1101110000110001;
uint16_t settingZ_read = 0b1101110000110010;
uint16_t settingT_read = 0b1101110000110011;

unsigned int X;
unsigned int Y;
unsigned int Z;
unsigned int T;


//////////////////////////////////// FUNCTIONS ///////////////////////////////////

void writeRegisters(int toWrite){
  SPI.beginTransaction(SPISettings(MV2_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin,LOW);
  SPI.transfer16(toWrite);
  digitalWrite(chipSelectPin,HIGH);
  SPI.endTransaction();
  }

int readWriteRegisters(int toWrite){
  int dataOut;
  SPI.beginTransaction(SPISettings(MV2_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin,LOW);
  dataOut=SPI.transfer16(toWrite);
  digitalWrite(chipSelectPin,HIGH);
  SPI.endTransaction();
  return dataOut;
  }

int readRegisters(int toWrite){
  int dataOut;
  SPI.beginTransaction(SPISettings(MV2_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin,LOW);
  dataOut=SPI.transfer16(toWrite);
  digitalWrite(chipSelectPin,HIGH);
  SPI.endTransaction();
  return dataOut;
  }


void MiscConfigureDigitalModePin()
{
    // Set common PIN
    MiscConfigureCommonPin();

    // Set PINs used in digital mode
    pinMode(NDIGITAL_ANALOG_PIN, OUTPUT);
    pinMode(D_DR_PIN, INPUT);
    pinMode(D_INIT_PIN, OUTPUT);
    pinMode(chipSelectPin, OUTPUT);

    // Ananlog/digital selector must be LOW
    digitalWrite(NDIGITAL_ANALOG_PIN, LOW);
    
    // MagVector Chip Select (CS) must be deactived
    digitalWrite(chipSelectPin, HIGH);
    
    // Initialize INIT to low
    digitalWrite(D_INIT_PIN, LOW);
}

void MiscConfigureAnalogModePin()
{
  // Set common PIN
  MiscConfigureCommonPin();

  // Set PINs used in analog mode
    pinMode(NDIGITAL_ANALOG_PIN, OUTPUT);
  pinMode(A_MA0_PIN, OUTPUT);
  pinMode(A_MA1_PIN, OUTPUT);
  pinMode(A_RA0_PIN, OUTPUT);
  pinMode(A_RA1_PIN, OUTPUT);
  pinMode(A_LP_PIN, OUTPUT);
  pinMode(A_EMR_PIN, OUTPUT);

    // Ananlog/digital selector must be HIGH
    digitalWrite(NDIGITAL_ANALOG_PIN, HIGH);

    // Initialize all other controls to LOW (logic 0)
    digitalWrite(A_MA0_PIN, LOW);
    digitalWrite(A_MA1_PIN, LOW);
    digitalWrite(A_RA0_PIN, LOW);
    digitalWrite(A_RA1_PIN, LOW);
    digitalWrite(A_LP_PIN, LOW);
    digitalWrite(A_EMR_PIN, LOW);
}

void MiscConfigureCommonPin()
{
  pinMode(VDD_PIN, INPUT);
    pinMode(A_BX_PIN, INPUT);
    pinMode(A_BY_PIN, INPUT);
    pinMode(A_BZ_PIN, INPUT);
    pinMode(A_TEMPERATURE_PIN, INPUT);
    pinMode(A_REF_PIN, INPUT);
    pinMode(A_INV_PIN, OUTPUT);

    // Initialize INV output to LOW
    digitalWrite(A_INV_PIN, LOW);

}

uint16_t AnalogDigitizeBx()
{
  return ((analogRead(A_BX_PIN) - analogRead(A_REF_PIN)) + ANALOG_OFFSET) << ANALOG_SHIFT;
}

uint16_t AnalogDigitizeBy()
{
  return ((analogRead(A_BY_PIN) - analogRead(A_REF_PIN)) + ANALOG_OFFSET) << ANALOG_SHIFT;
}

uint16_t AnalogDigitizeBz()
{
  return ((analogRead(A_BZ_PIN) - analogRead(A_REF_PIN)) + ANALOG_OFFSET) << ANALOG_SHIFT;
}

void printValues(){
  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(Z);
  Serial.print(", ");
  Serial.print(T);
  Serial.println("");
  }





//////////////////////////////// MAIN FUNCTIONS //////////////////////////////

void setup() {
  Serial.begin(115200);
  if(analogDigital == 0){
  MiscConfigureAnalogModePin();
  }
  else if(analogDigital == 1){
  MiscConfigureDigitalModePin();
  SPI.begin();
  delay(100);
  writeRegisters(initSettingRegister00);
  writeRegisters(initSettingRegister01);
  writeRegisters(initSettingRegister10);
  }
}

void loop() {
if(analogDigital == 0){
  X=AnalogDigitizeBx();
  Y=AnalogDigitizeBy();
  Z=AnalogDigitizeBz();
}
else if(analogDigital == 1){
writeRegisters(settingX_write);
X=readRegisters(settingX_read);
writeRegisters(settingY_write);
Y=readRegisters(settingY_read);
writeRegisters(settingZ_write);
Z=readRegisters(settingZ_read);
writeRegisters(settingT_write);
T=readRegisters(settingT_read);
}
  delay(100);
  printValues();
}
