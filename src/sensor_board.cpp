#include <VL53L0X.h> 
 
// TOF Sensor
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;
VL53L0X sensor6;
VL53L0X sensor7;
VL53L0X sensor8;
 
uint16_t s1;
uint16_t s2;
uint16_t s3;
uint16_t s4;
uint16_t s5;
uint16_t s6;
uint16_t s7;
uint16_t s8;
 
// SPI Data
volatile uint8_t spi_data[9] = {250,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
volatile uint8_t tx_index = 0;
 
void setup()
{
  // Initialize SPI1 as Slave with interrupt
  SPI1_Init();                
  // Enable the SPI1 interrupt in NVIC
  NVIC_EnableIRQ(SPI1_IRQn);  
   
  // Initialize TOF
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PC3, OUTPUT);
  pinMode(PB2, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB0, OUTPUT);
  pinMode(PC5, OUTPUT);
 
  digitalWrite(PB12, LOW);
  digitalWrite(PB13, LOW);
  digitalWrite(PB14, LOW);
  digitalWrite(PC3, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB0, LOW);
  digitalWrite(PC5, LOW);
 
  Wire.setSDA(PB4);
  Wire.setSCL(PA8);
 
  Wire.begin();
 
  digitalWrite(PB12, HIGH);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)00);
   
  digitalWrite(PB13, HIGH);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)01);
   
  digitalWrite(PB14, HIGH);
  delay(150);
  sensor3.init(true);
  delay(100);
  sensor3.setAddress((uint8_t)02);
  
  digitalWrite(PC3, HIGH);
  delay(150);
  sensor4.init(true);
  delay(100);
  sensor4.setAddress((uint8_t)03);
  
  digitalWrite(PB2, HIGH);
  delay(150);
  sensor5.init(true);
  delay(100);
  sensor5.setAddress((uint8_t)04);
  
  digitalWrite(PB1, HIGH);
  delay(150);
  sensor6.init(true);
  delay(100);
  sensor6.setAddress((uint8_t)05);
  
  digitalWrite(PB0, HIGH);
  delay(150);
  sensor7.init(true);
  delay(100);
  sensor7.setAddress((uint8_t)06);
  
  digitalWrite(PC5, HIGH);
  delay(150);
  sensor8.init(true);
  delay(100);
  sensor8.setAddress((uint8_t)07);
  
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
  sensor5.startContinuous();
  sensor6.startContinuous();
  sensor7.startContinuous();
  sensor8.startContinuous();
  
  // User Code
 
}
 
void loop()
{
  sensor1.setMeasurementTimingBudget(100000);
  sensor2.setMeasurementTimingBudget(100000);
  sensor3.setMeasurementTimingBudget(100000);
  sensor4.setMeasurementTimingBudget(100000);
  sensor5.setMeasurementTimingBudget(100000);
  sensor6.setMeasurementTimingBudget(100000);
  sensor7.setMeasurementTimingBudget(100000);
  sensor8.setMeasurementTimingBudget(100000);
  s1 = sensor1.readRangeContinuousMillimeters();
  s2 = sensor2.readRangeContinuousMillimeters();
  s3 = sensor3.readRangeContinuousMillimeters();
  s4 = sensor4.readRangeContinuousMillimeters();
  s5 = sensor5.readRangeContinuousMillimeters();
  s6 = sensor6.readRangeContinuousMillimeters();
  s7 = sensor7.readRangeContinuousMillimeters();
  s8 = sensor8.readRangeContinuousMillimeters();
  
  // TODO: Need to do right conversions here, i need to split stuff over two spi transfers each
 
  if (s1<125) spi_data[1] = s1; else spi_data[1] = 0;
  if (s2<125) spi_data[2] = s2; else spi_data[2] = 0;
  if (s3<125) spi_data[3] = s3; else spi_data[3] = 0;
  if (s4<125) spi_data[4] = s4; else spi_data[4] = 0;
  if (s5<125) spi_data[5] = s5; else spi_data[5] = 0;
  if (s6<125) spi_data[6] = s6; else spi_data[6] = 0;
  if (s7<125) spi_data[7] = s7; else spi_data[7] = 0;
  if (s8<125) spi_data[8] = s8; else spi_data[8] = 0;
   
  // User Code
   
}
 
// Initialize SPI1 in slave mode with interrupt
void SPI1_Init() 
{
  // Enable clocks for GPIOA and SPI1
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
 
  // Configure PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
  GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (5 << (4 * 4)) | (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));  // Set AF5 for SPI1
 
  // Configure SPI1 as a slave
  SPI1->CR1 = 0;                 // Reset CR1
  SPI1->CR1 &= ~SPI_CR1_MSTR;    // Set as slave
  SPI1->CR1 &= ~SPI_CR1_SSM;     // Disable software slave management
  SPI1->CR1 |= SPI_CR1_SPE;      // Enable SPI
  SPI1->CR2 |= SPI_CR2_RXNEIE;   // Enable RXNE interrupt
  SPI1->CR2 |= SPI_CR2_TXEIE;    // Enable TXE interrupt
}
 
extern "C" void SPI1_IRQHandler(void)
{
   // Check if RXNE (Receive Not Empty) flag is set and handle RX first
    if (SPI1->SR & SPI_SR_RXNE)
    {
      uint8_t rx_value = SPI1->DR;  
    }
      // Check if TXE (Transmit Empty) flag is set, but only if not busy
    if (SPI1->SR & SPI_SR_TXE)
    {
        // Send data from tx_buffer if in "send mode"
        if (tx_index < 9)
        {
          // Write data to SPI
          SPI1->DR = spi_data[tx_index];  
          tx_index++;
        }
        else
        {
          // Reset buffer index
          tx_index = 0;                      
        }
    }
}