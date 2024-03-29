/**
 * @file max11635.cpp
 * @date 2023-03-26
 * @author Robert Morley (robert.morley@utoronto.ca)
 * 
 * @brief Analog Devices (formerly Maxium Integrated) MAX11635 class source code.
 * 
 * @version 1.0.0
 * @copyright Copyright (c) 2023
 */
#include "max11635.h"

arduino::SPISettings max11635::driver::default_setting = SPISettings(fclock_default, MSBFIRST, SPIMode::SPI_MODE0);

max11635::driver MAX11635_ADC { &SPI }; /// Default SPI0 

/**
 * @brief Default Interrupt handler for the MAX11635 ADC on the Phyduino Board, can be overridden by function of the same name.
 */
void MAX11635_InteruptHandler() noexcept {
  max11635::driver::InterruptHandler(&MAX11635_ADC);
}

/**
 * @defgroup MAX11635_DRV_PUBLIC MAX11635 Driver public functions
 * @{
 */
void max11635::driver::InterruptHandler(driver* drv) noexcept{
  drv->current_sample = drv->read_conversion();
  drv->sample_ready = true;
}
/**
 * @brief Initializes and Starts MAX11635 driver.
 */
void max11635::driver::begin() noexcept {
  if(_settings == nullptr){
    _settings = &default_setting;
  }

  initialize();
}
/**
 * @brief Initializes and Starts MAX11635 driver.
 * @param settings [in] Pointer to SPI Settings.
 */
void max11635::driver::begin(arduino::SPISettings* settings) noexcept {
  _settings = settings;
  begin();
}
/**
 * @brief Stops MAX11635 driver.
 */
void max11635::driver::end() noexcept {
  if(!is_initialized()){
    _bus->end();
  }
}
/**
 * @brief Set analog channel selection.
 * @param channel [in] Analog channel number selection [0-3]
 * @return true Channel set.
 * @return false Invalid channel number given.
 */
bool max11635::driver::set_channel(const std::uint8_t channel) noexcept {
  if(channel > driver::max_channels - 1){
    return false;
  }

  _regs.conversion.b.CHSEL = static_cast<std::uint8_t>(channel);                // Select channel.
  _regs.conversion.b.SCAN  = 0b11U;                                             // No Scan, Converts N channel once.
  write_nbytes(&_regs.conversion.w);

  return true;
}
/**
 * @brief Starts analog conversion.
 */
void max11635::driver::start_conversion() const noexcept {
  digitalWrite(_n_cnvst, LOW);
  delayMicroseconds(1);
  digitalWrite(_n_cnvst, HIGH);
}
/**
 * @brief Gets Conversion Value from ADC.
 * @return max11635::driver::data_type ADC conversion value
 */
max11635::driver::data_type max11635::driver::read_conversion() noexcept {
  return this->get_conversion();
}
/**
 * @brief Reads any available analog channel on the Phyduino.
 * @param analog_pin [in] Analog pin number. valid values 0 - 3
 * @return Number of Bytes transferred.
 */
max11635::driver::data_type max11635::driver::analogRead(std::uint8_t channel) noexcept {
  #ifdef MAX11635_DEBUG_L2
    Serial.printf("Selecting external channel : %d\r\n", channel);
  #endif

  if(!this->set_channel(channel)){    // Set channel for next conversion.
    return 0;
  }
  
  start_conversion();                 // Start conversion

  #ifdef MAX11635_DEBUG_L2
    Serial.println("Starting Analog to digital conversion.");
  #endif
  
  while(!sample_ready);               // Wait for conversion to finish. (flag set in interrupt function MAX11635_InteruptHandler())
  sample_ready = false;               // Reset sample flag for next conversion.

  #ifdef MAX11635_DEBUG_L2
    Serial.println("Conversion complete.");
  #endif
  
  return current_sample; // Return new sample.
}

/**
 * @brief Configures IO for SPI/GPIO interface.
 * @param mosi [in] SPI MOSI IO index.
 * @param miso [in] SPI MISO IO index
 * @param sck [in] SPI SCK IO index
 * @param cs [in] SPI CS IO index
 * @param ncnvst [in] nConvert IO index
 * @param n_eoc [in] nEOC IO index
 */
void max11635::driver::configure_io(pin_t mosi, 
    pin_t miso, 
    pin_t sck, 
    pin_t cs, 
    pin_t ncnvst, 
    pin_t n_eoc
  ) noexcept {
    _mosi = mosi;
    _miso = miso;
    _sck = sck;
    _cs = cs;
    _n_cnvst = ncnvst;
    _n_eoc = n_eoc;
}
/**
 * @brief [static] Converts the ADC value to a voltage.
 */
float max11635::driver::to_voltage(const data_type val) noexcept{
  float voltage = static_cast<float>(val) * max11635::driver::v_resolution;
  return voltage;
}
/**
 * @brief 
 * @param channel [in] sampled channel number.
 * @return float Value of the signal in volts.
 */
float max11635::driver::get_voltage(std::uint8_t channel) noexcept{
  auto value = analogRead(channel);
  return max11635::driver::to_voltage(value);
}
/** @} */

/**
 * @defgroup MAX11635_DRV_PRIVATE MAX11635 Driver private functions
 * @{
 */

/**
 * @brief Write N bytes to the ADC
 * @param buf [in] Pointer to Write buffer.
 * @param n [in] Number of bytes to be written.
 */
void max11635::driver::write_nbytes(std::uint8_t* buf, const std::size_t n) noexcept {
  if(!is_initialized()){
    return;
  }

  #ifdef MAX11635_DEBUG_L3
    if(n > 1){
      Serial.printf("Write N(%d) bytes :\r\n", n);
      for(std::size_t i = 0; i < n; i++){
        Serial.printf("\t[%d] : 0x%02x\r\n", i, *(buf + i));
      }
    } else {
      Serial.printf("Write %d byte : 0x%02x\r\n", n, *buf);
    }
  #endif

  _bus->beginTransaction(*_settings);
  digitalWrite(_cs, LOW);
  
  for(std::size_t i = 0; i < n; i++){
    _bus->transfer(*(buf + i));
  }
  
  digitalWrite(_cs, HIGH);
  _bus->endTransaction();
  // delayMicrosecond(5);    // Test delay for stablity of SPI interface
}
/**
 * @brief Read N bytes from MAX11635
 * @param buf [out] Pointer to data buffer.
 * @param n Number of bytes to be read.
 */
void max11635::driver::read_nbytes(std::uint8_t* buf, const std::size_t n) noexcept {
  if(!is_initialized()){
    return;
  }

  _bus->beginTransaction(*_settings);
  digitalWrite(_cs, LOW);
  
  for(std::size_t i = 0; i < n; i++){
    *(buf + i) = _bus->transfer(dummy_byte);
  }
  
  digitalWrite(_cs, HIGH);
  _bus->endTransaction();
  // delayMicrosecond(5);    // Test delay for stablity of SPI interface
}
/**
 * @brief Initialize MAX11635 ADC.
 */
void max11635::driver::initialize() noexcept {
  if(_bus == nullptr){
    initialized = false;
    return;
  }

  // Set GPIO pins for ADC.
  pinMode(_n_eoc, INPUT_PULLUP);
  pinMode(_n_cnvst, OUTPUT);

  digitalWrite(_n_cnvst, HIGH);
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  pinMode(_mosi, OUTPUT);

  #ifndef SPI_CLASS_INIT_IO
    [[maybe_unused]] auto is_set = _bus->setRX(static_cast<pin_t>(_miso));
    #ifdef MAX11635_DEBUG_L2 
      Serial.print("MISO is set : ");
      Serial.println((is_set? "YES" : "NO"));
    #endif
    is_set = _bus->setTX(static_cast<pin_t>(_mosi));
    #ifdef MAX11635_DEBUG_L2
      Serial.print("MOSI is set : ");
      Serial.println((is_set? "YES" : "NO"));
    #endif
    is_set = _bus->setSCK(static_cast<pin_t>(_sck));
    #ifdef MAX11635_DEBUG_L2
      Serial.print("SCK is set : ");
      Serial.println((is_set? "YES" : "NO"));
      Serial.println();
    #endif
    is_set = _bus->setCS(static_cast<pin_t>(_cs));
    #ifdef MAX11635_DEBUG_L2
      Serial.print("CS is set : ");
      Serial.println((is_set? "YES" : "NO"));
      Serial.println();
    #endif
    // Initialize SPI class.
  #else
    gpio_set_function(_miso, GPIO_FUNC_SPI);  
    gpio_set_function(_mosi, GPIO_FUNC_SPI);  
    gpio_set_function(_sck, GPIO_FUNC_SPI);  
  #endif
 
  _bus->begin();  // Start SPI Class driver.
  config_regs();

  // Clear EOC pin before starting.
  if(digitalRead(_n_eoc) == LOW){
    this->clear_nEOC();
  }

  attachInterrupt(digitalPinToInterrupt(_n_eoc), MAX11635_InteruptHandler, LOW);

  initialized = true;
}
/**
 * @brief Configures ADC to read analog data.
 */
void max11635::driver::config_regs() noexcept{
  _regs.reset_all();                        // Reset all register values.
  // Reset Register
  _regs.reset = 0b1 << 3;            // Clear FIFO and reset registers.
  write_nbytes(&_regs.reset.w);
  // Conversion Register
  _regs.conversion.b.CHSEL = 0b000U;    // AIN1 selected.
  _regs.conversion.b.SCAN  = 0b11U;     // No scan selection
  write_nbytes(&_regs.conversion.w);
  // Setup Register
  _regs.setup.b.CKSEL   = 0b00U;        // Set Conversion Clock to internal. use nCVST to start conversion and wait for nEOC before reading data.
  _regs.setup.b.REFSEL  = 0b01U;        // Voltage reference External single-ended
  _regs.setup.b.DIFFSEL = 0b10U;        // 1 byte of data follows the setup byte and is written to the unipolar mode register.
  _regs.unipolar.w      = 0x00U;        // 1 to configure AIN0 - AIN3 for unipolar differential conversion
  std::uint8_t setup[2] { _regs.setup.w, _regs.unipolar.w };
  write_nbytes(setup, sizeof(setup));
  // Averaging Register
  _regs.averaging = 0x00;
  write_nbytes(&_regs.averaging.w);

  #ifdef MAX11635_DEBUG_L2
    Serial.println("MAX11365 ADC Initialized.\r\n");
    Serial.println("Register settings:");
    Serial.printf("\treset      : 0x%02x\r\n", _regs.reset.w);
    Serial.printf("\tconversion : 0x%02x\r\n", _regs.conversion.w);
    Serial.printf("\tsetup      : 0x%02x\r\n", _regs.setup.w);
    Serial.printf("\tunipolar   : 0x%02x\r\n", _regs.unipolar.w);
    Serial.printf("\tAveraging  : 0x%02x\r\n", _regs.averaging.w);
    Serial.println();
  #endif
}
/**
 * @brief Get the conversion value for the ADC. This should be call after nEOC pin goes LOW.
 */
max11635::driver::data_type max11635::driver::get_conversion() noexcept {
  // static int j = 0;
  std::uint8_t adc_rd[2] { };

  read_nbytes(adc_rd, sizeof(adc_rd));

  data_type analog_val = static_cast<data_type>(adc_rd[0]) << 8U;
  analog_val |= static_cast<data_type>(adc_rd[1]);

  // Corrects ADC value count, Calibration should be measure and set in the user program.
  current_sample = analog_val == 0 ? analog_val : analog_val + calibration_offset;

  return current_sample;
}

/** @} */
