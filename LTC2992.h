// Chidobem Udejiofor 5/30/2024
// LTC 2992 Basic Functionality Library

#define DEFAULT_PORT 0 // Should match Bus Number

// Exposed Data Registers
#define LTC2992_CTRLA_REG 0x00
#define LTC2992_CTRLB_REG 0x01
#define LTC2992_ALERT1_REG 0x02
#define LTC2992_FAULT1_REG 0x03
#define LTC2992_NADC_REG 0x04
#define LTC2992_POWER1_MSB2_REG 0x05
#define LTC2992_POWER1_MSB1_REG 0x06
#define LTC2992_POWER1_LSB_REG 0x07
#define LTC2992_MAX_POWER1_MSB2_REG 0x08
#define LTC2992_MAX_POWER1_MSB1_REG 0x09
#define LTC2992_MAX_POWER1_LSB_REG 0x0A
#define LTC2992_MIN_POWER1_MSB2_REG 0x0B
#define LTC2992_MIN_POWER1_MSB1_REG 0x0C
#define LTC2992_MIN_POWER1_LSB_REG 0x0D
#define LTC2992_MAX_POWER1_THRESHOLD_MSB2_REG 0x0E
#define LTC2992_MAX_POWER1_THRESHOLD_MSB1_REG 0x0F
#define LTC2992_MAX_POWER1_THRESHOLD_LSB_REG 0x10
#define LTC2992_MIN_POWER1_THRESHOLD_MSB2_REG 0x11
#define LTC2992_MIN_POWER1_THRESHOLD_MSB1_REG 0x12
#define LTC2992_MIN_POWER1_THRESHOLD_LSB_REG 0x13
#define LTC2992_DELTA_SENSE1_MSB_REG 0x14
#define LTC2992_DELTA_SENSE1_LSB_REG 0x15
#define LTC2992_MAX_DELTA1_SENSE_MSB_REG 0x16
#define LTC2992_MAX_DELTA1_SENSE_LSB_REG 0x17
#define LTC2992_MIN_DELTA1_SENSE_MSB_REG 0x18
#define LTC2992_MIN_DELTA1_SENSE_LSB_REG 0x19
#define LTC2992_MAX_DELTA1_SENSE_THRESHOLD_MSB_REG 0x1A
#define LTC2992_MAX_DELTA1_SENSE_THRESHOLD_LSB_REG 0x1B
#define LTC2992_MIN_DELTA1_SENSE_THRESHOLD_MSB_REG 0x1C
#define LTC2992_MIN_DELTA1_SENSE_THRESHOLD_LSB_REG 0x1D
#define LTC2992_SENSE1_MSB_REG 0x1E
#define LTC2992_SENSE1_LSB_REG 0x1F
#define LTC2992_MAX_SENSE1_MSB_REG 0x20
#define LTC2992_MAX_SENSE1_LSB_REG 0x21
#define LTC2992_MIN_SENSE1_MSB_REG 0x22
#define LTC2992_MIN_SENSE1_LSB_REG 0x23
#define LTC2992_MAX_SENSE1_THRESHOLD_MSB_REG 0x24
#define LTC2992_MAX_SENSE1_THRESHOLD_LSB_REG 0x25
#define LTC2992_MIN_SENSE1_THRESHOLD_MSB_REG 0x26
#define LTC2992_MIN_SENSE1_THRESHOLD_LSB_REG 0x27
#define LTC2992_GPIO1_MSB_REG 0x28
#define LTC2992_GPIO1_LSB_REG_REG 0x29
#define LTC2992_MAX_GPIO1_MSB_REG 0x2A
#define LTC2992_MAX_GPIO1_LSB_REG 0x2B
#define LTC2992_MIN_GPIO1_MSB_REG 0x2C
#define LTC2992_MIN_GPIO1_LSB_REG 0x2D
#define LTC2992_MAX_GPIO1_THRESHOLD_MSB_REG 0x2E
#define LTC2992_MAX_GPIO1_THRESHOLD_LSB_REG 0x2F
#define LTC2992_MIN_GPIO1_THRESHOLD_MSB_REG 0x30
#define LTC2992_MIN_GPIO1_THRESHOLD_LSB_REG 0x31
#define LTC2992_ADC_STATUS_REG 0x32
#define LTC2992_ALERT2_REG 0x34
#define LTC2992_FAULT2_REG 0x35
#define LTC2992_POWER2_MSB2_REG 0x37
#define LTC2992_POWER2_MSB1_REG 0x38
#define LTC2992_POWER2_LSB_REG 0x39
#define LTC2992_MAX_POWER2_MSB2_REG 0x3A
#define LTC2992_MAX_POWER2_MSB1_REG 0x3B
#define LTC2992_MAX_POWER2_LSB_REG 0x3C
#define LTC2992_MIN_POWER2_MSB2_REG 0x3D
#define LTC2992_MIN_POWER2_MSB1_REG 0x3E
#define LTC2992_MIN_POWER2_LSB_REG 0x3F
#define LTC2992_MAX_POWER2_THRESHOLD_MSB2_REG 0x40
#define LTC2992_MAX_POWER2_THRESHOLD_MSB1_REG 0x41
#define LTC2992_MAX_POWER2_THRESHOLD_LSB_REG 0x42
#define LTC2992_MIN_POWER2_THRESHOLD_MSB2_REG 0x43
#define LTC2992_MIN_POWER2_THRESHOLD_MSB1_REG 0x44
#define LTC2992_MIN_POWER2_THRESHOLD_LSB_REG 0x45
#define LTC2992_DELTA_SENSE2_MSB_REG 0x46
#define LTC2992_DELTA_SENSE2_LSB_REG 0x47
#define LTC2992_MAX_DELTA2_SENSE_MSB_REG 0x48
#define LTC2992_MAX_DELTA2_SENSE_LSB_REG 0x49
#define LTC2992_MIN_DELTA2_SENSE_MSB_REG 0x4A
#define LTC2992_MIN_DELTA2_SENSE_LSB_REG 0x4B
#define LTC2992_MAX_DELTA2_SENSE_THRESHOLD_MSB_REG 0x4C
#define LTC2992_MAX_DELTA2_SENSE_THRESHOLD_LSB_REG 0x4D
#define LTC2992_MIN_DELTA2_SENSE_THRESHOLD_MSB_REG 0x4E
#define LTC2992_MIN_DELTA2_SENSE_THRESHOLD_LSB_REG 0x4F
#define LTC2992_SENSE2_MSB_REG 0x50
#define LTC2992_SENSE2_LSB_REG 0x51
#define LTC2992_MAX_SENSE2_MSB_REG 0x52
#define LTC2992_MAX_SENSE2_LSB_REG 0x53
#define LTC2992_MIN_SENSE2_MSB_REG 0x54
#define LTC2992_MIN_SENSE2_LSB_REG 0x55
#define LTC2992_MAX_SENSE2_THRESHOLD_MSB_REG 0x56
#define LTC2992_MAX_SENSE2_THRESHOLD_LSB_REG 0x57
#define LTC2992_MIN_SENSE2_THRESHOLD_MSB_REG 0x58
#define LTC2992_MIN_SENSE2_THRESHOLD_LSB_REG 0x59
#define LTC2992_GPIO2_MSB_REG 0x5A
#define LTC2992_GPIO2_LSB_REG_REG 0x5B
#define LTC2992_MAX_GPIO2_MSB_REG 0x5C
#define LTC2992_MAX_GPIO2_LSB_REG 0x5D
#define LTC2992_MIN_GPIO2_MSB_REG 0x5E
#define LTC2992_MIN_GPIO2_LSB_REG 0x5F
#define LTC2992_MAX_GPIO2_THRESHOLD_MSB_REG 0x60
#define LTC2992_MAX_GPIO2_THRESHOLD_LSB_REG 0x61
#define LTC2992_MIN_GPIO2_THRESHOLD_MSB_REG 0x62
#define LTC2992_MIN_GPIO2_THRESHOLD_LSB_REG 0x63
#define LTC2992_GPIO3_MSB_REG 0x64
#define LTC2992_GPIO3_LSB_REG_REG 0x65
#define LTC2992_MAX_GPIO3_MSB_REG 0x66
#define LTC2992_MAX_GPIO3_LSB_REG 0x67
#define LTC2992_MIN_GPIO3_MSB_REG 0x68
#define LTC2992_MIN_GPIO3_LSB_REG 0x69
#define LTC2992_MAX_GPIO3_THRESHOLD_MSB_REG 0x6A
#define LTC2992_MAX_GPIO3_THRESHOLD_LSB_REG 0x6B
#define LTC2992_MIN_GPIO3_THRESHOLD_MSB_REG 0x6C
#define LTC2992_MIN_GPIO3_THRESHOLD_LSB_REG 0x6D
#define LTC2992_GPIO4_MSB_REG 0x6E
#define LTC2992_GPIO4_LSB_REG_REG 0x6F
#define LTC2992_MAX_GPIO4_MSB_REG 0x70
#define LTC2992_MAX_GPIO4_LSB_REG 0x71
#define LTC2992_MIN_GPIO4_MSB_REG 0x72
#define LTC2992_MIN_GPIO4_LSB_REG 0x73
#define LTC2992_MAX_GPIO4_THRESHOLD_MSB_REG 0x74
#define LTC2992_MAX_GPIO4_THRESHOLD_LSB_REG 0x75
#define LTC2992_MIN_GPIO4_THRESHOLD_MSB_REG 0x76
#define LTC2992_MIN_GPIO4_THRESHOLD_LSB_REG 0x77
#define LTC2992_ISUM_MSB_REG 0x78
#define LTC2992_ISUM_LSB_REG_REG 0x79
#define LTC2992_MAX_ISUM_MSB_REG 0x7A
#define LTC2992_MAX_ISUM_LSB_REG 0x7B
#define LTC2992_MIN_ISUM_MSB_REG 0x7C
#define LTC2992_MIN_ISUM_LSB_REG 0x7D
#define LTC2992_MAX_ISUM_THRESHOLD_MSB_REG 0x7E
#define LTC2992_MAX_ISUM_THRESHOLD_LSB_REG 0x7F
#define LTC2992_MIN_ISUM_THRESHOLD_MSB_REG 0x80
#define LTC2992_MIN_ISUM_THRESHOLD_LSB_REG 0x81
#define LTC2992_PSUM_MSB1_REG 0x82
#define LTC2992_PSUM_MSB_REG 0x83
#define LTC2992_PSUM_LSB_REG_REG 0x84
#define LTC2992_MAX_PSUM_MSB1_REG 0x85
#define LTC2992_MAX_PSUM_MSB_REG 0x86
#define LTC2992_MAX_PSUM_LSB_REG 0x87
#define LTC2992_MIN_PSUM_MSB1_REG 0x88
#define LTC2992_MIN_PSUM_MSB_REG 0x89
#define LTC2992_MIN_PSUM_LSB_REG 0x8A
#define LTC2992_MAX_PSUM_THRESHOLD_MSB1_REG 0x8B
#define LTC2992_MAX_PSUM_THRESHOLD_MSB_REG 0x8C
#define LTC2992_MAX_PSUM_THRESHOLD_LSB_REG 0x8D
#define LTC2992_MIN_PSUM_THRESHOLD_MSB1_REG 0x8E
#define LTC2992_MIN_PSUM_THRESHOLD_MSB_REG 0x8F
#define LTC2992_MIN_PSUM_THRESHOLD_LSB_REG 0x90
#define LTC2992_ALERT3_REG 0x91
#define LTC2992_FAULT3_REG 0x92
#define LTC2992_ALERT4_REG 0x93
#define LTC2992_FAULT4_REG 0x94
#define LTC2992_GPIO_STATUS_REG 0x95
#define LTC2992_GPIO_IO_CONT_REG 0x96
#define LTC2992_GPIO4_CFG_REG 0x97

// Command Codes
#define LTC2992_OFFSET_CAL_DEMAND 0x80
#define LTC2992_OFFSET_CAL_EVERY 0x00

#define LTC2992_MODE_SHUTDOWN 0x60
#define LTC2992_MODE_SINGLE_CYCLE 0x40
#define LTC2992_MODE_SNAPSHOT 0x20
#define LTC2992_MODE_CONTINUOUS 0x00

#define LTC2992_CONTINUOUS_GPIO_1_2_3_4 0x18
#define LTC2992_CONTINUOUS_GPIO_1_2 0x10
#define LTC2992_CONTINUOUS_SENSE_1_2 0x08
#define LTC2992_CONTINUOUS_SENSE_1_2_GPIO_1_2_3_4 0x00

#define LTC2992_SNAPSHOT_GPIO_1_2 0x07
#define LTC2992_SNAPSHOT_SENSE_1_2 0x06
#define LTC2992_SNAPSHOT_GPIO_4 0x05
#define LTC2992_SNAPSHOT_GPIO_3 0x04
#define LTC2992_SNAPSHOT_GPIO_2 0x03
#define LTC2992_SNAPSHOT_GPIO_1 0x02
#define LTC2992_SNAPSHOT_SENSE_2 0x01
#define LTC2992_SNAPSHOT_SENSE_1 0x00

#define LTC2992_ENABLE_ALERT_CLEAR 0x80
#define LTC2992_ENABLE_CLEARED_ON_READ 0x20
#define LTC2992_ENABLE_STUCK_BUS_RECOVER 0x10
#define LTC2992_ENABLE_RESET_PEAK_VALUES 0x08
#define LTC2992_ENABLE_RESET_ALL 0x01

#define LTC2992_DISABLE_ALERT_CLEAR 0x7F
#define LTC2992_DISABLE_CLEARED_ON_READ 0xDF
#define LTC2992_DISABLE_STUCK_BUS_RECOVER 0xEF
#define LTC2992 DISABLE_RESET_PEAK_VALUES 0xF7
#define LTC2992_DISABLE_RESET_ALL 0xFE

#define LTC2992_ENABLE_MAX_POWER1_ALERT 0x80
#define LTC2992_ENABLE_MIN_POWER1_ALERT 0x40
#define LTC2992_DISABLE_MAX_POWER1_ALERT 0x7F
#define LTC2992_DISABLE_MIN_POWER1_ALERT 0xBF

#define LTC2992_ENABLE_MAX_I_SENSE1_ALERT 0x20
#define LTC2992_ENABLE_MIN_I_SENSE1_ALERT 0x10
#define LTC2992_DISABLE_MAX_I_SENSE1_ALERT 0xDF
#define LTC2992_DISABLE_MIN_I_SENSE1_ALERT 0xEF

#define LTC2992_ENABLE_MAX_SENSE1_ALERT 0x08
#define LTC2992_ENABLE_MIN_SENSE1_ALERT 0x04
#define LTC2992_DISABLE_MAX_SENSE1_ALERT 0xF7
#define LTC2992_DISABLE_MIN_SENSE1_ALERT 0xFB

#define LTC2992_ENABLE_MAX_GPIO1_ALERT 0x02
#define LTC2992_ENABLE_MIN_GPIO1_ALERT 0x01
#define LTC2992_DISABLE_MAX_GPIO1_ALERT 0xFD
#define LTC2992_DISABLE_MIN_GPIO1_ALERT 0xFE

// NADC
#define LTC2992_ADC_RESOLUTION 0x80

// ALERT2
#define LTC2992_ENABLE_MAX_POWER2_ALERT 0x80
#define LTC2992_ENABLE_MIN_POWER2_ALERT 0x40
#define LTC2992_DISABLE_MAX_POWER2_ALERT 0x7F
#define LTC2992_DISABLE_MIN_POWER2_ALERT 0xBF

#define LTC2992_ENABLE_MAX_I_SENSE2_ALERT 0x20
#define LTC2992_ENABLE_MIN_I_SENSE2_ALERT 0x10
#define LTC2992_DISABLE_MAX_I_SENSE2_ALERT 0xDF
#define LTC2992_DISABLE_MIN_I_SENSE2_ALERT 0xEF

#define LTC2992_ENABLE_MAX_SENSE2_ALERT 0x08
#define LTC2992_ENABLE_MIN_SENSE2_ALERT 0x04
#define LTC2992_DISABLE_MAX_SENSE2_ALERT 0xF7
#define LTC2992_DISABLE_MIN_SENSE2_ALERT 0xFB

#define LTC2992_ENABLE_MAX_GPIO2_ALERT 0x02
#define LTC2992_ENABLE_MIN_GPIO2_ALERT 0x01
#define LTC2992_DISABLE_MAX_GPIO2_ALERT 0xFD
#define LTC2992_DISABLE_MIN_GPIO2_ALERT 0xFE

// ALERT3
#define LTC2992_ENABLE_MAX_GPIO3_ALERT 0x80
#define LTC2992_ENABLE_MIN_GPIO3_ALERT 0x40
#define LTC2992_DISABLE_MAX_GPIO3_ALERT 0x7F
#define LTC2992_DISABLE_MIN_GPIO3_ALERT 0xBF

#define LTC2992_ENABLE_MAX_GPIO4_ALERT 0x20
#define LTC2992_ENABLE_MIN_GPIO4_ALERT 0x10
#define LTC2992_DISABLE_MAX_GPIO4_ALERT 0xDF
#define LTC2992_DISABLE_MIN_GPIO4_ALERT 0xEF

#define LTC2992_ENABLE_MAX_I_SUM_ALERT 0x08
#define LTC2992_ENABLE_MIN_I_SUM_ALERT 0x04
#define LTC2992_DISABLE_MAX_I_SUM_ALERT 0xF7
#define LTC2992_DISABLE_MIN_I_SUM_ALERT 0xFB

#define LTC2992_ENABLE_MAX_P_SUM_ALERT 0x02
#define LTC2992_ENABLE_MIN_P_SUM_ALERT 0x01
#define LTC2992_DISABLE_MAX_P_SUM_ALERT 0xFD
#define LTC2992_DISABLE_MIN_P_SUM_ALERT 0xFE

// ALERT4
#define LTC2992_ENABLE_V_ADC_READY_ALERT 0x80
#define LTC2992_ENABLE_I_ADC_READY_ALERT 0x40
#define LTC2992_DISABLE_V_ADC_READY_ALERT 0x7F
#define LTC2992_DISABLE_I_ADC_READY_ALERT 0xBF

#define LTC2992_ENABLE_STUCK_BUS_TIMEOUT_ALERT 0x10
#define LTC2992_DISABLE_STUCK_BUS_TIMEOUT_ALERT 0xEF

#define LTC2992_ENABLE_GPIO1_INPUT_ALERT 0x08
#define LTC2992_ENABLE_GPIO2_INPUT_ALERT 0x04
#define LTC2992_DISABLE_GPIO1_INPUT_ALERT 0xF7
#define LTC2992_DISABLE_GPIO2_INPUT_ALERT 0xFB

#define LTC2992_ENABLE_GPIO3_INPUT_ALERT 0x02
#define LTC2992_DISABLE_GPIO3_INPUT_ALERT 0xFD

// GPIO IO Control Registers
#define LTC2992_GPIO1_OUT_HIGH_Z 0x7F
#define LTC2992_GPIO1_OUT_LOW 0x80
#define LTC2992_GPIO2_OUT_HIGH_Z 0xBF
#define LTC2992_GPIO2_OUT_LOW 0x40

#define LTC2992_GPIO3_CONFIG_LOW_DATARDY 0x30
#define LTC2992_GPIO3_CONFIG_128_LOW 0x20
#define LTC2992_GPIO3_CONFIG_16_LOW 0x10
#define LTC2992_GPIO3_CONFIG_IO 0x00

#define LTC2992_GPIO1_IN_HIGH_POL_ALERT 0x08
#define LTC2992_GPIO1_IN_LOW_POL_ALERT 0xFC
#define LTC2992_GPIO2_IN_HIGH_POL_ALERT 0x04
#define LTC2992_GPIO2_IN_LOW_POL_ALERT 0xFB
#define LTC2992_GPIO3_IN_HIGH_POL_ALERT 0x02
#define LTC2992_GPIO3_IN_LOW_POL_ALERT 0xFD
#define LTC2992_GPIO3_OUT_LOW 0x01
#define LTC2992_GPIO3_OUT_HIGH_Z 0xFE

// GPIO4 Control Reg
#define LTC2992_ALERT_GENERATED_TRUE 0x80
#define LTC2992_ALERT_GENERATED_CLEAR 0x7F
#define LTC2992_GPIO4_OUT_LOW 0x40
#define LTC2992_GPIO4_OUT_HI_Z 0xBF

// Register Mask Command
#define LTC2992_CTRLA_OFFSET_MASK 0x7F
#define LTC2992_CTRLA_MEASUREMENT_MODE_MASK 0x9F
#define LTC2992_CTRLA_VOLTAGE_SEL_CONTINIOUS_MASK 0xE7
#define LTC2992_CTRLA_VOLTAGE_SEL_SNAPSHOT_MASK 0xF8
#define LTC2992_CTRLB_ACC_MASK 0xF3
#define LTC2992_CTRLB_RESET_MASK 0xFC
#define LTC2992_GPIOCFG_GPIO1_MASK 0x3F
#define LTC2992_GPIOCFG_GPIO2_MASK 0xCF
#define LTC2992_GPIOCFG_GPIO3_MASK 0xCF
#define LTC2992_GPIOCFG_GPIO2_OUT_MASK 0xFD
#define LTC2992_GPIO3_CTRL_GPIO3_MASK 0xBF


// Typical MSB Sensitivities
const float DSENSE_LSB_8BIT = 200e-6;                            // Volts
const float SENSE_LSB_8BIT = 400e-3;                             // Volts
const float GPIO_LSB_8BIT = 8e-3;                                // Volts
const float POWER_LSB_8BIT = DSENSE_LSB_8BIT *SENSE_LSB_8BIT;    // Volts

const float DSENSE_LSB_12BIT = 12.5e-6;                          // Volts
const float SENSE_LSB_12BIT = 25e-3;                             // Volts
const float GPIO_LSB_12BIT = .5e-3;                              // Volts
const float POWER_LSB_12BIT = DSENSE_LSB_12BIT *SENSE_LSB_12BIT; // Volts

// GPIO Data Registers
static const uint8_t gpio_regs[] = {LTC2992_GPIO1_MSB_REG, LTC2992_GPIO2_MSB_REG, LTC2992_GPIO3_MSB_REG, LTC2992_GPIO4_MSB_REG};
static const uint8_t gpio_max_regs[] = {LTC2992_MAX_GPIO1_MSB_REG, LTC2992_MAX_GPIO2_MSB_REG, LTC2992_MAX_GPIO3_MSB_REG, LTC2992_MAX_GPIO4_MSB_REG};
static const uint8_t gpio_min_regs[] = {LTC2992_MIN_GPIO1_MSB_REG, LTC2992_MIN_GPIO2_MSB_REG, LTC2992_MIN_GPIO3_MSB_REG, LTC2992_MIN_GPIO4_MSB_REG};
static const uint8_t gpio_state_regs[] = {LTC2992_GPIO_STATUS_REG};

// Sense Data Registers
static const uint8_t sense_regs[] = {LTC2992_SENSE1_MSB_REG, LTC2992_SENSE2_MSB_REG};
static const uint8_t sense_max_regs[] = {LTC2992_MAX_SENSE1_MSB_REG, LTC2992_MAX_SENSE2_MSB_REG};
static const uint8_t sense_min_regs[] = {LTC2992_MIN_SENSE1_MSB_REG, LTC2992_MIN_SENSE2_MSB_REG};

// Current Data Registers
static const uint8_t dsense_regs[] = {LTC2992_DELTA_SENSE1_MSB_REG, LTC2992_DELTA_SENSE2_MSB_REG};
static const uint8_t dsense_max_regs[] = {LTC2992_MAX_DELTA1_SENSE_MSB_REG, LTC2992_MAX_DELTA2_SENSE_MSB_REG};
static const uint8_t dsense_min_regs[] = {LTC2992_MAX_DELTA1_SENSE_MSB_REG, LTC2992_MAX_DELTA2_SENSE_MSB_REG};

// Power Data Registers
static const uint8_t power_regs[] = {LTC2992_POWER1_MSB1_REG, LTC2992_POWER2_MSB1_REG};
static const uint8_t power_max_regs[] = {LTC2992_MAX_POWER1_MSB1_REG, LTC2992_MAX_POWER2_MSB1_REG};
static const uint8_t power_min_regs[] = {LTC2992_MIN_POWER1_MSB1_REG, LTC2992_MIN_POWER2_MSB1_REG};

// Functions
int8_t ltc2992_write_confg(int ic_addr, uint16_t c_reg, int c_reg_Num);
float *ltc2992_read_gpio(int ic_addr, const int Pin);
float *ltc2992_read_sense(uint8_t ic_addr, const int senseNum);
float *ltc2992_read_dsense(int ic_addr, const int dsenseNum);
float *ltc2992_read_power(int ic_addr, const int powerNum);
int readSMBUsWord(int i2cbus, int address, int daddress);