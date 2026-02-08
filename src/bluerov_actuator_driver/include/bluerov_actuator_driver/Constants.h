#ifndef CSE_ACTUATOR_DRIVER__CONSTANTS_H_
#define CSE_ACTUATOR_DRIVER__CONSTANTS_H_

// namespace cse_actuator_driver
// {
// Registers/etc:
// constexpr uint8_t MODE1              = 0x00;
// constexpr uint8_t MODE2              = 0x01;
// constexpr uint8_t SUBADR1            = 0x02;
// constexpr uint8_t SUBADR2            = 0x03;
// constexpr uint8_t SUBADR3            = 0x04;
// constexpr uint8_t PRESCALE           = 0xFE;
// constexpr uint8_t LED0_ON_L          = 0x06;
// constexpr uint8_t LED0_ON_H          = 0x07;
// constexpr uint8_t LED0_OFF_L         = 0x08;
// constexpr uint8_t LED0_OFF_H         = 0x09;
// constexpr uint8_t ALL_LED_ON_L       = 0xFA;
// constexpr uint8_t ALL_LED_ON_H       = 0xFB;
// constexpr uint8_t ALL_LED_OFF_L      = 0xFC;
// constexpr uint8_t ALL_LED_OFF_H      = 0xFD;

// // Bits:
// constexpr uint8_t RESTART            = 0x80;
// constexpr uint8_t SLEEP              = 0x10;
// constexpr uint8_t ALLCALL            = 0x01;
// constexpr uint8_t INVRT              = 0x10;
// constexpr uint8_t OUTDRV             = 0x04;

// constexpr uint16_t MODE1 = 0x00;
// constexpr uint16_t MODE2 = 0x01;
// constexpr uint16_t SUBADR1 = 0x02;
// constexpr uint16_t SUBADR2 = 0x03;
// constexpr uint16_t OUTDRV = 0x04;        /**< I2C-bus subaddress 3 */
// constexpr uint16_t ALLCALL = 0x05; /**< LED All Call I2C-bus address */
// constexpr uint16_t LED0_ON_L = 0x06;          /**< LED0 on tick, low byte*/
// constexpr uint16_t LED0_ON_H = 0x07;         /**< LED0 on tick, high byte*/
// constexpr uint16_t LED0_OFF_L = 0x08;         /**< LED0 off tick, low byte */
// constexpr uint16_t LED0_OFF_H = 0x09;         /**< LED0 off tick, high byte
// */
// // etc all 16:  LED15_OFF_H 0x45
// constexpr uint16_t ALL_LED_ON_L = 0xFA;         /**< load all the LEDn_ON
// registers, low */ constexpr uint16_t ALL_LED_ON_H  = 0xFB;         /**< load
// all the LEDn_ON registers, high */ constexpr uint16_t ALL_LED_OFF_L = 0xFC;
// /**< load all the LEDn_OFF registers, low */ constexpr uint16_t ALL_LED_OFF_H
// = 0xFD;       /**< load all the LEDn_OFF registers,high */ constexpr uint16_t
// PRESCALE = 0xFE;             /**< Prescaler for PWM output frequency */
// constexpr uint16_t PCA9685_TESTMODE = 0xFF;     /**< defines the test mode to
// be entered */

// // MODE1 bits
// constexpr uint16_t MODE1_ALLCAL = 0x01;  /**< respond to LED All Call I2C-bus
// address */ constexpr uint16_t MODE1_SUB3  = 0x02;    /**< respond to I2C-bus
// subaddress 3 */ constexpr uint16_t MODE1_SUB2 = 0x04;    /**< respond to
// I2C-bus subaddress 2 */ constexpr uint16_t MODE1_SUB1 = 0x08;    /**< respond
// to I2C-bus subaddress 1 */ constexpr uint16_t SLEEP = 0x10;         /**< Low
// power mode. Oscillator off */ constexpr uint16_t MODE1_AI = 0x20;      /**<
// Auto-Increment enabled */ constexpr uint16_t MODE1_EXTCLK = 0x40;  /**< Use
// EXTCLK pin clock */ constexpr uint16_t RESTART = 0x80;       /**< Restart
// enabled */
// // MODE2 bits
// constexpr uint16_t MODE2_OUTNE_0 = 0x01; /**< Active LOW output enable input
// */ constexpr uint16_t MODE2_OUTNE_1 = 0x02;                  /**< Active LOW
// output enable input - high impedience */ constexpr uint16_t MODE2_OUTDRV =
// 0x04l /**< totem pole structure vs open-drain */ constexpr uint16_t MODE2_OCH
// = 0x08;    /**< Outputs change on ACK vs STOP */ constexpr uint16_t
// MODE2_INVRT = 0x10;  /**< Output logic state inverted */

// constexpr uint16_t PCA9685_I2C_ADDRESS = 0x40;      /**< Default PCA9685 I2C
// Slave Address */ constexpr uint16_t FREQUENCY_OSCILLATOR = 25000000; /**<
// Int. osc. frequency in datasheet */

// constexpr uint16_t PCA9685_PRESCALE_MIN = 3;   /**< minimum prescale value */
// constexpr uint16_t PCA9685_PRESCALE_MAX = 255; /**< maximum prescale value */

#define MODE1 0x00      /**< Mode Register 1 */
#define MODE2 0x01      /**< Mode Register 2 */
#define SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define OUTDRV 0x04     /**< I2C-bus subaddress 3 */
#define ALLCALL 0x05    /**< LED All Call I2C-bus address */
#define LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define ALL_LED_ON_L 0xFA     /**< load all the LEDn_ON registers, low */
#define ALL_LED_ON_H 0xFB     /**< load all the LEDn_ON registers, high */
#define ALL_LED_OFF_L 0xFC    /**< load all the LEDn_OFF registers, low */
#define ALL_LED_OFF_H 0xFD    /**< load all the LEDn_OFF registers,high */
#define PRESCALE 0xFE         /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01 /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02   /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04   /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08   /**< respond to I2C-bus subaddress 1 */
#define SLEEP 0x10        /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20     /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40 /**< Use EXTCLK pin clock */
#define RESTART 0x80      /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
// #define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */
#define FREQUENCY_OSCILLATOR 24550000 /**< Int. osc. frequency required to initialize thrusters at 1500 us */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

// } // namespace cse_actuator_driver
#endif // CSE_ACTUATOR_DRIVER__CONSTANTS_H_
