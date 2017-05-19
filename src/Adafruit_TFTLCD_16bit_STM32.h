// IMPORTANT: SEE COMMENTS @ LINE 15 REGARDING SHIELD VS BREAKOUT BOARD USAGE.

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#ifndef _ADAFRUIT_TFTLCD_16BIT_STM32_H_
#define _ADAFRUIT_TFTLCD_16BIT_STM32_H_

    #define PROGMEM
    #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
    #define pgm_read_word(addr) (*(const unsigned short *)(addr))

#include <Adafruit_GFX.h>

#include <libmaple/gpio.h>

//#define USE_MAPLE_MINI_PINOUT // for use with maple mini

/*****************************************************************************/
// LCD controller chip identifiers
#define ID_932X    0
#define ID_7575    1
#define ID_9341    2
#define ID_HX8357D    3
#define ID_UNKNOWN 0xFF

/*****************************************************************************/
#ifndef TFTWIDTH
  #define TFTWIDTH   320
  #define TFTHEIGHT  480
#endif
// Initialization command tables for different LCD controllers
#define TFTLCD_DELAY 0xFF

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

/*****************************************************************************/
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
/*****************************************************************************/
// Define pins and Output Data Registers
/*****************************************************************************/
// Port data bits D0..D16:
#if defined (__STM32F1__)
  #define TFT_DATA_PORT	GPIOD
#elif defined (__STM32F4__)
  #define TFT_DATA_PORT	(&GPIOD)
#endif
#define TFT_DATA_IO_PORT	'D'

//Control pins |RD |WR |RS |CS |RST|
#if defined (__STM32F1__)
  #define TFT_CNTRL_PORT	GPIOC
#elif defined (__STM32F4__)
  #define TFT_CNTRL_PORT	(&GPIOC)
#endif
#define TFT_CNTRL_IO_PORT	'C'
//#define TFT_RD_BIT		5
#define TFT_RST_BIT		6
#define TFT_RS_BIT		7
#define TFT_WR_BIT		8
#define TFT_CS_BIT		9

#define Port2Pin(port, bit) ((port-'A')*16+bit)

//#define TFT_RD_PIN		Port2Pin(TFT_CNTRL_IO_PORT, TFT_RD_BIT)
#define TFT_WR_PIN		Port2Pin(TFT_CNTRL_IO_PORT, TFT_WR_BIT)
#define TFT_RS_PIN		Port2Pin(TFT_CNTRL_IO_PORT, TFT_RS_BIT)
#define TFT_CS_PIN		Port2Pin(TFT_CNTRL_IO_PORT, TFT_CS_BIT)
#define TFT_RST_PIN		Port2Pin(TFT_CNTRL_IO_PORT, TFT_RST_BIT) //PB10

	#define RST_ACTIVE	digitalWrite(TFT_RST_PIN, LOW)
	#define RST_IDLE	digitalWrite(TFT_RST_PIN, HIGH)
#if 0 // used TFT does not support RD operation
	#define RD_ACTIVE    digitalWrite(TFT_RD_PIN, LOW)
	#define RD_IDLE      digitalWrite(TFT_RD_PIN, HIGH)
#endif
#if 0
	// use old definition, standard bit toggling, low speed
	#define WR_ACTIVE    digitalWrite(TFT_WR_PIN, LOW) //
	#define WR_IDLE      digitalWrite(TFT_WR_PIN, HIGH) //
	#define CD_COMMAND   digitalWrite(TFT_RS_PIN, LOW)
	#define CD_DATA      digitalWrite(TFT_RS_PIN, HIGH)
	#define CS_ACTIVE    digitalWrite(TFT_CS_PIN, LOW)
	#define CS_IDLE      digitalWrite(TFT_CS_PIN, HIGH)
	#define CS_ACTIVE_CD_COMMAND	{ CS_ACTIVE; CD_COMMAND; }
#else
	// use fast bit toggling, very fast speed!
extern gpio_reg_map * cntrlRegs;
	#define WR_ACTIVE				gpio_clear_dev_bit(TFT_CNTRL_PORT, TFT_WR_BIT) //digitalWrite(TFT_WR_PIN, LOW)//
	#define WR_IDLE					digitalWrite(TFT_WR_PIN, HIGH)//gpio_set_dev_bit(TFT_CNTRL_PORT, TFT_WR_BIT) //
	#define CD_COMMAND				gpio_clear_dev_bit(TFT_CNTRL_PORT, TFT_RS_BIT) //{ cntrlRegs->BSRRH = BIT(TFT_RS_BIT); } //
	#define CD_DATA					gpio_set_dev_bit(TFT_CNTRL_PORT, TFT_RS_BIT) //{ cntrlRegs->BSRRH = BIT(TFT_RS_BIT); } //
	#define CS_ACTIVE				gpio_clear_dev_bit(TFT_CNTRL_PORT, TFT_CS_BIT) //{ cntrlRegs->BSRRL = BIT(TFT_CS_BIT); } //
	#define CS_IDLE					gpio_set_dev_bit(TFT_CNTRL_PORT, TFT_CS_BIT) //{ cntrlRegs->BSRRH = BIT(TFT_CS_BIT); } //
	#define CS_ACTIVE_CD_COMMAND	{ cntrlRegs->BSRRH = BIT(TFT_CS_BIT)|BIT(TFT_RS_BIT); }
#endif

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

extern gpio_reg_map * dataRegs;

#if 0 // used TFT cannot be read
  extern uint8_t read8_(void);
  #define read8(x) ( x = read8_() )
  #define setReadDir() { for (uint8 i = 0; i<16; i++) { pinMode(Port2Pin(TFT_DATA_IO_PORT,0)+i, OUTPUT); } }	// set the bits as input
#endif // used TFT cannot be read
#define setWriteDir() { for (uint8 i = 0; i<16; i++) { pinMode(Port2Pin(TFT_DATA_IO_PORT,0)+i, OUTPUT); } }	// set the bits as output

// set pins to output the 8 bit value
 #define write8(d)		{ dataRegs->ODR = d&0x00FF; WR_STROBE; }
 #define write16_(d)	{ dataRegs->ODR = d;}
 #define write16(d)		{ write16_(d); WR_STROBE; }

/*****************************************************************************/

#define swap(a, b) { int16_t t = a; a = b; b = t; }

/*****************************************************************************/
class Adafruit_TFTLCD_16bit_STM32 : public Adafruit_GFX {

 public:

  //Adafruit_TFTLCD_8bit_STM32(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t rst);
  Adafruit_TFTLCD_16bit_STM32(void);

  void     begin(uint16_t id = 0x9341);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  void     drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void     fillScreen(uint16_t color);
  void     reset(void);
  void     setRegisters8(uint8_t *ptr, uint8_t n);
  void     setRegisters16(uint16_t *ptr, uint8_t n);
  void     setRotation(uint8_t x);
       // These methods are public in order for BMP examples to work:
  void     setAddrWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
  void     invertDisplay(boolean i),
			pushColors(uint16_t *data, int16_t len, boolean first),
           drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t * bitmap);
#if 0 // used TFT cannot be read
  uint16_t readPixel(int16_t x, int16_t y),
           readID(void);
#endif
 private:

  void     init(),
           flood(uint16_t color, uint32_t len);
  uint8_t  driver;
};

extern uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
#if 0 // used TFT cannot be read
extern uint16_t readReg(uint8_t r);
extern uint32_t readReg32(uint8_t r);
#endif

static inline void writeCommand(uint16_t c) __attribute__((always_inline));
static inline void writeCommand(uint16_t c) {
	CS_ACTIVE_CD_COMMAND;
	write16(c);
}

extern void writeRegister8(uint16_t a, uint8_t d);
extern void writeRegister16(uint16_t a, uint16_t d);
//extern void writeRegister24(uint16_t a, uint32_t d);
extern void writeRegister32(uint16_t a, uint32_t d);
//extern void writeRegister32(uint16_t a, uint16_t d1, uint16_t d2);
extern void writeRegisterPair(uint16_t aH, uint16_t aL, uint16_t d);


#endif
