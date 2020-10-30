#include <Arduino.h>
#include "hondaRadio.h"

#define PIN_LCD_DI   51 /* MOSI output from radio */
#define PIN_LCD_DO 50 /* MISO input to radio */
#define PIN_LCD_CLK 52 /* spi clock */
#define PIN_LCD_CE 21 /* command select / chip enable */
#define PIN_PWD_SW 22 /* power switch, active low */

#define PIN_SPI_SELECT_OUT 53 /* manually toggle CE pin */

#define PIN_DEBUG_YELLOW 24
#define PIN_DEBUG_GREEN 31

#define DEBUG_GREEN_HIGH PORTC |= 0x40
#define DEBUG_GREEN_LOW PORTC &= ~0x40

#define DEBUG_YELLOW_HIGH PORTA |= 0x04
#define DEBUG_YELLOW_LOW PORTA &= ~0x04

#define BUTTON_MAX 6
#define LENGTH_OF_CODE 5

uint8_t code[LENGTH_OF_CODE] = {3, 4, 2, 5, 6};

#define BUFFER_SIZE 20
volatile uint8_t inData[BUFFER_SIZE];
volatile uint8_t inBytePos = 0;
volatile bool commandComplete;
volatile bool inputCmd;
uint8_t currentButton;

uint8_t txCmd[][5] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },/* fake data to put button 1 at offset 1 */
  { 0x90, 0x00, 0x00, 0x00, 0x00 },
  { 0x88, 0x00, 0x00, 0x00, 0x00 },
  { 0x84, 0x00, 0x00, 0x00, 0x00 },
  { 0x80, 0x00, 0x02, 0x00, 0x00 },
  { 0x80, 0x00, 0x01, 0x00, 0x00 },
  { 0x80, 0x00, 0x00, 0x80, 0x00 },
};


void ISR_endCommand( void )
{
  /* reset our internal spi so that the bytes are synced up */
  digitalWrite(PIN_SPI_SELECT_OUT, 1);
  digitalWrite(PIN_SPI_SELECT_OUT, 0);
  commandComplete = true;
  inputCmd = true;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT); /* onboard led */
  digitalWrite(LED_BUILTIN, 0);
  pinMode(PIN_LCD_DI, INPUT);
  pinMode(PIN_LCD_DO, OUTPUT);
  pinMode(PIN_LCD_CLK, INPUT);
  pinMode(PIN_LCD_CE, INPUT);
  pinMode(PIN_SPI_SELECT_OUT, OUTPUT);
  pinMode(PIN_PWD_SW, OUTPUT);
  pinMode(PIN_DEBUG_YELLOW, OUTPUT);
  pinMode(PIN_DEBUG_GREEN, OUTPUT);

  /* turn off power switch */
  digitalWrite(PIN_PWD_SW, 1);

  attachInterrupt(digitalPinToInterrupt(PIN_LCD_CE), ISR_endCommand, FALLING);

  SPDR = 0xff;

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);
}

ISR (SPI_STC_vect)
{
  if ( inBytePos < BUFFER_SIZE ) {
    inData[inBytePos] = SPDR;

    if ( inData[0] == LCD_COMMAND_INPUT ) {
      SPDR = txCmd[code[currentButton]][0];
      while ( (SPSR & 0x80 ) == 0 );
      SPDR = txCmd[code[currentButton]][1];
      while ( (SPSR & 0x80 ) == 0 );
      SPDR = txCmd[code[currentButton]][2];
      while ( (SPSR & 0x80 ) == 0 );
      SPDR = txCmd[code[currentButton]][3];
      while ( (SPSR & 0x80 ) == 0 );
      SPDR = 0xff;
    }
    /*
        if ( inputCmd == true ){
          SPDR = txCmd[code[currentButton]][inBytePos];

          if ( inData[0] == LCD_COMMAND_OUTPUT ){
            inputCmd = false;
          }
        }
    */
    else {
      SPDR = 0xff;
      inBytePos++;
    }
  }
  else {
    Serial.println("ERROR buffer overflow");
  }

}




LCD_STATE_T commandParse()
{
  LCD_STATE_T ret = LCD_STATE_UNKNOWN;
  static bool poweredOn;

  if ( inData[0] == LCD_COMMAND_OUTPUT && commandComplete == true ) {
#ifdef PRINT_COMMANDS
    uint8_t i;
    for ( i = 0 ; i < inBytePos; i ++ ) {
      Serial.print(inData[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
#endif

    if (inData[12] == 0x02 ) {
      if ( inData[1] == 0x00 ) {
        Serial.println("debug: got POWERED_OFF");
        poweredOn = false;
        ret = LCD_STATE_POWERED_OFF;
      }
      else if ( inData[1] == 0x10 ) {
        poweredOn = true;
      }
      else if ( inData[1] == 0x13 ) {
        ret = LCD_STATE_RUNNING;
      }
    }
    else if ( inData[12] == 0x04 && poweredOn == true ) {

      if ( inData[7] == 0x77 ) {
        ret = LCD_STATE_CODE_CODE;
        Serial.println("debug: got CODE");
      }
      else if ( inData[7] == 0x00
                && inData[10] == 0x00 ) {
        ret = LCD_STATE_CODE_C;
        //Serial.println("debug: got C___");
      }
      else if ( inData[7] == 0x00
                && inData[9] == 0x00
                && inData[10] == 0x1c ) {
        ret = LCD_STATE_CODE_CO;
        //Serial.println("debug: got CO__");
      }
      else if ( inData[7] == 0x60 ) {
        ret = LCD_STATE_CODE_COD;
        //Serial.println("debug: got COD_");
      }
      else if ( inData[9] == 0x0C
                && inData[10] == 0x24 ) {
        ret = LCD_STATE_ERROR_E;
        Serial.print(".");
      }
      else {
        ret = LCD_STATE_ERROR_NUM;
        Serial.println("debug: got err #");
      }
    }
  }
  return ret;
}

void press( void ) {
  SPDR = 0x00;
}

bool incrementCode ( void )
{
  bool ret = true;
  int i = 0;
  int carry = 1;
  while ( carry == 1 && i < 6 ) {
    code[i]++;
    if (code[i] < 7 ) {
      carry = 0;
    }
    else {
      code[i] = 1;
      i++;
    }
  }

  if ( i == 6) {
    ret = false;
  }

  Serial.print("next code: ");
  for ( i = 0; i < LENGTH_OF_CODE; i ++ ) {
    Serial.print(code[i]);
  }
  Serial.println();

  return ret;
}

#define BLINK_RATE_MS 250

void ledOutput ( void )
{
  static uint16_t currentNum, currentBlink;
  static unsigned long timeToTransition;

  if (millis() > timeToTransition ) {
    currentBlink++;
    if ( currentBlink == (code[currentNum] * 2) ) {
      currentBlink = 0;
      currentNum++;
      if ( currentNum == LENGTH_OF_CODE ) {
        currentNum = 0;
        timeToTransition = millis() + ( BLINK_RATE_MS * 6);
      }
      else {
        timeToTransition = millis() + ( BLINK_RATE_MS * 3 );
      }
    }
    else {
      timeToTransition = millis() + BLINK_RATE_MS;
    }

    if ( currentBlink & 1 ) {
      PORTB |= 0x80;
    }
    else {
      PORTB &= ~0x80;
    }
  }
}

void loop() {
  static bool CODEatStart;
  static bool seenAlready;
  static bool keepRunning = true;
  static bool printResult = false;

  if ( printResult == true ) {
    ledOutput();
  }

  if ( commandComplete == true && keepRunning == true ) {
    LCD_STATE_T incomingCmd = commandParse();
    if ( incomingCmd == LCD_STATE_POWERED_OFF ) {
      CODEatStart = true;
      Serial.println("powered off, powering on, look for CODE");

      digitalWrite(PIN_PWD_SW, 0);

    }
    else if ( incomingCmd == LCD_STATE_CODE_CODE
              && CODEatStart ) {
      /*let go of power button if we were pushing it down */
      digitalWrite(PIN_PWD_SW, 1);
      if ( seenAlready ) {
        CODEatStart = false;
        /*transmit first button */
        Serial.print("trying ");
        Serial.print(code[currentButton]);
        press();
        seenAlready = false;
      }
      else {
        seenAlready = 1;
      }
    }
    else if ( incomingCmd == LCD_STATE_CODE_C
              || incomingCmd == LCD_STATE_CODE_CO
              || incomingCmd == LCD_STATE_CODE_COD
              || ( incomingCmd == LCD_STATE_CODE_CODE && CODEatStart == false) ) {
      if ( seenAlready ) {
        currentButton++;
        Serial.print(code[currentButton]);
        press();
        seenAlready = false;
      }
      else {
        seenAlready = true;
      }
    }
    else if ( incomingCmd == LCD_STATE_ERROR_E ) {
      if ( CODEatStart == false ) {
        CODEatStart = true;
        keepRunning = incrementCode();
        seenAlready = false;
        currentButton = 0;
      }
    }
    else if ( incomingCmd == LCD_STATE_ERROR_NUM ) {
      if ( seenAlready == true ) {
        seenAlready = false;
        keepRunning = incrementCode();
        if ( keepRunning ) {
          currentButton = 0;
          Serial.println();
          Serial.print(code[currentButton]);
          press();
        }
      }
      else {
        seenAlready = true;
      }
    }
    else if ( incomingCmd == LCD_STATE_RUNNING ) {
      keepRunning = false;
      printResult = true;
      Serial.println("found Code");
    }
    inBytePos = 0;
    commandComplete = false;
  }

  if ( keepRunning == false) {
    SPCR = 0x00;
  }
}

