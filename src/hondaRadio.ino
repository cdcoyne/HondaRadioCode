#include <Arduino.h>
#include "hondaRadio.h"

/* serial log display states */
//#define LOG_RX_STATES
//#define LOG_TX_STATES
/* serial log incoming data */
//#define LOG_RX_DATA

#define PIN_LCD_DI   51 /* MOSI output from radio */
#define PIN_LCD_DO 50 /* MISO input to radio */
#define PIN_LCD_CLK 52 /* spi clock */
#define PIN_LCD_CE 21 /* command select / chip enable */
#define PIN_PWD_SW 22 /* power switch, active low */

#define PIN_SPI_SELECT_OUT 53 /* manually toggle CE pin */


/* current code to be attempted, each element is a keypress */
uint8_t code[LENGTH_OF_CODE] = {5, 4, 2, 5, 4};
uint8_t currentButton;

volatile uint8_t rxData[BUFFER_SIZE];
volatile uint8_t rxDataPtr = 0;
volatile bool commandComplete;

uint8_t txCmd[][5] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },/* fake data to put button 1 at offset 1 */
  { 0x90, 0x00, 0x00, 0x00, 0x00 },
  { 0x88, 0x00, 0x00, 0x00, 0x00 },
  { 0x84, 0x00, 0x00, 0x00, 0x00 },
  { 0x80, 0x00, 0x02, 0x00, 0x00 },
  { 0x80, 0x00, 0x01, 0x00, 0x00 },
  { 0x80, 0x00, 0x00, 0x80, 0x00 },
};


  /* manipulate the SPI enable line ourself since 
    the actual Honda hardware toggles the CE line after the first byte
    which it uses for a command type. If we used it with the 
    spi peripheral it would miss the first byte. 
    we still use it since it keeps the data in sync with the 
    spi clock  */
void ISR_endCommand( void )
{
  digitalWrite(PIN_SPI_SELECT_OUT, 1);
  digitalWrite(PIN_SPI_SELECT_OUT, 0);
  commandComplete = true;
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

  /* turn off power switch */
  digitalWrite(PIN_PWD_SW, 1);

  attachInterrupt(digitalPinToInterrupt(PIN_LCD_CE), ISR_endCommand, FALLING);

/* initialize spi bus.  
  lowering the data line prompts the master to send the 
  "read key press" command instead of the "update display" */
  SPDR = 0xff;
  // enable SPI in slave mode
  SPCR |= _BV(SPE);
  // enable interrupts
  SPCR |= _BV(SPIE);
}

/* SPI ISR
on the mega2560 the datarate was too fast to exit the interrupt after 
each byte, so just sit in here and handle the 
entire thing, even using a loop to send data was too slow in testing */
ISR (SPI_STC_vect)
{
  if ( rxDataPtr < BUFFER_SIZE ) {
    rxData[rxDataPtr] = SPDR;

  /* send key press if cpu is expecting it */
    if ( rxData[0] == LCD_COMMAND_INPUT ) {
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

    /* otherwise command is just for updating LCD, ignore it */
    else {
      SPDR = 0xff;
      rxDataPtr++;
    }
  }
  else {
    Serial.println("ERROR buffer overflow");
  }

}




LCD_STATE_T commandParse()
{
  static LCD_STATE_T lastCommand = LCD_STATE_UNKNOWN;
  LCD_STATE_T thisCommand = LCD_STATE_UNKNOWN;
  LCD_STATE_T ret = LCD_STATE_UNKNOWN;
  static bool poweredOn;

  if ( rxData[0] == LCD_COMMAND_OUTPUT ) {
#ifdef LOG_RX_DATA
    uint8_t i;
    for ( i = 0 ; i < rxDataPtr; i ++ ) {
      Serial.print(rxData[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
#endif

  /* locations and data taken from looking at all the commands
  and finding the minimal set of unique data to identify what
  the display would be showing. 
  The data is to the LCD is sent over several indexed commands 
  and this just cherry picks some unique locations to determine 
  what the display state is.
  */
    if (rxData[12] == 0x02 ) {
      if ( rxData[1] == 0x00 ) {
#ifdef LOG_RX_STATES
        Serial.println("RX POWERED_OFF");
#endif
        poweredOn = false;
        thisCommand = LCD_STATE_POWERED_OFF;
      }
      else if ( rxData[1] == 0x10 ) {
        /* once powered on this matches any command index 2 */
        poweredOn = true;
      }
      else if ( rxData[1] == 0x13 ) {
        /* code was accepted, running screen */
        thisCommand = LCD_STATE_RUNNING;
      }
    }
    else if ( rxData[12] == 0x04 && poweredOn == true ) {

      if ( rxData[7] == 0x77 ) {
        thisCommand = LCD_STATE_CODE_CODE;
#ifdef LOG_RX_STATES
        Serial.println("RX CODE");
#endif
      }
      else if ( rxData[7] == 0x00
                && rxData[10] == 0x00 ) {
        thisCommand = LCD_STATE_CODE_C;
#ifdef LOG_RX_STATES
        Serial.println("RX C___");
#endif
      }
      else if ( rxData[7] == 0x00
                && rxData[9] == 0x00
                && rxData[10] == 0x1c ) {
        thisCommand = LCD_STATE_CODE_CO;
#ifdef LOG_RX_STATES
        Serial.println("RX CO__");
#endif
      }
      else if ( rxData[7] == 0x60 ) {
        thisCommand = LCD_STATE_CODE_COD;
#ifdef LOG_RX_STATES       
        Serial.println("RX COD_");
#endif
      }
      else if ( rxData[10] == 0x24 ) {
        if ( rxData[9] == 0x0C ) {
          thisCommand = LCD_STATE_ERROR_E;
        }
        else {
          thisCommand = LCD_STATE_ERROR_NUM;
#ifdef LOG_RX_STATES
          Serial.println("RX err #");
#endif
        }
      }
    }
  }

  /* retain last command to reference to make sure the
  display is stable, pressing a button in the middle
  of a display update may cause stale dipslay data */
  if ( thisCommand == lastCommand ){
    ret = thisCommand;
  }
  else if ( thisCommand != LCD_STATE_UNKNOWN ){
    lastCommand = thisCommand;
    ret = LCD_STATE_UNKNOWN;
  }
  else {
    ret = LCD_STATE_UNKNOWN;
  }

  return ret;
}

/* mimic button press by lowering the SPI data line 
which tells the master that there is a button pressed down. */
void pressButton( void ) {
  SPDR = 0x00;
}

/* increment through possible radio codes placing the 
result in the 'code' array */

bool incrementCode ( void )
{
  bool ret = true;
  int i = LENGTH_OF_CODE-1;
  int carry = 1;
  while ( ret == true && carry == 1 ) {
    code[i]++;
    if (code[i] <= BUTTON_MAX ) {
      carry = 0;
    }
    else {
        code[i] = 1;
        if ( i > 0 ){
            i--;
        }
        else{
            ret = false;
        }

    }
  }

  if ( ret == true ){
    Serial.print("next code: ");
    for ( i = 0; i < LENGTH_OF_CODE; i ++ ) {
      Serial.print(code[i]);
    }
    Serial.println("");
  }

  return ret;
}

/* blink the onboard led indicating that the code was found 
code is blinked out with a pause between each button, then 
a long pause when it repeats */
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
        timeToTransition = millis() + ( LED_BLINK_RATE_MS * 6);
      }
      else {
        timeToTransition = millis() + ( LED_BLINK_RATE_MS * 3 );
      }
    }
    else {
      timeToTransition = millis() + LED_BLINK_RATE_MS;
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
  static bool running = true;

  if ( running == false ) {
    ledOutput();
    /* turn off spi when complete */
    SPCR = 0x00;
  }
  else if ( commandComplete == true ) {
    LCD_STATE_T incomingCmd = commandParse();
    if ( incomingCmd == LCD_STATE_POWERED_OFF ) {
      CODEatStart = true;
      digitalWrite(PIN_PWD_SW, 0);

    }
    else if ( incomingCmd == LCD_STATE_CODE_CODE
              && CODEatStart ) {
      /*let go of power button if we were pushing it down */
      digitalWrite(PIN_PWD_SW, 1);
      CODEatStart = false;
      /*transmit first button */
#ifdef LOG_TX_STATES
      Serial.print("TX ");
      Serial.println(code[currentButton]);
#endif
      pressButton();
    }
    else if ( incomingCmd == LCD_STATE_CODE_C
              || incomingCmd == LCD_STATE_CODE_CO
              || incomingCmd == LCD_STATE_CODE_COD
              || ( incomingCmd == LCD_STATE_CODE_CODE && CODEatStart == false) ) {
      currentButton++;
#ifdef LOG_TX_STATES
      Serial.print("TX ");
      Serial.println(code[currentButton]);
#endif
      pressButton();
    }
    else if ( incomingCmd == LCD_STATE_ERROR_E ) {
      if ( CODEatStart == false ) {
        CODEatStart = true;
        running = incrementCode();
        currentButton = 0;
      }
    }
    else if ( incomingCmd == LCD_STATE_ERROR_NUM ) {
      running = incrementCode();
      if ( running ) {
        currentButton = 0;
#ifdef LOG_TX_STATES
        Serial.print("TX ");
        Serial.println(code[currentButton]);
#endif
        pressButton();
      }
    }
    else if ( incomingCmd == LCD_STATE_RUNNING ) {
      running = false;
      Serial.println("found Code");
    }
    rxDataPtr = 0;
    commandComplete = false;
  }

}

