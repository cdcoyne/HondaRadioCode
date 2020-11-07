#include <Arduino.h>
#include "hondaRadio.h"

/* serial log display states */
//#define LOG_RX_STATES
//#define LOG_TX_STATES
/* serial log incoming data */
//#define LOG_RX_DATA


/* current code to be attempted, each element is a keypress */
uint8_t code[LENGTH_OF_CODE] = {5, 4, 2, 5, 4};
uint8_t currentButton;

volatile uint8_t rxData[BUFFER_SIZE];
volatile uint8_t rxDataPtr = 0;
volatile bool commandComplete;

/* output commands to mimic button presses */
uint8_t txCmd[][5] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },/* fake data to put button 1 at offset 1 */
  { 0x90, 0x00, 0x00, 0x00, 0x00 },/* press 1 */
  { 0x88, 0x00, 0x00, 0x00, 0x00 },/* press 2 */
  { 0x84, 0x00, 0x00, 0x00, 0x00 },/* press 3 */
  { 0x80, 0x00, 0x02, 0x00, 0x00 },/* press 4 */
  { 0x80, 0x00, 0x01, 0x00, 0x00 },/* press 5 */
  { 0x80, 0x00, 0x00, 0x80, 0x00 },/* press 6 */
};


/* manipulate the SPI enable line ourself since 
  the actual Honda hardware toggles the CE line after the first byte
  which it uses for a command type. If we used it with the 
  spi peripheral it would miss the first byte. 
  we still use it since it keeps the data in sync with the 
  spi clock  */
void ISR_endCommand( void )
{
  digitalWrite(PIN_SPI_CE_INTERNAL, 1);
  digitalWrite(PIN_SPI_CE_INTERNAL, 0);
  commandComplete = true;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(PIN_LED_FEEDBACK, OUTPUT); 
  digitalWrite(PIN_LED_FEEDBACK, 0);

  pinMode(PIN_LCD_DI, INPUT);
  pinMode(PIN_LCD_DO, OUTPUT);
  pinMode(PIN_LCD_CLK, INPUT);
  pinMode(PIN_LCD_CE, INPUT);
  pinMode(PIN_SPI_CE_INTERNAL, OUTPUT);

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
      SPDR = txCmd[code[currentButton]][4];;
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


/* parse incoming data to determine the state of the LCD 
and return the state */
LCD_STATE_T getLCDCommand()
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
  of a display update may cause stale display data */
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
#ifdef LOG_TX_STATES
        Serial.print("TX ");
        Serial.println(code[currentButton]);
#endif
  SPDR = 0x00;
}

/* increment through possible radio codes placing the 
result in the 'code' array return false if no more codes */

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

/* blink led indicating that the code was found 
code is blinked out with a pause between each button, then 
a long pause when it repeats */
void ledOutputCode ( void )
{
  static uint16_t currentNum = 0, currentBlink = 0;
  static unsigned long timeToTransition = 0;

  if (millis() > timeToTransition ) {
    currentBlink++;
    if ( currentBlink == (code[currentNum] * 2) ) {
      currentBlink = 0;
      currentNum++;
      /* reset sequence, long pause */
      if ( currentNum == LENGTH_OF_CODE ) {
        currentNum = 0;
        timeToTransition = millis() + ( LED_BLINK_RATE_MS * 6);
      }
      /* pause between numbers */
      else {
        timeToTransition = millis() + ( LED_BLINK_RATE_MS * 3 );
      }
    }
    else {
      timeToTransition = millis() + LED_BLINK_RATE_MS;
    }

    digitalWrite(PIN_LED_FEEDBACK,currentBlink & 1);
  }
}

/* press buttons to input codes using the state of the 
LCD to know how fast to push buttons and if the code worked 
returns true as long as it hasnt reached an end state, 
which is either the code worked or you run out of codes 
parameter is the state of the LCD */
bool updateStateMachine( LCD_STATE_T incomingCmd )
{
    bool keepRunning;
  /* CODE is shown on the screen both after you press the 4th button,
  and after initially after power on or after the timeout resets 
  from the ERROR E state so need to differentiate between them */
    static bool CODEatStart; 

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
      pressButton();
    }
    else if ( incomingCmd == LCD_STATE_CODE_C
              || incomingCmd == LCD_STATE_CODE_CO
              || incomingCmd == LCD_STATE_CODE_COD
              || ( incomingCmd == LCD_STATE_CODE_CODE && CODEatStart == false) ) {
      currentButton++;
      pressButton();
    }
    else if ( incomingCmd == LCD_STATE_ERROR_E ) {
      /* heartbeat blink LED */
      digitalWrite(PIN_LED_FEEDBACK,!digitalRead(PIN_LED_FEEDBACK));
      if ( CODEatStart == false ) {
        CODEatStart = true;
        keepRunning = incrementCode();
        currentButton = 0;
      }
    }
    else if ( incomingCmd == LCD_STATE_ERROR_NUM ) {
      keepRunning = incrementCode();
      if ( keepRunning ) {
        currentButton = 0;

        pressButton();
      }
    }
    else if ( incomingCmd == LCD_STATE_RUNNING ) {
      keepRunning = false;
      Serial.println("found Code");
    }

    return keepRunning;
}

void loop() {
  static bool running = true;

  if ( running == false ) {
    ledOutputCode();
    /* turn off spi when complete */
    SPCR = 0x00;
  }
  else if ( commandComplete == true ) {
    LCD_STATE_T incomingCmd = getLCDCommand();
    running = updateStateMachine(incomingCmd);
    rxDataPtr = 0;
    commandComplete = false;  

  }
}

