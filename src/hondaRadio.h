/* state of what would be displayed on the LCD */
typedef enum {
  LCD_STATE_UNKNOWN,
  LCD_STATE_INCOMPLETE,
  LCD_STATE_POWERED_OFF,
  LCD_STATE_CODE_C,
  LCD_STATE_CODE_CO,
  LCD_STATE_CODE_COD,
  LCD_STATE_CODE_CODE,
  LCD_STATE_ERROR_NUM,
  LCD_STATE_ERROR_E,
  LCD_STATE_RUNNING
}LCD_STATE_T;

/* command headers from radio */
#define LCD_COMMAND_OUTPUT 0x42
#define LCD_COMMAND_INPUT 0xC2

/* recieve buffer length. needs to be longer than a message 
on 7BK0 radio this is 12 bytes */
#define BUFFER_SIZE 20

/* number of buttons available to push in codes */
#define BUTTON_MAX 6
/* length of code in key presses */
#define LENGTH_OF_CODE 5

