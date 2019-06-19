#define CHRISTMAS               4
#define PARTY                   5
#define COLOR_OVERRIDE          6
#define STROBE                  7
#define POLICE                  8
#define RAINBOW                 9
#define UA                      10
#define COLOR_BY_BOX            11

#define LEFT_CAMERA             0
#define RIGHT_CAMERA            1

#define Starting                true
#define Ending                  false

#define overrideEngaged         false
//analog joystick pins
#define JOYSTICK_1_X            A10
#define JOYSTICK_1_Y            A11
#define JOYSTICK_2_X            A13
#define JOYSTICK_2_Y            A12

//pot pins for scaling
#define POT_2                   A1
#define POT_1                   A2

//set the digital io pins
#define ACTUATOR_UP1            29// left yellow
#define ACTUATOR_DOWN1          28 //left blue
#define ACTUATOR_UP2            25 // right yellow
#define ACTUATOR_DOWN2          24 // right blue

#define MACRO_BUTTON_ENABLE1    25  //Right Blue
#define MACRO_BUTTON_ENABLE2    24  //Right Yellow

#define ESTOP                   27 //red

//autonmous button pins
#define MACRO_BUTTON            26 // black
#define AUTO_LIGHT              A14 //was 30

//set the speeds
#define NEUTRAL                 127
#define MIN                     1
#define MAX                     255

//define the states of the robot
#define MANUAL                  0
#define AUTONOMOUS              1

//Macro_commands
#define MACRO_SNIPPIT           4
#define ENCODER_SNIPPIT         6
#define ACTUATOR_SNIPPIT        7
#define WII_SNIPPIT             8

//blinking led pin
#define LEDIN                   13

//SCREEN COMMAND OBJECTS DEFINED
#define FORMMESSAGE             0x0A
#define SLIDER                  0x04
#define KNOB                    0x01
#define BUTTON                  0x06
#define LEDDIGITS               0x0F
#define USERLED                 0x13
#define ANGULARMETER            0x07
#define METER                   0x10
#define LED                     0x0E
#define GUAGE                   0x0B
#define STRINGS                 0x11
#define SCOPE                   0x19
#define KEYBOARD                0x0D
#define CONTRAST                0x04
#define COOLGAUGE               0x08
#define COLORPICKER             0x20
#define FOURDBUTTON             0x1E

//SCREEN FORMS DEFINED AND LABELED FOR SMART UPDATE
#define StartPage      0
#define Battery        1
#define Info           2
#define Manual         3
#define CommError      4
#define Autonomous     5
#define Debug          6
#define PID            7
#define ActuatorMacros 8
#define MacroTester    9
#define ValueLoader    10
#define PIDViewer      11
#define GyroViewer     12
#define EncoderMacros  13

#define MacroPage      15
#define WiiPage        16
#define ServoOverride  18

//KEYPAD DEFINITIONS
#define BACK           98
#define ENTER          101

//Definitions for LED digits
#define LEDSTRIP1      A5
#define LEDSTRIP2      A6

#define VOLTAGE_DIVIDER_RATIO 1.0/0.506
#define BATTERY_LOW_LEVEL     60
#define BATTERY_DETECT_LEVEL  20

