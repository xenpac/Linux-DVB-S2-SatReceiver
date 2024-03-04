this is the cypress dongle software for the DVBworld mpeg TV tuner , my board.
This is a Keil uvision project, so you will need the ctools.rar to compile.

there seem to be a problem when NOT board is powered and then connect usb.
so first power the board, then connect usb.
we need to power the cypress board from the tuner supply!!

I disconnected the 5V from USB-connector LCsoft-board (remove a bridge-resistor) and connect the 5V from
the powersupply to the input of the LCsoft-board voltage regulator.
Now you have a self-powered device ;)

the original design requires the cypress board to identify itself as:
#define VENDOR 	0x04b4
#define PRODUCT	0x2104   //dvb world usb device

therefore you have to flash the firmwarefile "fx2lp.hex" to a blank fx2lp board once using
the cycontrol.exe. install usb driver for this.

copy the files "dvb-fe-cx24116.fw" and "dvb-usb-dw2104.fw" to linux under /lib/firmware .

have fun ;)