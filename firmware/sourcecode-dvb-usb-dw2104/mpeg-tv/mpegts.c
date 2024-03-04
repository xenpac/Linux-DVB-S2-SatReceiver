/* MPEG-TS transport stream over USB to PC.
i2c support for the CX24116 in **Sharp BSBE2-401A NIM Tuner**
!!! This is the firmware for linux to download as DVBworld usb device. "dvb-usb-dw2104.fw"
device simulated: DVBWorld2104C model

Linux driver dw2102.c config: 
.devices ="DVBWorld DW2104 USB2.0"
.i2c_algo = &dw2104_i2c_algo,
.generic_bulk_ctrl_endpoint = 0x81,
.download_firmware = dw2102_load_firmware,
.read_mac_address = dw210x_read_mac_address,
.frontend_attach = dw2104_frontend_attach,
.stream = {.type = USB_BULK,.count = 8,.endpoint = 0x82,.buffersize = 4096,}

command interface = EP0
 NOTE: if enabled in the kernel sourcecode, EP1 can be used for vendorcommands instaed of EP0 !
per default: EP1 is not used!! 

stream interface = 0x82 = EP2 slave fifo


+++ CY7C68013A ENDPOINTS:
EP0 = always control endpoint. This is used for command interface, i2c transfers and eeprom read...64Bytes
EP2 =  unidirectional IN endpoint for mpegts streaming, slave fifo

cx24116 TunerStatus Register 0x9D:
bit:
0 = Tuner PLL Lock Indicator, FE_HAS_SIGNAL, found something above the noise level 
1 = Demodulator Sync Indicator, FE_HAS_CARRIER, found a DVB signal
2 = Viterbi or LDPC Sync Indicator, FE_HAS_VITERBI, FEC is stable
3 = Reed-Solomon or BCH Sync Indicator, FE_HAS_SYNC, found sync bytes, everything's working...
4 = reserved
5 = Low-Priority Sync Indicator
6 = Front-End AGC Accumulator
7 = Front-End AGC Accumulator

+++ Bugs >>
-- DVB Tuner, ir-receiver inside dvb-usb not working, it sends spurious keypresses:
solution:
-> create file /etc/modprobe.d/dvb-usb.conf, containing: options dvb-usb disable_rc_polling=1
   then on probe, driver does not load ir support.
   
there migt be a problem when banana-pi reboots and usbdev stays active.
- usb reconnect scenarios testing:
press fx2lp powerbutton = works
unplug/replug usb-cable = works
reboot Pi while fx2lp stays on = ?
powercycle Pi and fx2lp at same time = ?
powercycle just fx2lp = 
powercycle just Pi = 
   

-- ubuntu-laptop: on device disconnect, the dvb-usb function dw2102_disconnect() causes a kernel Ooops: (dw2102 kernel oops)
 usb 1-2: USB disconnect, device number 5
Nov 10 14:50:47 LinuxLaptop kernel: BUG: unable to handle kernel NULL pointer dereference at 0000000000000008
Nov 10 14:50:47 LinuxLaptop kernel: IP: [<ffffffffc09de1bd>] dw2102_disconnect+0x1d/0x50 [dvb_usb_dw2102]
-> no solution. connect works. disconnect = kern oops/reboot.

+++ EndBugs<<

-- enable linux kernel debugging messages (dmesg):
modprobe dvb-core dvbdev_debug=1 
echo "options dvb-usb-dw2102 demod=2" > /storage/.config/modprobe.d/dw2104.conf
linux usb errors encountered:
 -32 = broken pipe, fx2lp returned 0 causing stall
 -71 = protocol error, this should be used
 -19 = no such device. its disconnected
 -110 = Connection timed out.Timeout expired before the transfer completed

-- firmware:
There are 5 versions of DVB-USB-DW2104.fw:
1. DVB-S2 USB 2104A (EARDA4B47, STB6100 + ST0903)
2. DVB-S2 USB 2104B (sharp0169, stv6110A + ST0903)
3. DVB-S2 USB 2104C (cx24116)
4. DVB-S2 USB 2104D (montage_ts2020 + ds3000)
5. DVB-S2 USB 2104E (SERIT2636, stv6110A + ST0903)

-- fx2-board pin assignment:
PORTA alternate Pin configurations is activated when  IFCONFIG = 0x03; enabling slave fifo mode:
PIN   CONNECT  ALTFUNCTION
==========================
PA0 = LED1 (normal port)
PA1 = LED2 (normal port)

//the following PA alt functions are all INPUTS!
PA2 = +3.3v = SLOE = Slave Output Enable act-0. Sets the DataPort to Output!! else Input.
PA3 = nc             WU2/wakeup           # is on +3.3v
PA4 = GND = FIFOADR0 - encodes the fifo to be used ie. ep2,4,6,8
PA5 = GND = FIFOADR1
PA6 = +3.3v = PKTEND   use it to commit last packet at 0-transition.
PA7 = GND = SLCS (FLAGD)  (slave fifo chip select). This can be used to disable the fifo data input!!!!  act-0. high floats all fifo inputs!
---> PA7 function needs to be selected in PORTACFG = 0x40; 

PB0 = D0 from tuner, the data input to fifo
PB1 = D1
PB2 = D2
PB3 = D3
PB4 = D4
PB5 = D5
PB6 = D6
PB7 = D7

PortD is normal port:
PD0 = H/V control lnb 0=18V-H;1=13V-V
PD1 = Tuner RESET. 0=ON;1=off
PD2 = nc
PD3 = nc
PD4 = nc
PD5 = output to enable the slave fifo outside port. usefull to isolate the cx24116 in case of trouble
PD6 = nc
PD7 = nc

SCL = i2c
SDA = i2c
IF Data output:
SLRD = slave read strobe act-0. +3.3v (enables IFCLK). only used for data output!
IF Data Input:
SLWR = enable for IFCLK pin!act-0. -> inverted, DATA_VALID_TUNER act-1 . if 1, data is captured, else not

IFCLK = input CLOCK mpeg stream from tuner. data-valid on rising edge of clock. same as fx2lp!
CLKOUT = nc
CTL0 = nc (FLAGA) - programable FLAG (EP asto FIFOADR[1:0]). this outputs slave-fifo ep2in full/transfers. nice for debug
CTL1 = nc (FLAGB) - FULL flag (EP asto FIFOADR[1:0])
CTL2 = nc (FLAGC) - EMPTY flag (EP asto FIFOADR[1:0])


https://groups.google.com/g/comp.arch.embedded/c/0X0vFWcoM0M/m/UZ7Gnunn__8J

https://www.altx.ro/projects/dvb-usb-direct-interface/

Copyright: Oct-2021  Thomas Krueger Germany
*/

// place data items in order defined!!! we use it for the descriptor tables below.
#pragma ORDER

#define ALLOCATE_EXTERN   // this a special Keil define to format the following includefiles correctly!
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"





//macro: byte swap for the descriptor WORD-items
#define BIGTOLITTLE(x)  ((x>>8)|(x<<8))

//protos:
void SetupCommand(void);
bit vendorcommands(void);
void Initialize(void);
void delay( WORD ms);
void cmd_transfer(void);
bit i2c_read(BYTE wvalue, BYTE wlength);
bit i2c_write(BYTE wlength);
BYTE readreg(BYTE adr, BYTE reg);
void I2Cinit(void);
bit I2Cwrite(BYTE addr, BYTE length, BYTE xdata *dat);
bit I2Cread(BYTE addr, BYTE length, BYTE xdata *dat);

void mcopy(BYTE *from, BYTE *to, BYTE len);
void ep2reset(void);

// ++++++    XDATA ++++++ contains the USB Descriptors  +++++++++

//defines for PID VID VERSION:

#define VENDOR 	0x04b4
#define PRODUCT	0x2104   //dvb world usb device
#define VERSION	0x600


// NOTE: on 8081, portpins are bit addressable (really, how nice, SFR). syntax fe. bit5 of portA is named: PA5 (see fx2regs.inc in keil includes)
#define LED1 PA0
#define LED2 PA1
#define POLARITY PD0
#define CHIPRESET   PD1
#define FIFOEN	PD5 // enable the slave fifo if 1

// MUST start at even address!! check linker mapfile
// insert dummy bytes if necessary
DEVICEDSCR xdata myDeviceDscr  =
{
    sizeof(DEVICEDSCR),		// Descriptor length
    DEVICE_DSCR,			// Descriptor type
    0,			// USB spec version 2.00
    0x02,
    0,				// Device class
    0,				// Device sub-class
    0,				// Device sub-sub-class
    64,				// Max packet size for EP0 (bytes). we have 64,
    BIGTOLITTLE(VENDOR), // Vendor ID (Future Technology Devices Intl). swap bytes!!
    BIGTOLITTLE(PRODUCT), // Product ID (FT8U100AX Serial Port)
    BIGTOLITTLE(VERSION), // Product version (1.00)
    1,				// Manufacturer string index
    2,				// Product string index
    3,				// Serial number string index
    1				// Number of configurations
};

// MUST start at even address!! see linker mapfile
DEVICEQUALDSCR  xdata myDeviceQualDscr   =
{
    sizeof(DEVICEQUALDSCR),	// Descriptor length
    DEVQUAL_DSCR,		// Descriptor type
    0,
    0x02,			// USB spec version 2.00 as a WORD: lsByte first
    0,				// Device class
    0,				// Device sub-class
    0,				// Device sub-sub-class
    64,				// Max packet size for EP0 (bytes)
    1				// Number of alternate configurations
};

// MUST start at even address!! check linker mapfile. This stuff is sent in one go alltogether!
BYTE xdata myConfigDscr[]  =
{
    sizeof(CONFIGDSCR),		// Descriptor length
    CONFIG_DSCR,			// Descriptor Type
    0x12,		// Total len of this array. Will be set in init function!!**************************
    0x34,
    1,				// Number of interfaces supported
    1,				// Configuration index for SetConfiguration()
    0,				// Config descriptor string index
    bmBUSPWR|bmSELFPWR,		// Attributes  bmSELFPWR=bit6, bmBUSPWR=bit7=must be set in any case.
    3,				// Maximum Power Consumption in 2mA units 

    // interface 0 descriptor Management CommunicationClass interface
    sizeof(INTRFCDSCR),		// Descriptor length
    INTRFC_DSCR,			// Descriptor type
    0,				// Index of this interface (zero based)
    0,				// Value used to select alternate
    1,				// Number of endpoints
    0xff,			// Class Code
    0,			// Subclass Code
    0,			// Protocol Code
    2,				// Index of interface string description


    //EP2 Descriptor bulk IN, 512 bytes for mpeg-ts
    sizeof(ENDPNTDSCR),  ENDPNT_DSCR, 0x82, EP_BULK, 0x00, 0x02, 0,

 };

BYTE xdata dummy; // to make even address for next descriptor

//languange ID
// MUST start at even address!! check linker mapfile
BYTE xdata string0[] =
{
    4,  // length of this array
    STRING_DSCR,
    0x09,
    0x04   // US languagne code
};

//manufacturer
// MUST start at even address!! check linker mapfile
BYTE xdata string1[] =
{
    8,  // length of this array
    STRING_DSCR,
    'T',00,
    'K',00,
    'R',00,
};

//product
// MUST start at even address!! check linker mapfile
BYTE xdata string2[] =
{
    32,  // length of this array
    STRING_DSCR,
    'M',00,
    'P',00,
    'E',00,
    'G',00,
    '-',00,
    'T',00,
    'S',00,
    ' ',00,
    '-',00,
    'D',00,
    'o',00,
    'n',00,
    'g',00,
    'l',00,
    'e',00,
};

//serial
BYTE xdata string3[] =
{
    18,  // length of this array
    STRING_DSCR,
    'A',00,
    '1',00,
    '2',00,
    '3',00,
    '4',00,
    '5',00,
    '6',00,
    'T',00,
};




// ++++++++++++  Data   +++++++++++++++++++++++++++++++++
// we do not allow suspend/wakeup as processor never goes to sleep.
bit Rwuen_allowed = 0;	// DisAllow remote wakeup, this also means Suspend is not allowed
bit Rwuen = 0;		// Remote wakeup 0
bit Selfpwr = 1;		// Device is  self-powered

WORD ptr;
BYTE Configuration;  // Current set device configuration
BYTE Interface;      // Current set interface



DWORD Millisecs;
WORD Tim1, Delaytim;


#define IOSIZE 70
BYTE xdata Iobuf[IOSIZE]; // used for Rx and Tx I2C
bit GotSudav=0; // if set, a setup request is pending.
bit noSTOP=0; // controls repeat start


BYTE Mac[8]= { 00,0x18,0xbd,0x5d,0xbf,0x78,0,0};



// ++++++++++++++++++++     CODE +++++++++++++++++++++++++++

// macro for generating the address of an endpoint's control and status register (EPnCS)
#define epcs(EP) (EPCS_Offset_Lookup_Table[(EP & 0x7E) | (EP > 128)] + 0xE6A1)

// this table is used by the epcs macro
const char code  EPCS_Offset_Lookup_Table[] =
{
    0,    // EP1OUT
    1,    // EP1IN
    2,    // EP2OUT
    2,    // EP2IN
    3,    // EP4OUT
    3,    // EP4IN
    4,    // EP6OUT
    4,    // EP6IN
    5,    // EP8OUT
    5,    // EP8IN
};


void main(void)
{
    int i=0;
	unsigned int t,ini,p,f;
	
    Initialize();

// Disconnect the USB interface,wait about 1,5 seconds,  reconnect as new device

// conditional DISCON cycle
/*
if (!(USBCS&0x02)) // if RENUM is 0 thats after initial c0 load from eeprom, firmware was downloaded first time.
{
    USBCS |= 0x0A; //disconnect (DISCON=1, RENUM=1)
    delay(1500);  //IMPORTANT, if no delay, device will not come up!
}
else
*/	
// LED2=0; // debug led on if: RENUM was 1. host still downloads the firmware again, but that does not harm. RENUM stays 1 over DISCON cycle.
//*** we are flashing the device with C2 firmware,so the descriptors are uptodate. 
//    the host will then download firmware again which may be different.	this way host only downloads 1 time with no renumeration!
LED1=0; // this indicates the downloaded firmware is running!
FIFOEN=1; //disable fifoport from demodulator
    USBIRQ = 0xff;          // Clear any pending USB interrupt requests.
    EPIRQ = 0xff; // clear Endpoint ints
    EXIF &= ~0x10;      // clear USBINT

    // CT1 |= 0x02; // become FullSpeed Device. comment this out if you want to be a High-Speed Device.
   USBCS &= ~(0x08); // clear (DISCON=0).
   USBCS |=0x02; // set RENUM to use custom descriptors


    CHIPRESET=0;
    delay(200);
    CHIPRESET=1;
    delay(200);
	
FIFOEN=0; //enable fifoport

ini=0xffff;
t=ini;

    while(1) //mainloop
    {

	
		if (GotSudav)
		{
			GotSudav=0;
		  SetupCommand(); //serve the setup request.there will be no other setup request until the current one is serviced!.
		}

// check for EP2IN transfer problems:
/* FIFOFULL , FIFOEMPTY and FIFOHALFFULL Flags are monitored 0xffff times:
 LED1: ON if FIFOFULL never active in period(Host is reading), AND FIFOEMPTY detected (Tuner not delivering data)
 LED1: TOGGLE if FIFOFULL never active in period(Host is reading), AND FIFOEMPTY not detected (normal streaming)
 LED1: OFF if some FIFOFULL detected in period (Host not reading)

 LED2: ON if FIFOHALFFULL always active in period(Host not reading fast enough, calls RESETFIFO)
 LED2: OFF if FIFOHALFFULL not always active in period(Host reading fast enough)
 states ON and OFF may change in interval period, so TOGGLE of LED2 is possible. (ChannelChange)

 normal streamstate: 		LED2 OFF, 	LED1=toggles. (Tuner delivers, Host reads)
 NoStreaming from Tuner:	LED2=OFF; 	LED1=ON;  	  (Tuner not delivering, Host reads)
 NoStreaming at all:		LED2=OFF; 	LED1=OFF;  	  (Tuner not delivering, Host not reading)
 
 NoStreaming to Host:		LED2=ON;  	LED1=OFF;     (Tuner delivers, Host not reading fast enough, RESETFIFO)
 HostNotReading fast:		LED2=toggle; LED1=OFF;	  (Tuner delivers, Host not reading fast enough, RESETFIFO)
 HostNotReading at all:		LED2=ON; 	LED1=ON; 	  (Tuner not delivering, Host not reading fast enough, RESETFIFO)
*/

		if (!t) // check time interval
		{
			// LED1=show streamstate: 
			
			if (!f)    // no EPfull encountered,
			{
				if (EP2FIFOFLGS &0x02) // if empty flag is set, there is no data coming from tuner, ie.not tuned
				LED1=0; //constantly ON = no tuner data coming ie. cx24116 does not stream
				else
				LED1=!LED1; // toggle = to indicate active streaming 
			}
			else // some full detected
			{
				LED1=1; // some EPfull encountered, host not reading
			}
			
			
		// EPprog constantly ON=we had errors, we where constantly over 256bytes fillstate. blinks if state persists.
			if ((p==ini)||(f&&p)||(f==ini)) 
			{
					LED2=0; // we have delivery problems, host not reading fast enough

			// as to my fixup of the i2c communication being errorfree now, it seems no more problems appear.
					//ep2reset();
					
			}
			else
				LED2=1;
		
			// if f and p, then bad streaming, corrupt picture. kaffeine . maybe also reset 24116?!
			// if f == t; no streaming output, player reports no input! then nothing happens any more. only on kaffeine!
			// p is always nonzero
			// e is always zero
			// f is nonzero if host read problem
		
		/*	//alternate test to above
			if (f==ini) LED1=0; // nur das kommt
			else LED1=1;
			if (p==ini) LED2=0;
			else LED2=1;
		*/	
			
			//reinit
			t=ini;
			p=f=0;
		}
		else //update status counters
		{
			t--;
// FlagBits: 0=full(1), 1=empty(2), 2=halffull(4)		
			if (EP2FIFOFLGS &0x01) // if ep2 is full, fifo is full, host not read at all
			{
				f++;
			}

			if (EP2FIFOFLGS &0x04) // if ep2 is prog, fifo half full, host not reading fast
			{
				p++;
			}
			
		}
			

    } //endwhile

}


//-----------------------------------------------------------------------------
// Control Endpoint 0 Device Request handler
//-----------------------------------------------------------------------------

/*
bmRequestType:
D7 Data Phase Transfer Direction
0 = Host to Device WRITE
1 = Device to Host READ
D6..5 Type
0 = Standard
1 = Class
2 = Vendor
3 = Reserved
D4..0 Recipient
0 = Device
1 = Interface
2 = Endpoint
3 = Other
4..31 = Reserved
SETUPDAT[0] = bmRequestType
SETUPDAT[1] = bmRequest
SETUPDAT[2:3] = wValue
SETUPDAT[4:5] = wIndex
SETUPDAT[6:7] = wLength of next data packet in or out
*/
void SetupCommand(void)  // this an interrupt service routine!
{



    // Errors are signaled by stalling endpoint 0.

    switch(SETUPDAT[0] & SETUP_MASK)    //0x60: switch by type
    {

    case SETUP_STANDARD_REQUEST: //0 = standard requests
        switch(SETUPDAT[1])
        {
        case SC_GET_DESCRIPTOR:
            switch(SETUPDAT[3])
            {
            case GD_DEVICE:
                SUDPTRH = MSB(&myDeviceDscr);
                SUDPTRL = LSB(&myDeviceDscr);
                break;

            case GD_DEVICE_QUALIFIER:
                SUDPTRH = MSB(&myDeviceQualDscr);
                SUDPTRL = LSB(&myDeviceQualDscr);
                break;

            case GD_CONFIGURATION:
                SUDPTRH = MSB(&myConfigDscr);
                SUDPTRL = LSB(&myConfigDscr);
                break;
            case GD_OTHER_SPEED_CONFIGURATION:
                SUDPTRH = MSB(&myConfigDscr);
                SUDPTRL = LSB(&myConfigDscr);
                break;
            case GD_STRING: //get string descriptor asto index in SETUPDAT[2]
                switch (SETUPDAT[2])
                {
                case 0: //lang
                    ptr = &string0;
                    break;
                case 1:
                    ptr = &string1;
                    break;
                case 2:
                    ptr = &string2;
                    break;
                case 3:
                    ptr = &string3;
                    break;
                default:
                    ptr = 0;
                    break;
                }
                if (ptr) // if valid string supported
                {
                    SUDPTRH = MSB(ptr);
                    SUDPTRL = LSB(ptr);
                }
                else
                    EZUSB_STALL_EP0();
                break;

            default:            // Invalid request
                EZUSB_STALL_EP0();
            }
            break;


        case SC_GET_INTERFACE:
            EP0BUF[0] = Interface;
            EP0BCH = 0;
            EP0BCL = 1;
            break;
        case SC_SET_INTERFACE:
            Interface = SETUPDAT[2];
            break;
        case SC_SET_CONFIGURATION:
            Configuration = SETUPDAT[2]; //wvalue
            break;
        case SC_GET_CONFIGURATION:
            EP0BUF[0] = Configuration;
            EP0BCH = 0;
            EP0BCL = 1;
            break;
        case SC_GET_STATUS:
            switch(SETUPDAT[0])
            {
            case GS_DEVICE:
                EP0BUF[0] = ((BYTE)Rwuen << 1) | (BYTE)Selfpwr; //selfpwr=bit0
                EP0BUF[1] = 0;
                EP0BCH = 0;
                EP0BCL = 2;
                break;
            case GS_INTERFACE:
                EP0BUF[0] = 0;
                EP0BUF[1] = 0;
                EP0BCH = 0;
                EP0BCL = 2;
                break;
            case GS_ENDPOINT: //get stall status
                EP0BUF[0] = *(BYTE xdata *) epcs(SETUPDAT[4]) & bmEPSTALL;
                EP0BUF[1] = 0;
                EP0BCH = 0;
                EP0BCL = 2;
                break;
            default:            // Invalid Command
                EZUSB_STALL_EP0();
            }
            break;
        case SC_CLEAR_FEATURE:
             switch(SETUPDAT[0])
            {
            case FT_DEVICE:
                if(SETUPDAT[2] == 1)
                    Rwuen = 0;       // Disable Remote Wakeup
                else
                    EZUSB_STALL_EP0();
                break;
            case FT_ENDPOINT: // take out of stall
                if(SETUPDAT[2] == 0)
                {
				
//#define epcs(EP) (EPCS_Offset_Lookup_Table[(EP & 0x7E) | (EP > 128)] + 0xE6A1)
                   *(BYTE xdata *) epcs(SETUPDAT[4]) &= ~bmEPSTALL; //clear endpoint stall bit
                    EZUSB_RESET_DATA_TOGGLE( SETUPDAT[4] );  //  and reset data-toggle.  
                }
                else
                    EZUSB_STALL_EP0();
                break;
            }
            break;
        case SC_SET_FEATURE:
            switch(SETUPDAT[0])
            {
            case FT_DEVICE:
               if((SETUPDAT[2] == 1) && Rwuen_allowed)
                    Rwuen = 1;      // Enable Remote Wakeup
                else
                    EZUSB_STALL_EP0();
                break;
            case FT_ENDPOINT: // stall endpoint
                *(BYTE xdata *) epcs(SETUPDAT[4]) |= bmEPSTALL; // set endpoint stall bit
                break;
            default:
                EZUSB_STALL_EP0();
            }
            break;
        default:  // *** Invalid Command
            EZUSB_STALL_EP0();
            break;
        }
        break;
        // end standard requests

    case 0x40: // vendor commands
        if (vendorcommands() == 1)
            break;
/* test: unknown setup commands	. i did not find any!
		else
		{
			if (SETUPDAT[0]==0x40) //write
			LED1=0; //test
			else
				LED2=0; //test
		}
*/
    default: // unknown request
        // negative acknowledge/not supprted
        EZUSB_STALL_EP0(); //No answer!. set stall-bit in ep0cs.  the USB core sends the STALL PID for any IN or OUT token.
        break;
    }

    // Acknowledge handshake phase of device request
    EP0CS |= bmHSNAK; // ACK the transfer. clear bit7 in ep0cs by writing a 1 to it. this instructs the USB core to ACK the STATUS stage.
}

//returns 1 if successfull processed, else 0 to stall ep0 as nack.
bit vendorcommands(void)
{
    /*
    format of the linux-driver call function:
    u8 request, u16 value, u16 index, u8 * data, u16 len. -> bmRequest, wValue, wIndex, EP0buf, wLength.

    Endpoint 0 has only one 64 bytes data buffer for either read or write. The 8 Byte SETUP message resides in a different buffer.
    This is used for command interface like i2c transfers, read or write, reset-control, HV setting etc.:
    SETUPDAT[0] contains the direction bit (Bit7).
    0=write operation
    1=read operation
    Format of the received 8-Byte SETUP message for our command interface:
    SETUPDAT[1] = bmRequest is the vendor request. ie. the command given to us.
    SETUPDAT[2:3] = wValue, not used, 0 or the registeraddress, or offset when software download.
    SETUPDAT[4:5] = wIndex, not used always 0
    SETUPDAT[6:7] = wLength of data in EP0BUF, if any. this contains the i2c transfer request:
    content of the 64byte EP0BUF on receiving a i2c-request:
    i2c_address_with direction_bit_set; length of requested transfer in bytes; [ Registeraddress, [Data...]]
    so DATA-section at offset 2.
	
	TESTED on 29.01.2022, fixed EP0 BUSY check in write operations. Now no error in communication with cx24116!
    */

    BYTE wvalue,wlength,windex;
    BYTE i;
    BYTE ret_len=0;

	wvalue= SETUPDAT[2];
    windex= SETUPDAT[4];
    wlength=SETUPDAT[6];
	

// +++READ: we need to return data in EP0BUF
    if (SETUPDAT[0] == 0xC0)
    {

        switch(SETUPDAT[1]) // Request, command interface , the command
        {

		// i2c_read from wvalue=i2c_adr<<1; wlength=transferlen -2; put result in ep0buf_offset=2.
        case 0xC3: //read from current registerpointer of cx24116 the registervalue(s):
			if (!i2c_read( wvalue, wlength)) 
			{
				wlength=0;
				return(0); // this is new
			}
			ret_len=wlength;
			break;
		

        case 0xB8: //RC query, remote control query if command was given.READ, expects 2 byte being returned.
			if (wlength==2)
			{
			EP0BUF[0]=0;
			EP0BUF[1]=0;
			ret_len=wlength;  // this caused trouble, sporadic chars in terminal
			}
            break;
			
        case 0xB6: //read eeprom 256 byte in chunks of 2byte using windex as offset.firstbyte is value, second byte is 0.
            i=windex&7;
            EP0BUF[0]=Mac[i];
            ret_len=2;
            break;
			
 //       case 0xB5: // read from tuner, frequency
 //           break;

        default:
			return(0); // not supported, stall endpoint
        }

 if (ret_len) // arming an endpoint is done by setting the bytecount register. here we send zero length packets!
        {
            EP0BCH = 0;
            EP0BCL = ret_len;
        }

    } // end read

// +++WRITE: EP0OUT, we do not need to return data, just ACK
    else if (SETUPDAT[0] == 0x40)
    {
		
        switch(SETUPDAT[1]) // Request, command interface , the command
        {
        case 0xB2: //set voltage control or write tuner.WRITE: 2 bytes, 0x30=setvoltage,setvalue 1=18V-Hor,0=14V-Vert
		// the band select is done by 22khz signal send from the cx24116 which is send by a write command.
		if (EP0BUF[0]==0x30) // set voltage
		{
			if (EP0BUF[1]) // if vertical
			POLARITY=0; //V
			else
			POLARITY=1; //H
		delay(10);
		}
            break;

		// write to cx24116 including a registervalue to set the registerpointer. 
        case 0xC2: //write cx24116 register.EP0BUF: i2c_address<<1, transfer_len_requested, : registervalue,[Data...]
            if (!i2c_write(wlength))
            {
		//it fails if i2c adr != 0xaa fe. probing a ds3000 device.
		// important: we have to wait for the BUSY bit in EP0CS to clear, so the EP0-OUT transfer is completed.	
		// testing:
		// ;//LED1=0; //write error occured
		// ;// if (EP0BUF[0]!=0xaa) LED2=0; // wrong i2c address encountered
           return(0); 
            }
            
            break;

        case 0xC4: //write, reset cx34116 going active one byte of data telling the reset state wanted.
			if (EP0BUF[0]==1) // do a cx24116 reset cycle
			{
			CHIPRESET=0;
			delay(10);
			CHIPRESET=1;
			delay(10);
			}
            break;

        case 0xBF: //write, wData=0x40, reset cx34116 going inactive. we not use this as zerolength data packet trouble
            break;

        case 0x8A: //voltage control, HV ?? this is done above!
		break;

//        case 0x80: //write firmware(download after usb connect). we do not see this!!
//            break;

        default:
         return(0); // not supported, stall endpoint
        }



    } //end write

// some delay seems good;)
	SYNCDELAY;
/*	
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
	SYNCDELAY;
*/
    return (1); //OK
}


void Initialize(void)
{

    // init outputs for leds. led1=PA0,led2=PA1
    OEA=0x03; // output direction
    IOA=0x03; //off
    // init outputs on Port D for : PD0=14=1/18V=0 select,PD1=chipReset act 0, Debug serial output PD5
    OED=0x23; // output direction
    IOD=0x20; //Reset on=0, 18V, uart=1

    I2Cinit();


    // init timer0 as delay timer
    TMOD = 0x10; //timer0 as 13bit timer with overflow int enabled; timer1 as 16bit up counter,clocked at 12Mhz.
    TH0 = 0;			// load 16bit count-register
    TL0 = 0;
    TR0 = 1;			// Start timer0
//	PT0 = 1; // set tim0 to high prio
    ET0 = 1; // enable timer0 ints




    // adjust the length of the total config descriptor, as it cannot be evaluated at compiletime
    ptr = sizeof(myConfigDscr);
    myConfigDscr[2]=ptr;
    myConfigDscr[3]=ptr>>8;

    CPUCS = 0x10; // CLKSPD[1:0]=10, set 48 MHz operation
    SYNCDELAY;
    CKCON=0; // set stretch value to 0 = fastest DATA access. also timers are clocked by 48MHZ/12
    SYNCDELAY;

// auto arming endpoints and cpu intervention control for the slave fifo. only relevant for out transfers.
    REVCTL=0x03;
    SYNCDELAY;

    IFCONFIG = 0x03; // this connects the slavefifo control/data pins to the ports.
    SYNCDELAY;
	
    /*
    Bit 0-1: = 0x03 , sets IFCFG to slave fifo mode, external master.
    Bit 2: = 0, disable GSTATE output on PE pins
    Bit 3: = 0, ASYNC off, selects synchronious mode ie. sycronious to the input clock ifclk
    Bit 4: = 0, IFCLK polarity = not inverted
    Bit 5: = 0, IFCLKOE is off as we are inputting the clock from outside
    Bit 6: = 0, 3048mhz, internal fifo clock frequency 0=30mhz, 1=48mhz, only relevant if IFCLKSRC = 1. 
    Bit 7: = 0, IFCLKSRC, fifo clock source. 0=external, 1=internal asto 3048mhz. we use external clock from tuner!!!

    The fifo databus-width is selected in the EP2FIFOCFG register, bit0  (WORDWIDE) 0=8-bit mode.
    NOTE: bit0 (WORDWIDE) must be clear in all EPxFIFOCFG registers!!

     IFCLK Source external (i.e.) connected to MPEG_CLK minimum should be 5 MHz
     Synchronous Mode, Free Running MPEG_CLK,  MPEG_VALID is used as SLWR strobe
     FX2LP in SLAVE FIFO Mode
     */
	 
// reset all fifos:
    FIFORESET = 0x80; // activate NAK-ALL to avoid race conditions. ie. NAK all transfers
    SYNCDELAY; // see TRM section 15.14

    FIFORESET = 0x82; // reset, FIFO 2
    SYNCDELAY; //

    FIFORESET = 0x84; // reset, FIFO 4
    SYNCDELAY; //

    FIFORESET = 0x86; // reset, FIFO 6
    SYNCDELAY; //

    FIFORESET = 0x88; // reset, FIFO 8
    SYNCDELAY; //

    FIFORESET = 0x00; // deactivate NAK-ALL, resume normal operation
    SYNCDELAY;

// clear all fifo flags
    PINFLAGSAB = 0x00; //defines FLAGA as prog-level flag = half full,FLAGB as full flag
    SYNCDELAY;
    PINFLAGSCD = 0x00; //FLAGC as empty flag, FLAGD (PA7) is not activated
    SYNCDELAY;
// set fill level of FLAGA=level-programmable flag to 512/2=256 so you have a nice squarwave to monitor on scope.
// 0xC1 = FLAGA asserts if 256 Bytes have filled the fifo
// 

	EP2FIFOPFH = 0xC1; //0xC1;  (1,0)= fill-level bits8,9; BIT7=DECIS=1; BIT6=PKTSTAT=1. default=0x88;
	SYNCDELAY; // to be active at the level you wish
	EP2FIFOPFL = 0x00; // = fill level lowbyte. default=0x00;
	SYNCDELAY;


    PORTACFG = 0x40; //used PA7 as SLCS, the slave fifo chip select
    SYNCDELAY;

    FIFOPINPOLAR = 0x04; // SLWR is configured as active HIGH : PKTEND is active high
    SYNCDELAY;


    EP2CFG = 0xE0;   // VALID - 1,DIR - IN,Type- Bulk, Size - 512 Bytes, Quad Buffered
    SYNCDELAY;

    EP4CFG = 0x00; // clear valid bit
    SYNCDELAY; //

    EP6CFG = 0x00;	 // clear valid bit
    SYNCDELAY; //

    EP8CFG = 0x00; // clear valid bit
    SYNCDELAY;

// config ep2 as datainput slave fifo
    EP2FIFOCFG = 0x08; // 0x08 = autoin, or 0x0c = +allow 0-length packets. AUTO IN asto AUTOINLEN, NO Zero Length Packets, 8- bit Wide bus (bit0=WORDWIDE = 0 for 8bit mode).
    SYNCDELAY;

    EP2AUTOINLENH = 0x02; // Auto-commit 512-byte packets
    SYNCDELAY;

    EP2AUTOINLENL = 0x00;
    SYNCDELAY;
	
    // clear all fifo WORDWIDE bits to enable Port D as GPIO. maybe we need to set IFCONFIG after here !!!
    EP4FIFOCFG = 0x00;
    SYNCDELAY;

    EP6FIFOCFG = 0x00;
    SYNCDELAY;

    EP8FIFOCFG = 0x00;
    SYNCDELAY;


// arm ep0buf
/*
    EP0BCH = 0;
    EP0BCL = 64;
*/



//    AUTOPTRSETUP |= 0x01;         // enable dual autopointer feature for easy data copy
//    Rwuen = 1;                 // Enable remote-wakeup
    INTSETUP=0; //disable autovector interrupts
    GPIFIE=0; //disable INT4 ints, GPIF stuff




    SUDPTRCTL = 1; //use automatic descriptor sending in SETUP requests ( AUTO mode). descriptor needs to be at even address!!

// EndPoint interrupts: are enabled in EPIE. They are flagged in EPIRQ. Active EPIRQ ints cause a USBINT (int2) where the source can be checked in EPIRQ.
// in the USBINT service routine you can also check the autovector-value in INT2VEC register to tell the source.
    EPIE |= 0x02 ;              // Enable EP0OUT interrupt


    // Note: The Endpoint Interrupts also cause a USBINT, even when nothing is enabled in USBIE.
    // USBINT is actually INT2 being used as USB interrupt
    // Enable Bits in the USBIE interrupt source register, served by interrupt 8 vector in USBINT:
    USBIE = 0x11; //SUDAV(0), URES(4)
// general interrupt enable
    EUSB = 1; // enable USB interrupt
    EA = 1;  //   IE = 0x80; //global interrupt enable
}


/*
INT2 = USBINT  general interrupt service:  (except RESUME)
This is the only vector for all USB related interrupts.
- all USBINTS flagged in USBIRQ-register and enabled in USBIE register
- all flagged EPIRQ ints enabled in EPIE
The 27 sources of the USBINT can be determined by the register INT2IVEC.
It hold the autovector value of the source, but we disabled autovectoring, so use a switch to indentify the source.
see Techman page 63!
*/
void USB_isr(void) interrupt 8
{



    switch (INT2IVEC)
    {
    case 0: //SUDAV setup command received. We just received the EP0 SETUP stage
        USBIRQ = 0x01;  // Clear SUDAV IRQ_bit by write a 1 to it.

// if vendor_write and nonzero wlength, enable ep0buf out data stage, so the out transfer can happen.
// a datastage only follows, if wlength is non zero!!!
		if ((SETUPDAT[0]==0x40)&&SETUPDAT[6]) 
		{
			// delay setup processing until datastage has arrived
			EP0BCL = 0; // trigger ep0out interrupt
			SYNCDELAY;

		}
		else
		GotSudav=1; //request setup processing in main loop, no datastage
			
 // the EP0 OUT datastage has not been received yet, so we arm EP0BUF for it.
 // only set EP0BCL, this starts the ep0-out datastage and sets the BUSY bit in EP0CS. 
 // we then wait for BUSY-bit to clear to indicate that the ep0 datastage has arrived.
	break;
 
 	case 0x24: //  EP0-OUT data has arrived	in setup request with data stage
		EPIRQ = 0x02; // clear ep0out int flag
		GotSudav=1; //request setup processing in main loop, datastage
		
        break;
		
	case 0x10: //USB Reset issued by host. happens when host does a reconnect after it disconnects the device.
	USBIRQ = bmURES; // Clear URES IRQ
	break;
	
    default:
        break;
    }
    EXIF &= ~0x10; // Clear  USB_IRQ irq flag

}

// 2.047 msecs passed. the system tick service routine using timer0. It always interrupts as has high prio!!
// global vars: millisecs, tim1, tim2 = custom downcounters
void Tim0_int(void) interrupt 1
{
    Millisecs += 2;
    if (Tim1) Tim1--;
    if (Delaytim) Delaytim--;
}

// reset the ep2 in fifo in case of trouble. FIFO_RESET in manual
/*
void ep2reset(void) // see also e6a3/ep2cs register!
{
FIFOEN = 1;		


	
					   FIFORESET = 0x80; // activate NAK-ALL to avoid race conditions. ie. NAK all transfers
						SYNCDELAY; // see TRM section 15.14


						EP2FIFOCFG = 0x00;   //switching to manual mode
						SYNCDELAY;
						
						FIFORESET = 0x82; // reset, FIFO 2
						SYNCDELAY; //
						FIFORESET = 0x82; // reset, FIFO 2
						SYNCDELAY; //
						
						
						EP2FIFOCFG = 0x08;  //switching back to auto-in mode
						SYNCDELAY;

						FIFORESET = 0x00; // deactivate NAK-ALL, resume normal operation
						SYNCDELAY;
		
FIFOEN = 0;		
		
}
*/

/*
delay function
ms = miliseconds wanted to wait, > 1
*/
void delay( WORD ms)
{
    Delaytim=ms/2;

    while (Delaytim); // use timer0 for countdown

}


// copy memory
void mcopy(BYTE *from, BYTE *to, BYTE len)
{
	while (len--) *to++=*from++;
}


/* read transfer_len bytes from starting register in cx24116, using its autoincrement feature for the register pointer.
(needs to be enabled in reg 0xf4_bit7, otherwise we will read from the same register as usefull in tuner transit.)

entry:
- wvalue = i2c-address <<1
- wlength = total length + 2

exit:
The data is in EP0BUF matching wlength
- 1=OK; 0=fail
*/
bit i2c_read(BYTE wvalue, BYTE wlength)
{
    bit ret;
    BYTE adr=wvalue >> 1; // convert back to normal address
    BYTE len=wlength - 2;

    if ((len>IOSIZE)||(wlength < 3)) return (0);
	
    ret=I2Cread(adr, len, Iobuf);
    mcopy(Iobuf, &EP0BUF[2], len);
    return (ret);
}

/* write transfer_len bytes from starting register to cx24116, using its autoincrement feature for the register pointer.
(needs to be enabled in reg 0xf4_bit7, otherwise we will read from the same register as usefull in tuner transit.)

entry:
- wlength = total length of data in EP0BUF

exit:
The data is written
- 1=OK; 0=fail
*/
bit i2c_write(BYTE wlength)
{
    bit ret;
    BYTE adr=EP0BUF[0]>>1; // convert back to normal address
    BYTE len=wlength -2; // sub 2 as the data to write begins at offset 2

    if (len>IOSIZE) return (0);
    mcopy(&EP0BUF[2], Iobuf, len); // copy register and possible data
    ret=I2Cwrite(adr, len, Iobuf); //set register-pointer
    return (ret);
}

/*
//reads one byte from register reg. it first writes the registeraddress to access, then reads its value.
// it uses noStop to do this without releasing the i2c bus in between the 2 transfers. 
// returns byte red. sets global Error bit.
BYTE readreg(BYTE adr, BYTE reg)
{
    noSTOP=1; // enable restart with no stop
    // first set the register by doig a write
    Iobuf[0]=reg;
    I2Cwrite(adr, 1, Iobuf);
    noSTOP=0;
    I2Cread(adr, 1, Iobuf);
    return(Iobuf[0]);
}
*/


// +++++++++   I2C stuff +++++++++++++++++++++++++++++++++

#define I2C_IDLE              0     // I2C Status: Idle mode
#define I2C_SENDING           1     // I2C Status: I2C is sending data
#define I2C_RECEIVING         2     // I2C Status: I2C is receiving data
#define I2CRECSTART             3     // I2C Status: I2C is receiving the first byte of a string
#define I2C_STOP              5     // I2C Status: I2C waiting for stop completion
#define I2C_BERROR            6     // I2C Status: I2C error; Bit Error
#define I2C_NACK              7     // I2C Status: I2C error; No Acknowledge
#define I2C_WAITSTOP          9     // I2C Status: Wait for STOP complete
#define TNG 1

struct
{
    BYTE   length;
    BYTE   *dat;
    BYTE   count;
    BYTE   status;
}  I2CPckt;



void I2Cinit(void)
{
    I2CPckt.status = I2C_IDLE;
    I2CTL = 0x03; // enable I2C Stop interrupt, enable 400 khz
    EI2C = 1;	// Enable I2C interrupt. int occurs on DONE or STOP
}


/* write length bytes to device at i2c_address:
min length = 1.
 ret: 1=ok, 0=fail.
 */
bit I2Cwrite(BYTE addr, BYTE length, BYTE xdata *dat)
{
    I2CS |= bmSTART; // write operation
    I2DAT = addr << 1;  // send address and create start condition

    I2CPckt.status = I2C_SENDING;

    I2CPckt.length = length;
    I2CPckt.dat = dat;
    I2CPckt.count = 0;

    while (I2CPckt.status != I2C_IDLE) //while not finished 
    {
        if ( I2CPckt.status == I2C_NACK) // NACK detected, slave not respond
        {
            I2CS |= bmSTOP; // force stop condition, otherwise we hang
            I2CPckt.status = I2C_WAITSTOP;
            return 0;
        }
    }
    return(1);
}


/* read length bytes from device at i2c_address:
min length = 1.
 ret: 1=ok, 0=fail. xdata pointer as ep0outbuf is also in xdata
 */
bit I2Cread(BYTE addr, BYTE length, BYTE xdata *dat)
{
    I2CS |= bmSTART; // next write to I2DAT generates a start condition
    I2DAT = (addr << 1) | 0x01; // send first byte=address into dat register and create start-condition

    I2CPckt.status = I2CRECSTART; // start the reception on next DONE interrupt

    I2CPckt.length = length;
    I2CPckt.dat = dat;
    I2CPckt.count = 0;

    while (I2CPckt.status != I2C_IDLE) //while not finished
    {
        if ( I2CPckt.status == I2C_NACK) // NACK detected, slave not respond
        {
            I2CS |= bmSTOP; // force stop condition, otherwise we hang
            I2CPckt.status = I2C_WAITSTOP;
            return 0;
        }
    }
    return(1);
}


/* I2C INT
fires when a stop condition is completed, or a TransferDone of one byte is signaled(DONE-Bit).
DONE=1 indicates transfer complete 1 byte. ACK bit will be set at the same time it sets DONE=1, if no ACK bit, its a NACK.
 a NACK during write operation indicates a slave not responding/ not there.

 Repeat-Start means, that after a i2c transfer no i2c_stop is generated but another i2c_start. a delay in between is ok.
 After a repeat start, the slave address must be send!.

 SDA remains Low and Bus is in Busy condition for infinite time, until a Start or Stop event happens.no other busmaster can claim the bus then.
 Normally the SDA line must not changge while SCL is high, but:
 Start condition: SDA drops from high to Low while SCL is High.
 Stop condition:  SDA rises from low to High while SCL is High.
 Bus Busy condtition: SCL is High while SDA is Low.
 A Slave can pull SCL low to indicate its not ready to send now, this is called clock stretching. But normally SCL is High.
 
 The noStop flag can be set to enable multible transfer without releasing the Bus.
 This is usefull fe. to transfer data in passthrough from cx24116 to the tuner.
*/
void i2c_isr(void) interrupt 9
{
    // I2C State Machine

    if(I2CS & bmBERR)
    {
		// a second bus-master was detect, creating a busconflict. This should never happen!
        I2CPckt.status = I2C_BERROR; // BUSERROR: we cannot resolve this state!!! possibly issue a STOP request.
    }
    else if ((!(I2CS & bmACK)) && (I2CPckt.status != I2C_RECEIVING)) 
	// NACK: no acknowledge during write operation(DONE=1), ie. slave didnt respond!
    {
        I2CPckt.status = I2C_NACK; 
		// Note that ACK-Bit is only meaningful in a write transaction as it is generated by the slave.

    }
    else
    {
        switch(I2CPckt.status)
        {
        case I2C_SENDING:
            I2DAT = I2CPckt.dat[I2CPckt.count++];
            if(I2CPckt.count == I2CPckt.length)
                I2CPckt.status = I2C_STOP;
            break;
        case I2CRECSTART:
            I2CPckt.dat[I2CPckt.count] = I2DAT; // clock in next byte by reading from DAT register. this is a dummy read.
            I2CPckt.status = I2C_RECEIVING;
            if(I2CPckt.length == 1) // may be only one byte to read
                I2CS |= bmLASTRD;  // issue NACK to slave: float SDA at ACK-time to create a NACK to the slave to stop sending
            break;
        case I2C_RECEIVING:
            if(I2CPckt.count == I2CPckt.length - 2)
                I2CS |= bmLASTRD; // issue NACK to slave, last byte
            if(I2CPckt.count == I2CPckt.length - 1)
            {
                if (!noSTOP) //no multibyte io
                {
                    I2CS |= bmSTOP; // release the bus
                }
                I2CPckt.status = I2C_IDLE;
            }
            I2CPckt.dat[I2CPckt.count] = I2DAT;
            ++I2CPckt.count;
            break;

//stop after sending. this releases the bus and may not be desired. instead a new start can be signaled to continue with a new transmission.
        case I2C_STOP:
            if (noSTOP) // multibyte io
                I2CPckt.status = I2C_IDLE;
            else
            {
                I2CS |= bmSTOP; // release the bus
                I2CPckt.status = I2C_WAITSTOP;  // wait for next STOP interrupt
            }

            break;
        case I2C_WAITSTOP: //triggered by the STOP interrupt
            I2CPckt.status = I2C_IDLE;
            break;
        default:
            break;
        }


    }
    EXIF &= ~0x20;		// Clear interrupt flag for i2c by writing a 0 to it.
}

