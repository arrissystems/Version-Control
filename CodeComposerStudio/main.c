
#include <msp430.h>

/*
 * main.c
 * Revision 02/26/2018 added haskey to detect keys and added delay to fix key bounce
 * Revision 03/02/2018 changed writeintensity to show Adult double values
 * Revision 03/14/2018 fixed keybounce changed scanhnd to use average as base value
 * Revision V1 04/19/2018 Enter into Version Control in Git
 * Revision V2 04/19/2018 Changed scanhand timing and display baseline
 * Revision V3 05/18/2018 Added clktest and make array size selectable
 * Revision V4 06/02/2018 Added automatic voltage level setting for scanning
 * Revision V5 06/08/2018 Added recall of scanning and recall of RTP, LTP, and levels, independent Scan voltage levela
*/

#define VER4    true
#define VER5    true
#define clktest false       //true

//#include    "msp430f2274.h"
//#include    "msp430f2748.h"
//#include    "msp430fR5848.h"

#include    "msp430G2955.h"
//#include    "msp430G2855.h"

#include <stdint.h>
#include <stdbool.h>
//#include "boot.h"

#include    "charmap.h"
#include    "newimmumax2.h"

#define BYTE    unsigned char
#define scandly 450                                                     // scan pulse width
#define offdly  150                                                     // trailing
#define setdly  100                                                     // leading
int     lscanvt;                                                        // scan voltage
int     rscanvt;                                                        // scan voltage
#define arrayj  8                                                       // 8, 4, 2 VER3
#define arrayc  3                                                       // 3, 2, 1 VER3

int main(void);
void timeout(void);
void delay0(int xdelay);
void delay1(int xdelay);
void delay2(int xdelay);
void pulsexfmr(void);
void ExitSleep (void);
void EnterSleep (void);
void write_data(unsigned char data1);
void write_command(unsigned char command);
void scanadc(unsigned int INCHAN);
void sdelay(int xdelay);
void charAll(int redmask, int greenmask, int bluemask);
void beeper(void);
unsigned int leftscanchan(void);
unsigned int rightscanchan(void);
void leftselchan(int bii);
void rightselchan(int bii);
void writeintensity(void);

// RTC variables
int         TSEC;
int         SEC;
int         MIN;
int         UserMinutes;
int         red;
int         green;
int         blue;
int         SwitchData;
int         powerison;
int         previousstate;
bool        powerup;
bool        haskey;
bool        rdbattery;
int         dlycnt0;
int         dlycnt1;
int         dlycnt2;
int         dlycnt3;
int         clockcnt;
bool        callclock;
int         hasswitch;
bool        norewrite;
bool        scanit;
bool        scanning;
BYTE        RUP;
BYTE        LUP;
int         CHILD;
int         idata;
int         p1iflag;
bool        beeperon;
int         displaytimer;
int         switchcnt;

bool        ADCDone;
int         ADCValue;
int         BATValue;
int         iCol;
int         iRow;
int         iCols;
int         iRows;
long        iaddress;
unsigned int    iCSAddress;
unsigned int    iRSAddress;
unsigned int    iCEAddress;
unsigned int    iREAddress;
int         ichar;
int         iPixC;
int         iPixR;
int         NUMA[3]={0,0,0};

//unsigned char timercounter;

int         leftadcval;
int         rightadcval;
int         leftmax;
int         rightmax;
int         leftbar[11];
int         rightbar[11];
int         leftadcarray[13][arrayj];                                   // VER3
int         rightadcarray[13][arrayj];                                  // VER3
int         leftarray[13];
int         rightarray[13];
//int         leftmean;
//int         rightmean;
int         ltpoint;
int         rtpoint;
bool        extmode;
int         ltmp;
int         leftmin;
int         rightmin;
int         leftave;
int         rightave;
int         leftrange;
int         rightrange;
BYTE        llastp4;
BYTE        rlastp4;

char        PrgAry[5];
int         BatPcnt;

char digits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

const unsigned char * all[] =                                           //94 addresses
{
     spa,
     chEX,
     DQ,
     chpd,
     ch$,
     chPC,
     chAM,
     chAP,
     lp,
     rp,
     star,
     plus,
     comma,
     minus,
     period,
     fslash,
     ch0,
     ch1,
     ch2,
     ch3,
     ch4,
     ch5,
     ch6,
     ch7,
     ch8,
     ch9,
     colon,
     semi,
     LT,
     equal,
     GT,
     QM,
     chAT,
     AA,
     BB,
     CC,
     DD,
     EE,
     FF,
     GG,
     HH,
     II,
     JJ,
     KK,
     LL,
     MM,
     NN,
     OO,
     PP,
     QQ,
     RR,
     SS,
     TT,
     UU,
     VV,
     WW,
     XX,
     YY,
     ZZ,
     lb,
     bslash,
     rb,
     chXO,
     us,
     chGA,
     cha,
     chb,
     chc,
     chd,
     che,
     chf,
     chg,
     chh,
     chi,
     chj,
     chk,
     chl,
     chm,
     chn,
     cho,
     chp,
     chq,
     chr,
     chs,
     cht,
     chu,
     chv,
     chw,
     chx,
     chy,
     chz,
     lcb,
     orr,
     rcb,
     tilde,
//     clock1

};

void delay0(int xdelay)                                                 // delay for xdelay ms
{
    dlycnt0 = xdelay;                                                   // use timer interrupt
    while(dlycnt0>0);
}

void delay1(int xdelay)                                                 // delay for xdelay ms
{
    dlycnt1 = xdelay;                                                   // use timer interrupt
    while(dlycnt1>0);
}

void delay2(int xdelay)                                                 // delay for xdelay ms
{
    dlycnt2 = xdelay;                                                   // use timer interrupt
    while(dlycnt2>0);
}


void sdelay(int xdelay)                                                 // delay for xdelay nops 0.6133uS
{
int intii;
    for(intii=xdelay;intii>0;intii--)                                   // use nops
    {
        __no_operation();
    }
}

void beeper()
{
    dlycnt3=30;
    beeperon=true;
}


/*************************************************/
void write_command(unsigned char command)
{
    P4DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 +BIT6 + BIT7);    // Set P2.0,1,2,3,4,5 to output
    P4OUT = command;                                                    // output to 8 bit data port
    P2OUT |= (BIT1 + BIT2);                                             // RD,WR high
    P2OUT &= ~(BIT3);                                                   // set D/C low for command
    P2OUT &= ~(BIT0);                                                   // /CS low
    P2OUT &= ~(BIT1);                                                   // set WR low P2.1
    P2OUT |= (BIT1);                                                    // set WR high P2.1
    P2OUT |= (BIT0);                                                    // /CS high
}

void write_data(unsigned char data1)
{
    P4DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);   // Set P2.0,1,2,3,4,5 to output
    P4OUT = data1;                                                      //output to 8 bit data port
    P2OUT |= (BIT1 + BIT2 + BIT3);                                      //set RD,WR,D/C high
    P2OUT &= ~(BIT0);                                                   // /CS low
    P2OUT &= ~(BIT1);                                                   // set WR low P2.1
    P2OUT |= (BIT1);                                                    // set WR high P2.1
    P2OUT |= (BIT0);                                                    // /CS high

}

void writemonochar(int tmp, int REDM, int GRNM, int BLUM)
{
    if(tmp==0)
    {
        red = REDM;
        green = GRNM;
        blue = BLUM;
    }
    else
    {
        red = 0;    //0xff;                                             // white background
        green = 0;  //0xff;                                             // white background
        blue = 0;   //0xff;                                             // white background
    }

    write_data(red);
    write_data(green);
    write_data(blue);

}

void write_char(int tmp, int REDM, int GRNM, int BLUM)
{
    writemonochar(tmp & 0x01, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x02, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x04, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x08, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x10, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x20, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x40, REDM, GRNM, BLUM);
    writemonochar(tmp & 0x80, REDM, GRNM, BLUM);

}

void initi(void)
{
    P2OUT |= (BIT1 + BIT2);                                             //WR high. RD high
    P2OUT &= ~(BIT0);                                                   // /CS low
    //   res=1;
    P3OUT |= (BIT5);                                                    //LCD LRESET high
    delay1(1);                                                          //delay 1 ms
    //   res=0;
    P3OUT &= ~(BIT5);                                                   //LCD LRESET Low
    delay1(10);                                                         //delay 10 ms
    //   res=1;
    P3OUT |= (BIT5);                                                    //LCD LRESET back high
    delay1(120);                                                        //delay 120 ms


//**********************************************************************//LCD SETING
    write_command(0x11);                                                //sleepout command turn off sleep
    delay1(120);                                                        //Delay 120ms

    write_command(0x36);                                                //Memory data access control
    write_data(0xE0);                                                   //BIT7=1 Page address left to right
                                                                        //BIT6=1 Column left to right set to 1
                                                                        //BIT5=1 Normal Page/column Order set to 1 for top to bottom
                                                                        //BIT4=0 line left to right 320 lines
                                                                        //BIT3 RGB mode

    write_command(0xB2);                                                //  Porch Setting
    write_data(0x0C);
    write_data(0x0C);
    write_data(0x00);
    write_data(0x33);
    write_data(0x33);

    write_command(0xB7);                                                //Gate Control
    write_data(0x70);                                                   //VGH,VGL

    write_command(0xBB);                                                //VCOM
    write_data(0x33);

    write_command(0xC0);
    write_data(0x2C);

    write_command(0xC2);
    write_data(0x01);

    write_command(0xC3);                                                //VAP
    write_data(0x14);                                                   //12

    write_command(0xC4);
    write_data(0x20);

    write_command(0xC6);
    write_data(0x0F);                                                   //60HZ  0A

    write_command(0xD0);
    write_data(0xA4);
    write_data(0xA1);                                                   // AVDD VCL

    write_command(0xE0);
    write_data(0xD0);
    write_data(0x07);
    write_data(0x0D);
    write_data(0x09);
    write_data(0x08);
    write_data(0x25);
    write_data(0x28);
    write_data(0x53);
    write_data(0x39);
    write_data(0x12);
    write_data(0x0B);
    write_data(0x0A);
    write_data(0x17);
    write_data(0x34);

    write_command(0xE1);
    write_data(0xD0);
    write_data(0x07);
    write_data(0x0D);
    write_data(0x09);
    write_data(0x09);
    write_data(0x25);
    write_data(0x29);
    write_data(0x35);
    write_data(0x39);
    write_data(0x13);
    write_data(0x0A);
    write_data(0x0A);
    write_data(0x16);
    write_data(0x34);

//    write_command(0x21);                                              //Display white background
    write_command(0x20);                                                //Display black background

    write_command(0x3A);
    write_data(0x06);                                                   //18 bit per pixel, 66-24BIT

    write_command(0x2A);                                                //
    write_data(0x00);
    write_data(0x00);
    write_data(0x01);
    write_data(0x3F);

    write_command(0x2B);                                                //
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xEF);

    //**********************
    write_command(0x29);                                                //display on
    write_command(0x2C);                                                //RAM write control

 }

//*********************************************************************************************
void EnterSleep (void)
{
    P3OUT &= ~(BIT2);                                                   // turn off LLED
    write_command(0x28);
    delay1(100);                                                        // delay 20ms
    write_command(0x10);
    delay1(100);                                                        // delay 20ms

}

//*********************************************************
void ExitSleep (void)

{
    write_command(0x11);
    delay1(120);                                                        // delay 120ms
    write_command(0x29);

}



void    write_address()
{
    int  highs;
    int  lows;
    int  highe;
    int  lowe;

    lows = iCSAddress & 0x00ff;
    highs = iCSAddress >> 8;
    lowe = iCEAddress & 0x00ff;
    highe = iCEAddress >> 8;

    write_command(0x2A);                                                //CASET Column Address Set 00,00,01,3f
    write_data(highs);                                                  //XS8-15 Start Address
    write_data(lows);                                                   //XS0-7 at 0
    write_data(highe);                                                  //XE8-15 End Address 01
    write_data(lowe);                                                   //XE0-7 at 3f

    lows = iRSAddress & 0x00ff;
    highs = iRSAddress >> 8;
    lowe = iREAddress & 0x00ff;
    highe = iREAddress >> 8;

    write_command(0x2B);                                                //RASET Row Address Set 00, 00, 00, ef
    write_data(highs);                                                  //YS8-15 Start
    write_data(lows);                                                   //YS0-7
    write_data(highe);                                                  //YE8-15 End Address 00
    write_data(lowe);                                                   //YE0-7 at ef

}




void    charAll(int redmask, int greenmask, int bluemask)
{
    int    tmp;

    unsigned int pointer;
    unsigned int charoffset;

    charoffset = ichar-0x20;

    unsigned char *iints = (unsigned char*)all[charoffset];

    // start writing to LCD memory
    // write the character A iPixC X iPixR pixels
    iCEAddress = 320-(iCols*iPixC)-1;                                   //Column start at 0 to max 0000013f
    iREAddress = 240-(iRows*iPixR)-1;                                   //Row start at 0 to 240 max 000000ef
    iCSAddress = iCEAddress-iPixC+1;                                    //iCols-1+character pixels per column max 0000013f
    iRSAddress = iREAddress-iPixR+1;                                    //iRows-1+character pixels per row max 000000ef
    write_address();
    write_command(0x002C);                                              //Memory write

    pointer = 80;
    while(pointer>0)                                                    // 80 ints per character 16X20 pixels
    {
        if((pointer & 0x01)==0)                                         //skip the extra word from paint conversion
        {
            pointer-=2;
        }

        pointer-=1;
        tmp = iints[pointer];

        writemonochar(tmp & 0x01, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x02, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x04, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x08, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x10, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x20, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x40, redmask, greenmask, bluemask);
        writemonochar(tmp & 0x80, redmask, greenmask, bluemask);

    }
}


#pragma vector=PORT1_VECTOR
__interrupt void myport1_isr(void)
{
    if(powerup==true)
    {
        __bic_SR_register(LPM3_bits);                                   // CPU back on
        P1IFG=0;
        WDTCTL = WDTPW | WDTHOLD;                                       // stop watchdog timer
        __asm("\t   MOV.W\t    #0x20FE,SP");                            // reload stack
        __asm("\t   JMP\t    main");                                    // go to main

    }

    if(P1IFG == 0)
    {
        return;
    }
    else
    {
        p1iflag = P1IFG;
        SwitchData = ~P1IFG;

    }
    hasswitch+=1;
    P1IFG = 0;                                                          // clear interrupt flags
}


//  Interrupt Service Routines
#pragma vector = TIMER0_A0_VECTOR                                       // 0.9765ms each interrupt
__interrupt void CCR0_ISR(void) {
    if(dlycnt0>0)
    {
        dlycnt0-=1;
    }
    if(dlycnt1>0)                                                        // for xdelay1()
    {
        dlycnt1-=1;
    }
    if(dlycnt2>0)                                                        // for xdelay2()
    {
        dlycnt2-=1;
    }
    if(switchcnt>0)
    {
        switchcnt-=1;
    }

    if(scanning == false)
    {
        clockcnt-=1;
        if(clockcnt==0)
        {
            callclock=true;
            clockcnt=512;
            TSEC+=1;
            if(TSEC>1)
            {
                TSEC=0;
                SEC+=1;
                if(displaytimer>0)
                {
                    displaytimer-=1;
                }
            }
            if(SEC>59)
            {
                SEC=0;
                MIN+=1;
                rdbattery = true;
            }
        }


        if(beeperon)
        {
            if(dlycnt3>0)
            {
                dlycnt3-=1;
                P3OUT |= (BIT1);
            }
            else
            {
                beeperon=false;
                P3OUT &= ~(BIT1);
            }
        }
    }
    //_enable_interrupt();
}



void scanadc(unsigned int INCHAN)                                       // scan the ADC channel of the battery
{
    ADCDone = true;
    ADC10CTL0 &= ~ENC;                                                  // Disable ADC

    ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;                         // 16 clock ticks, ADC On, enable ADC interrupt

    ADC10CTL1 = ADC10SSEL_3 + INCHAN;                                   // Set 'chan', SMCLK

    ADC10CTL0 |= ENC + ADC10SC;                                         // Enable and start conversion
    while(ADCDone);
}


// ADC interrupt routine. Pulls CPU out of sleep mode for the main loop.


#pragma vector=ADC10_VECTOR

__interrupt void ADC10_ISR (void)
{
    ADCValue = ADC10MEM;                                                // Saves measured value.
    ADCDone = false;                                                    // Sets flag for main loop. TRUE
}


BYTE    xchangebits(int ibits)
{
BYTE cii;
    switch (ibits)
    {
    case 1:
        cii=8;                                                          // 1.543V
        break;
    case 2:
        cii=4;                                                          // 1.825V
        break;
    case 3:
        cii=12;                                                         // 2.161V
        break;
    case 4:
        cii=2;                                                          // 2.47V
        break;
    case 5:
        cii=10;                                                         // 2.778V
        break;
    case 6:
        cii=6;                                                          // 3.087V
        break;
    case 7:
        cii=14;                                                         // 3.396V
        break;
    case 8:
        cii=1;                                                          // 4.97V
        break;
    case 9:
        cii=9;                                                          // 5.248V
        break;
    case 10:
        cii=5;                                                          // 5.557V
        break;
    case 11:
        cii=13;                                                         // 5.866V
        break;
    case 12:
        cii=3;                                                          // 6.175V
        break;
    case 13:
        cii=11;                                                         // 6.483V
        break;
    case 14:
        cii=7;                                                          // 6.792V
        break;
    case 15:
        cii=15;                                                         // 7.101V
        break;
    default:
        cii=0;                                                          // 1.235V
        break;

    }
    return cii;
}

void convertdigits(int inumber)
{
    int iRemainder;
    int iTemp1;

    NUMA[0] = inumber / 100;                                            //get the ms digit (hundred)

    iTemp1 = NUMA[0] * 100;
    iRemainder = inumber - iTemp1;                                      //get the remainder (1 to 99)
    NUMA[1] = iRemainder / 10;                                          //get the 2nd digit (tens)

    iTemp1 = NUMA[1] * 10;                                              //get the ls digit (single digit)
    NUMA[2] = iRemainder - iTemp1;
}

void write3digits(int inumber, int REDM, int GRNM, int BLUM)
{
    convertdigits(inumber);                                             //convert numbers to array of 3 digits

    if(NUMA[0] == 0)                                                    //write ms digit
    {
        ichar = ' ';
    }
    else
    {
        ichar = NUMA[0]+0x30;
    }
    charAll(REDM, GRNM, BLUM);

    iCols+=1;                                                           //write 2nd digit
    ichar = NUMA[1]+0x30;
    charAll(REDM, GRNM, BLUM);

    iCols+=1;                                                           //write ls digit
    ichar = NUMA[2]+0x30;
    charAll(REDM, GRNM, BLUM);

}

void writedigits(int inumber, int REDM, int GRNM, int BLUM)
{
    convertdigits(inumber);                                             //convert numbers to array of 3 digits

    if(NUMA[0] == 0)                                                    //write ms digit
    {
        ichar = ' ';
    }
    else
    {
        ichar = NUMA[0]+0x30;
    }
//    charAll(0, 0, 0);

//    iCols+=1;                                                         //write 2nd digit
    ichar = NUMA[1]+0x30;
    charAll(REDM, GRNM, BLUM);

    iCols+=1;                                                           //write ls digit
    ichar = NUMA[2]+0x30;
    charAll(REDM, GRNM, BLUM);

}

void blankscreen(void)
{
int intjj;
int intii;

    //  Blank screen
    write_command(0x2A);                                                //
    write_data(0x00);
    write_data(0x00);
    write_data(0x01);
    write_data(0x3F);

    write_command(0x2B);                                                //
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xEF);

    //**********************
    write_command(0x29);                                                //display on
    write_command(0x2C);                                                //RAM write control

    for(intjj=240;intjj>0;intjj--)                                      // write 240 lines rows
    {
        for(intii=320;intii>0;intii--)                                  // write 320 pixels Columns
        {
            red = 0;
            green = 0;
            blue = 0;
            write_data(red);
            write_data(green);
            write_data(blue);
        }
    }

    write_command(0x29);                                                //display on
}


void writechild(void)
{
unsigned int    i;
//display program selection: CHILD or ADULT

    iRows=3;
    if(extmode == true)
    {
        PrgAry[0] = 'T';
        PrgAry[1] = 'P';
        PrgAry[2] = 'S';
        PrgAry[3] = 'E';
        PrgAry[4] = 'L';
        for (i=0;i<5;i++) {
            iCols=11+i;
            ichar=PrgAry[i];
            charAll(0xff, 0x7f, 0xff);
        };
    }
    else
    {
        if (CHILD==0)                                                   // VER5 init CHILD to 0
        {
            PrgAry[0] = 'C';
            PrgAry[1] = 'H';
            PrgAry[2] = 'I';
            PrgAry[3] = 'L';
            PrgAry[4] = 'D';
            for (i=0;i<5;i++) {
                iCols=11+i;
                ichar=PrgAry[i];
                charAll(0xff, 0x7f, 0xff);
            };
        };
        if (CHILD==1)
        {
            PrgAry[0] = 'A';
            PrgAry[1] = 'D';
            PrgAry[2] = 'U';
            PrgAry[3] = 'L';
            PrgAry[4] = 'T';
            for (i=0;i<5;i++) {
                iCols=11+i;
                ichar=PrgAry[i];
                charAll(0xff, 0xff, 0x7f);
            };
       };
    }

}


void writeintensity(void)
{
unsigned int    i, j, k;
//write program intensity
    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical

    iRows = 10;                                                         //display left hand intensity
    iCols = 1;
    j=LUP;
    writedigits(j, 0xff, 0, 0xff);

    iRows = 10;                                                         //display right hand intensity
    iCols = 17;
    k=RUP;
    writedigits(k, 0xff, 0x7f, 0);

    //display intensity bar
    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical
    ichar = 'Q';

    iRows=2;                                                            //display left intensity bar
    iCols=1;
    if(CHILD==1)                                                        // VER5 init CHILD to 0
    {
        j=LUP>>1;
    }
    for (i=0;i<7;i++)
    {
        if(i<j)
        {
            ichar = 'Q';
        }
        else
        {
            ichar = ' ';
        }
        iRows++;
        charAll(0xff, 0, 0xff);
    }

    iRows=2;                                                            //display right intensity bar
    iCols=18;
    if(CHILD==1)                                                        // VER5 init CHILD to 0
    {
        k=RUP>>1;
    }
    for (i=0;i<7;i++)
    {
        if(i<k)
        {
            ichar = 'Q';
        }
        else
        {
            ichar = ' ';
        }
        iRows++;
        charAll(0xff, 0x7f, 0);
    }

    writechild();

}

void    pulsexfmr (void)
{
    if(powerison==1)
    {
        if(LUP>0)
        {
            P2OUT |= (BIT4);                                            // turn on LHVTRIG
        }
        if(RUP>0)
        {
            P2OUT |= (BIT5);                                            // turn on RHVTRIG
        }

        sdelay(750);
        P2OUT &= ~(BIT4+BIT5);                                          // turn P2.4,5 off
    }
}

void voltageoff(void)
{
    llastp4 = 0;
    rlastp4 = 0;
    P4OUT = llastp4;                                                    // turn off voltage regulator
    P3OUT |= (BIT3+BIT4);                                               // strobe the LVSEL + RVSEL
    sdelay(3);
    P3OUT &= ~(BIT3+BIT4);
    sdelay(3);

}

void leftsetvoltage(int volt)
{
    llastp4 = xchangebits(volt) + BIT4;
    P4OUT = llastp4;                                                    // set to level 0 voltage enable voltage regulator
    P3OUT |= (BIT3);                                                    // strobe the LVSEL
    sdelay(3);
    P3OUT &= ~(BIT3);
}

void rightsetvoltage(int volt)
{
    rlastp4 = xchangebits(volt) + BIT4;
    P4OUT = rlastp4;                                                    // set to level 0 voltage enable voltage regulator
    P3OUT |= (BIT4);                                                    // strobe the RVSEL
    sdelay(3);
    P3OUT &= ~(BIT4);
}

void leftselchan(int bii)
{
    int    aii;
    int    bii1;
    bii1=bii+2;

//    P4OUT &= ~(BIT5 + BIT6);                                          // turn off Din, CLK
    P4OUT = llastp4;                                                    // turn off Din, CLK
    P4OUT |= (BIT7);                                                    // clear the shift register
    P3OUT |= (BIT3);                                                    // strobe the LVSEL
    sdelay(3);
    P3OUT &= ~(BIT3);

    P4OUT &= ~(BIT7);                                                   // turn off clear bit
    P3OUT |= (BIT3);                                                    // strobe the LVSEL
    sdelay(3);
    P3OUT &= ~(BIT3);

    for(aii=0; aii<bii1; aii++)
    {
        if(aii==0)
        {
            P4OUT |= (BIT5);                                            // Set Din=1
            P3OUT |= (BIT3);                                            // strobe the LVSEL
            sdelay(3);
            P3OUT &= ~(BIT3);
        }
        sdelay(3);
        P4OUT |= (BIT6);                                                // Clock high
        P3OUT |= (BIT3);                                                // strobe the LVSEL
        sdelay(3);
        P3OUT &= ~(BIT3);

        P4OUT &= ~(BIT5 + BIT6);                                        // Set clock, Din low
        P3OUT |= (BIT3);                                                // strobe the LVSEL
        sdelay(3);
        P3OUT &= ~(BIT3);
    }
    P4OUT &= ~(BIT5 + BIT6 + BIT7);
    sdelay(3);
}


unsigned int leftscanchan(void)
{
    P2OUT |= (BIT4);                                                    // turn on LHVTRIG
    sdelay(scandly);                                                    // 0.6133 us each
    scanadc(INCH_5);                                                    // return value in ADCValue
    P2OUT &= ~(BIT4);                                                   // turn LHVTRIG off
    return  ADCValue;
}

void rightselchan(int bii)
{
    int    aii;
    int    bii1;
    bii1=bii+2;
//    P4OUT &= ~(BIT5 + BIT6);                                          // turn off Din, CLK
    P4OUT = rlastp4;
    P4OUT |= (BIT7);                                                    // clear the shift register
    P3OUT |= (BIT4);                                                    // strobe the RVSEL
    sdelay(3);
    P3OUT &= ~(BIT4);

    P4OUT &= ~(BIT7);                                                   // turn off clear bit
    P3OUT |= (BIT4);                                                    // strobe the RVSEL
    sdelay(3);
    P3OUT &= ~(BIT4);

    for(aii=0; aii<bii1; aii++)
    {
        if(aii==0)
        {
            P4OUT |= (BIT5);                                            // Set Din=1
            P3OUT |= (BIT4);                                            // strobe the RVSEL
            sdelay(3);
            P3OUT &= ~(BIT4);
        }
        sdelay(3);
        P4OUT |= (BIT6);                                                // Clock high
        P3OUT |= (BIT4);                                                // strobe the RVSEL
        sdelay(3);
        P3OUT &= ~(BIT4);

        P4OUT &= ~(BIT5 + BIT6);                                        // Set clock, Din low
        P3OUT |= (BIT4);                                                // strobe the RVSEL
        sdelay(3);
        P3OUT &= ~(BIT4);
    }
    P4OUT &= ~(BIT5 + BIT6 + BIT7);
    sdelay(3);

}

unsigned int rightscanchan(void)
{
    P2OUT |= (BIT5);                                                    // turn on RHVTRIG
    sdelay(scandly);                                                    // 0.6133 us each
    scanadc(INCH_6);                                                    // return value in ADCValue
    P2OUT &= ~(BIT5);                                                   // turn RHVTRIG off
    return  ADCValue;
}

void scanhand(void)
{
    unsigned int i, j;

    scanning = true;
    P1IE  = 0x00;                                                       // 0-6 inputs interrupt disable
    P1IFG = 0x00;                                                       // clear interrupt flags

    // first test to activate the voltage regulators
    lscanvt = 4;
    rscanvt = 4;
    leftsetvoltage(lscanvt);                                            // changes P4, strobe LVSEL
    rightsetvoltage(rscanvt);                                           // changes P4, strobe RVSEL
    delay0(500);                                                        // suppress trailing edge droop
    leftselchan(12);                                                    // changes P4, strobe LVSEL
    rightselchan(12);                                                   // changes P4, strobe RVSEL
    delay0(setdly);                                                     // suppress leading edge spike
    leftadcval=leftscanchan();
    rightadcval=rightscanchan();

    while(lscanvt < 10)
    {
        leftsetvoltage(lscanvt);                                        // changes P4, strobe LVSEL

        leftmax=0;
        leftmin=2048;
        leftave=0;

        for(i=0; i<11; i++)
        {
            leftselchan(i);                                             // changes P4, strobe LVSEL
            delay0(setdly);                                             // suppress leading edge spike
            leftadcval=leftscanchan();
            leftarray[i] = leftadcval;
            delay0(offdly);                                             // suppress trailing edge droop

            if(leftmax < leftarray[i])
                leftmax = leftarray[i];
            if(leftmin > leftarray[i])
                leftmin=leftarray[i];
        }

        if((leftmax - leftmin) > 11)
        {
            break;
        }
        lscanvt+=1;
    }

    while(rscanvt < 10)
    {
        rightsetvoltage(rscanvt);                                       // changes P4, strobe RVSEL

        leftmax=0;
        leftmin=2048;
        leftave=0;

        rightmax=0;
        rightmin=2048;
        rightave=0;

        for(i=0; i<11; i++)
        {
            rightselchan(i);                                            // changes P4, strobe RVSEL
            delay0(setdly);                                             // suppress leading edge spike
            rightadcval=rightscanchan();
            rightarray[i] = rightadcval;
            delay0(offdly);                                             // suppress trailing edge droop

            if(rightmax < rightarray[i])
                rightmax = rightarray[i];
            if(rightmin > rightarray[i])
                rightmin=rightarray[i];
        }

        if((rightmax - rightmin) > 11)
        {
            break;
        }
        rscanvt+=1;
    }

    lscanvt+=1;                                                         // give it one higher level
    rscanvt+=1;                                                         // give it one higher level
    leftsetvoltage(lscanvt);                                            // changes P4, strobe LVSEL
    rightsetvoltage(rscanvt);                                           // changes P4, strobe RVSEL

    leftarray[12] = 0;
    rightarray[12] = 0;

    for(j=0; j<arrayj; j++)
    {
        leftselchan(12);                                                // changes P4, strobe LVSEL
        rightselchan(12);                                               // changes P4, strobe RVSEL
        delay0(setdly);                                                 // suppress leading edge spike
        leftadcval=leftscanchan();
        rightadcval=rightscanchan();
        leftadcarray[12][j] = leftadcval;
        leftarray[12] += leftadcval;
        rightadcarray[12][j] = rightadcval;
        rightarray[12] += rightadcval;
        delay0(offdly);                                                 // suppress trailing edge droop

    }

    leftarray[12] = leftarray[12]>>arrayc;
    rightarray[12] = rightarray[12]>>arrayc;

    for(j=0;j<arrayj;j++)                                               // VER3
    {
        for(i=0;i<11;i++)
        {
            leftadcarray[i][j]=0;
            rightadcarray[i][j]=0;
            leftselchan(i);                                             // changes P4, strobe LVSEL
            rightselchan(i);                                            // changes P4, strobe RVSEL
            delay0(setdly);                                             // suppress leading edge spike
            leftadcval=leftscanchan();
            rightadcval=rightscanchan();
            leftadcarray[i][j] = leftadcval;
            rightadcarray[i][j] = rightadcval;

            delay0(offdly);                                             // suppress trailing edge droop
        }
    }

    P1IE  = 0x7F;                                                       // 0-6 inputs interrupt enable
    voltageoff();
    scanning = false;

    leftmax=0;
    leftmin=2048;
    leftave=0;

    rightmax=0;
    rightmin=2048;
    rightave=0;

    for(i=0;i<11;i++)
    {
        leftarray[i]=0;
        rightarray[i]=0;
        for(j=0;j<arrayj;j++)                                           // VER3
        {
            leftarray[i]+=leftadcarray[i][j];
            rightarray[i]+=rightadcarray[i][j];
        }
        leftarray[i] = leftarray[i]>>arrayc;                            // VER3 get average
        rightarray[i] = rightarray[i]>>arrayc;                          // VER3

        leftarray[i] = leftarray[i] - leftarray[12];
        rightarray[i] = rightarray[i] - rightarray[12];

        if(leftarray[i] < 0)
            leftarray[i] = 0;
        if(rightarray[i] < 0)
            rightarray[i] = 0;

        leftave+=leftarray[i];
        if(leftmax < leftarray[i])
            leftmax = leftarray[i];
        if(leftmin > leftarray[i])
            leftmin=leftarray[i];

        rightave+=rightarray[i];
        if(rightmax < rightarray[i])
            rightmax = rightarray[i];
        if(rightmin > rightarray[i])
            rightmin=rightarray[i];
    }
    leftave=leftave/11;
    rightave=rightave/11;

    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical

    iRows = 11;                                                         //display left hand intensity
    iCols = 3;
    write3digits(leftmax, 0xff, 0, 0xff);
    iCols +=1;
    write3digits(lscanvt, 0xff, 0, 0xff);

    iRows = 11;                                                         //display right hand intensity
    iCols = 12;
    write3digits(rightmax, 0xff, 0x7f, 0);
    iCols +=1;
    write3digits(rscanvt, 0xff, 0x7f, 0);

    leftrange = leftmax-leftave;                                        //leftmax-leftave

    rightrange = rightmax-rightave;                                     //rightmax-rightave

    if(leftrange < 10)                                                  // no signal
    {
        leftrange = 0x7fff;
    }

    if(rightrange < 10)                                                 // no signal
    {
        rightrange = 0x7fff;
    }

    for (i=0;i<11;++i)                                                  //get values from adc
    {
        ltmp=leftarray[i]-leftave;
        ltmp = ltmp*7;
        ltmp = ltmp / leftrange;

        if(ltmp > 1)
        {
            leftbar[i]=ltmp;
        }
        else
        {
            leftbar[i]=0;
        }

        ltmp=rightarray[i]-rightave;
        ltmp = ltmp*7;
        ltmp = ltmp / rightrange;

        if(ltmp > 1)
        {
            rightbar[i]=ltmp;                                           //-rightmin;
        }
        else
        {
            rightbar[i]=0;
        }

        if (leftbar[i] > 7)
        {
            leftbar[i] =7;
        };
        if (rightbar[i] > 7)
        {
            rightbar[i] =7;
        };
    }

}


int Init(void)
{
    P2SEL = (BIT6 + BIT7);                                              // Set P2.6,7 to select XIN, XOUT
    BCSCTL1 &= (~XTS);                                                  // ACLK=LDXT1CLK
    BCSCTL3 &= ~(BIT4 | BIT5);                                          // 32768 crystal on LFXT1
    TACCR0 = 32 - 1;                                                    // Timer A period of 32 cycles 1/1024 sec 0.9765ms
    TACCTL0 = CCIE;                                                     // Enable interrupts for CCR0.
    TACTL = TASSEL_1 + ID_0 + MC_1 + TACLR;                             // ACLK, div 1, up mode, clear timer

    P1OUT = 0x7F;
    P1SEL = 0;                                                          // Set P1.0,7 to I/O
//    P1SEL2 = 0x00;                                                    // Turn off capacitive sensing
    P1DIR = 0x80;                                                       // Set P1.0-6 to Input 7 to output
    P1REN = 0x7F;                                                       // Turn on P1.0-6 pull ups
    P1IE  = 0x7F;                                                       // 0-6 inputs interrupt enabled
    P1IES = 0xFF;                                                       // interrupt on high to low edge
    P1IFG = 0;

//    P2SEL2 = 0x00;                                                    // Turn off capacitive sensing
    P2DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);                 // Set P2.0,1,2,3,4,5 to output
//    P2REN |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);               // Turn on P2.0,1,2,3,4,5 pull ups
    P2OUT |= BIT0;                                                      // turn off /LCS

    P3DIR = 0x3E;                                                       // Set P3.1-5 to output direction, 0,6,7 adc
//    P3REN |= (BIT1 + BIT2 + BIT3 + BIT4 + BIT5);
    P3SEL = 0;                                                          // Set P3.1-5 to I/O
//    P3SEL2 = 0x00;                                                    // Turn off capacitive sensing
    P3OUT= 0x00;
    P3OUT |= (BIT5);                                                    // LCD LRESET back high

    ADC10AE0 = (BIT5 +BIT6 + BIT7);                                     // A5 P3.0 LADC, A6 P3.6 RADC, A7 P3.7 VBAT

    P4SEL = 0x00;                                                       // Set P4 to I/O
    P4DIR = 0xFF;                                                       // Set P4 to output direction
//    P4REN |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7); // Set P2.0,1,2,3,4,5,6,7 to pull ups
//    P4SEL2 = 0x00;                                                    // Turn off capacitive sensing
    llastp4=0;
    rlastp4=0;
    P4OUT=0x00;                                                         // reset the two 373 latches
//    P4OUT=0x11;                                                       // reset the two 373 latches enable regulators
    P3OUT|= (BIT3);                                                     // strobe the LVSEL
    sdelay(3);
    P3OUT &= ~(BIT3);
    P3OUT|= (BIT4);                                                     // strobe the RVSEL
    sdelay(3);
    P3OUT &= ~(BIT4);


//    BCSCTL1 |= CALBC1_16MHZ;                                          // Set DCO to calibrated 16 MHz.
//    DCOCTL = CALDCO_16MHZ;

    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL = CALDCO_8MHZ;

    TSEC=0;
    SEC=0;
    MIN=0;
    powerison=1;
    dlycnt0=0;
    dlycnt1=0;
    dlycnt2=0;
    clockcnt=512;
    callclock=false;
    hasswitch=0;                                                        // switches pressed counter
    haskey=false;
    ADCDone=true;
    rdbattery=true;
    p1iflag=0;
    beeperon=false;
    displaytimer=90;
    switchcnt=0;
    scanit = false;
    scanning = false;

#if  clktest
    P2SEL |=0x01;                                                       // turn on aclk for test
    while(1);
#endif

    return 0;
}

void printscanning(void)
{
    iRows=4;
    iCols=3;
    ichar='S';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='C';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='A';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='N';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='N';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='I';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='N';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='G';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar=' ';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='W';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='A';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='I';
    charAll(0, 0xff, 0xff);
    iCols++;
    ichar='T';
    charAll(0, 0xff, 0xff);

}

void printbars(void)
{
    unsigned int i, j;

    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical
//
//  Blank bar areas
//
    ichar='Z';
    for (i=0;i<11;i++)                                                  //display left hand scan result
    {
        iRows=10-i;
        ichar=' ';
        iCols=2;
        for (j=0;j<15;j++)
        {
            charAll(0xff, 0xff, 0xff);
            iCols+=1;
        }
    }

    ichar='Z';
    for (i=0;i<11;i++)                                                  //display left hand scan result
    {
        iRows=10-i;
        iCols=2;
        for (j=0;j<leftbar[i];j++)
        {
            charAll(0xff, 0xff, 0xff);
            iCols+=1;
        }

        iCols=17-rightbar[i];
        for (j=0;j<rightbar[i];j++)
        {
            charAll(0xff, 0xff, 0xff);
            iCols+=1;
        }
    }

}

void rewritescr(void)
{
    unsigned int i;
    unsigned long intii;
    unsigned int pointer;
    char LabelAry[8];
    int     tmp;

    write_command(0x2A); //
    write_data(0x00);
    write_data(0x00);
    write_data(0x01);
    write_data(0x3F);

    write_command(0x2B); //
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xEF);

    //**********************
    write_command(0x29);                                                //display on
    write_command(0x2C);                                                //RAM write control

    pointer = 9599;

    for(intii=9600;intii>0;intii--)                                     // 16 character 20 pixels= 320 columns
    {
        tmp = newfacemono99[pointer] & 0x01;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x02;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x04;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x08;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x10;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x20;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x40;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        tmp = newfacemono99[pointer] & 0x80;                            // one bit per pixel
        writemonochar(tmp, 0xff, 0xff, 0x00);                           // blue color
        pointer-=1;

    }

    write_command(0x29);                                                //display on


    //write static texts on the LCD

    iPixC = 16;                                                         //16 pixels horizontal per char
    iPixR = 20;                                                         //20 pixels vertical per char

    iRows=7;
    iCols=13;
    ichar = 'M';
    charAll(0xff, 0xff, 0xff);                                          //black color

    iCols++;
    ichar = 'I';
    charAll(0xff, 0xff, 0xff);                                          //black color

    iCols++;
    ichar = 'N';
    charAll(0xff, 0xff, 0xff);                                          //black color

    iRows=4;
    iCols=3;
    ichar='L';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar='T';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar='P';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar=':';
    charAll(0xff, 0xff, 0xff);

    iRows=4;
    iCols=11;
    ichar='R';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar='T';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar='P';
    charAll(0xff, 0xff, 0xff);
    iCols++;
    ichar=':';
    charAll(0xff, 0xff, 0xff);


    //write "MODE:" to display CHILD or ADULT
    LabelAry[0] = 'M';
    LabelAry[1] = 'O';
    LabelAry[2] = 'D';
    LabelAry[3] = 'E';
    LabelAry[4] = ':';

    iRows=3;
    for (i=0;i<5;i++) {
        iCols=3+i;
        ichar=LabelAry[i];
        charAll(0xff, 0xff, 0xff);                                      //black color
    };

    iRows=1;
    iCols=14;
    ichar = '%';
    charAll(0xff, 0xff, 0xff);                                          //black color

    displaytimer=90;                                                    //90 sec LED on
    write_command(0x29);                                                //display on
    P3OUT |= (BIT2);                                                    //turn on LLED
    norewrite=true;
//    beeper();
}

void diagscreen(void)
{
    unsigned int i;
    // this section is to display diagnostic page
    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical

    for (i=11;i>0;--i)                                                  //display 1 to 11 number at the left side
    {
        iRows=11-i;
        iCols=0;
        if (i>9)                                                        //if it is a 2 digit number
        {
            ichar=digits[1];                                            //write ms digit
            charAll(0xff, 0xff, 0xff);
            iCols +=1;                                                  //write ls digit
            ichar=digits[i-10];
            charAll(0xff, 0xff, 0xff);
        }
        else                                                            //only one digit
        {
            iCols +=1;
            ichar=digits[i];
            charAll(0xff, 0xff, 0xff);
        }
    }

    for (i=11;i>0;--i)                                                  //display 1 to 11 number at the right side
    {
        iRows=11-i;
        iCols=17;
        if (i>9)                                                        //if it is a 2 digit number
        {
            ichar=digits[1];                                            //write ms digit
            charAll(0xff, 0xff, 0xff);
            iCols +=1;                                                  //write ls digit
            ichar=digits[i-10];
            charAll(0xff, 0xff, 0xff);
        }
        else                                                            //only one digit
        {
            iCols +=1;
            ichar=digits[i];
            charAll(0xff, 0xff, 0xff);
        }
    }

    ichar='L';
    iRows=11;
    iCols=1;
    charAll(0xff, 0xff, 0xff);

    ichar='R';
    iCols=18;
    charAll(0xff, 0xff, 0xff);
    P3OUT |= (BIT2);                                                    // turn on LLED

}

void writetp(void)
{
    // write left treatment point to LCD
    iPixC = 16;                                                         //16 pixels horizontal per char
    iPixR = 20;                                                         //20 pixels vertical per char

    iRows=4;                                                            //write ms digit of minute
    iCols=7;
    writedigits(ltpoint+1,0,0xff,0xff);                                 //red color

    // write right treatment point to LCD
    iPixC = 16;                                                         //16 pixels horizontal per char
    iPixR = 20;                                                         //20 pixels vertical per char

    iRows=4;                                                            //write ms digit of minute
    iCols=15;
    writedigits(rtpoint+1,0,0xff,0xff);                                 //red color


    // write running time to LCD
    iPixC = 16;                                                         //16 pixels horizontal per char
    iPixR = 20;                                                         //20 pixels vertical per char

    iRows=6;                                                            //write ms digit of minute
    iCols=10;
    writedigits(MIN,0,0xff,0xff);                                       //red color

    iCols+=1;                                                           //write : after minute
    ichar = ':';
    charAll(0xff, 0xff, 0xff);                                          //black color

    iCols+=1;                                                           //write ms digit of second
    writedigits(SEC,0,0xff,0xff);                                       //red color

    // write user selected time to LCD
    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical

    iRows=7;                                                            //write ms digit of minute
    iCols=10;
    writedigits(UserMinutes,0xff,0xff,0xff);                            //black color
    P3OUT |= (BIT2);                                                    // turn on LLED

}

void timeout(void)
{
    powerup=true;                                                       // power off mode
    UserMinutes = 0;
    voltageoff();
    beeper();
    EnterSleep();                                                       // display OFF
    P1IE  = 0x7F;                                                       // 0-6 inputs interrupt enabled
    __bis_SR_register(LPM3_bits+GIE);                                   // enter LPM3 mode
    while(1);

}

int main(void)
{
    int     tmpswitch;

startup:
    WDTCTL = WDTPW | WDTHOLD;                                           // stop watchdog timer
    if(Init()>0)return 1;
    _enable_interrupt();

    initi();                                                            // initial LCD screen

    // After Init:
    // powerison = 1
    // UserMinutes = 0
    // norewrite=false
    // rdbattery=true;
    // displaytimer=90;
    // extmode=false;
    // scanit = false;
    // scanning = false;

    while(1)
    {
        if(powerup==true)
        {
            powerup=false;

            if(previousstate == 0)                                      // re-display last scan
            {
                powerison=1;
                norewrite=false;
                displaytimer=90;
            }

            if(previousstate == 1)                                      // re-display last scan
            {
                powerison=1;
                norewrite=false;
                displaytimer=90;
            }

            if(previousstate == 2)                                      // re-display last scan
            {
                powerison=2;
                blankscreen();

                diagscreen();

                P3OUT |= (BIT2);                                        // turn on LLED
                scanit = false;

                printbars();

                beeper();
                displaytimer = 120;
            }

        }
        else
        {
            if(powerison==1)
            {
                previousstate = 1;
                if(UserMinutes == 0)
                {
                    if((RUP==0) && (LUP==0))
                    {
                        MIN = 0;
                        SEC = 0;
                    }
                }

                if(UserMinutes > MIN)
                {
                    if(callclock)                                       // 0.5 sec
                    {
                        if(LUP > 0)
                        {
                            leftsetvoltage(LUP);                        // changes P4, Strobe LVSEL
                            leftselchan(ltpoint+1);                     // changes P4, Strobe LVSEL
                        }
                        if(RUP > 0)
                        {
                            rightsetvoltage(RUP);                       // changes P4, Strobe RVSEL
                            rightselchan(rtpoint+1);                    // changes P4, Strobe LVSEL
                        }
                        if((LUP>0) || (RUP>0))
                        {
                            delay0(50);
                            pulsexfmr();                                // P2.4,5 to strobe LHVTRIG and HVTRIG
                            voltageoff();                               // changes P4, Strobe LVSEL,RVSEL
                            callclock=false;
                        }
                    }
                    displaytimer = 90;                                  // this will keep display on
                }

                if(UserMinutes == MIN)
                {
                    voltageoff();                                       // changes P4, Strobe LVSEL,RVSEL
                    MIN = UserMinutes;
                    SEC = 0;
                    if(displaytimer==0)
                    {
                        previousstate = 1;
                        timeout();
                    }
                }

                if(rdbattery)                                           // read the battery value
                {
                    scanadc(INCH_7);                                    // return value in ADCValue
                    BATValue = ADCValue;                                // save value in BATValue
                }

                if(norewrite==false)
                {
                    rewritescr();
                    displaytimer=90;
                }

                writetp();

                //display battery percent
                if(BATValue > 449)
                {
                    BATValue = 449;
                }

                if(BATValue < 300)
                {
                    BatPcnt = 0;
                }
                else
                {
                    BatPcnt = BATValue - 300;
                }

                BatPcnt = BatPcnt * 100;
                BatPcnt = BatPcnt / 150;

                iRows=1;                                                //write to LCD
                iCols=11;
                if(BatPcnt < 10)
                {
                    writedigits(BatPcnt,0,0xff,0xff);                   //red color
                }
                else
                {
                    writedigits(BatPcnt,0xff,0xff,0);                   //green color
                }

                writechild();
                writeintensity();
            } //if(powerison==1)
        }

        if(scanit == true)
        {
            scanit = false;

            printscanning();

//            while(1)
//            {
                scanhand();
//            }

            printbars();

            beeper();
            previousstate = 2;
            displaytimer = 120;
        } //if(scanit == true)

        if(powerison==2)
        {
            if(displaytimer == 0)
            {
                previousstate=2;
                timeout();
            }
        }

        if(hasswitch>0)
        {
            tmpswitch = 0xffff;
            while(tmpswitch != P1IN )
            {
                tmpswitch = P1IN;
                delay2(150);                                            // debounce delay in ms
            }
            switchcnt = 1000;

            hasswitch=0;
            haskey=true;
        } //if(hasswitch>0)

        if(haskey)
        {
            haskey=false;
            displaytimer=90;
            if((SwitchData & BIT0) == 0)                                // minute switch pressed
            {
                SwitchData = P1IN & BIT0;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT0;
                }
                SwitchData=0xff;

                if(switchcnt == 0)                                      // recall scan
                {
                    powerison = 2;
                    powerup =  true;
                    previousstate = 2;
                    goto startup;
                }

                UserMinutes += 15;                                      // add 15 minutes
                if(UserMinutes>90)
                {
                    UserMinutes=0;
                }// wrap around
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT0) == 0)

            if((SwitchData & BIT1) == 0)                                // CHILD switch pressed
            {
                SwitchData = P1IN & BIT1;                               // check for continuous pressing of child switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT1;
                }
                SwitchData=0xff;

                if(switchcnt == 0)                                      // time out? extended mode
                {
                    extmode = true;
                    writechild();
                }
                else
                {
                    if(extmode == true)                                 // in extmode, press child again turn off extmode
                    {
                        extmode = false;
                        writechild();
                    }
                    else
                    {
                        CHILD +=1;
                        if(CHILD >1)
                        {
                            CHILD=0;
                        }
                        writechild();
                    }
                }
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT1) == 0)

            if((SwitchData & BIT2) == 0)                                // Left Up switch pressed (BIT2)
            {
                SwitchData = P1IN & BIT2;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT2;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    ltpoint +=1;
                    if(ltpoint>10)                                      // VER5 init ltpoint to 0
                        ltpoint=0;
                }
                else
                {
                    LUP += 1;
                    if(CHILD==1)
                    {
                        if(LUP>14)
                            LUP=14;
                    }
                    if(CHILD==0)
                    {
                        if(LUP>7)
                            LUP=7;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT2) == 0)

            if((SwitchData & BIT3) == 0)                                // Right Up switch pressed (BIT3)
            {
                SwitchData = P1IN & BIT3;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT3;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    rtpoint +=1;
                    if(rtpoint>10)                                      // VER5 init rtpoint to 0
                        rtpoint=0;
                }
                else
                {
                    RUP += 1;
                    if(CHILD==1)
                    {
                        if(RUP>14)
                            RUP=14;
                    }
                    if(CHILD==0)
                    {
                        if(RUP>7)
                            RUP=7;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT3) == 0)

            if((SwitchData & BIT4) == 0)                                // Left Down switch pressed
            {
                SwitchData = P1IN & BIT4;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT4;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    if(ltpoint>0)                                      // VER5 init ltpoint to 0
                    {
                        ltpoint -=1;
                    }
                }
                else
                {
                    if(LUP > 0)
                    {
                        LUP -= 1;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT4) == 0)

            if((SwitchData & BIT5) == 0)                                // Right Down switch pressed
            {
                SwitchData = P1IN & BIT5;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT5;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    if(rtpoint>0)                                       // VER5 init rtpoint to 0
                    {
                        rtpoint -=1;
                    }
                }
                else
                {
                    if(RUP > 0)
                    {
                        RUP -= 1;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                        // turn on LLED
                beeper();
            } //if((SwitchData & BIT5) == 0)

            if((SwitchData & BIT6) == 0)                                // Power Switch pressed (BIT6)
            {
                SwitchData = P1IN & BIT6;                               // check for continuous pressing of power switch
                while(SwitchData == 0)                                  // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT6;
                }
                SwitchData=0xff;

                if(powerison==2)
                {
                    powerison=1;
                    previousstate=1;
                    timeout();
                }

                if(powerison==1)
                {
                    if(switchcnt == 0)                                  // time out?
                    {
                        powerison = 2;                                  // set to scan mode
                        extmode = false;
                        beeper();
                        blankscreen();

                        diagscreen();
                        displaytimer=120;
                        P3OUT |= (BIT2);                                // turn on LLED
                        scanit = true;
                    }
                    else
                    {
                        previousstate = 1;
                        timeout();
                    }
                } //if(powerison==1)
            } //if((SwitchData & BIT6) == 0)
            SwitchData = 0xff;
            P1IE  = 0x7F;                                               // enable 0-6 inputs interrupt enabled
        } //if(haskey)
    } //while(1)
 } //main

