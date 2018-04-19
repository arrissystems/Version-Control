
#include <msp430.h>

/*
 * main.c
 * Revision 02/26/2018 added haskey to detect keys and added delay to fix key bounce
 * Revision 03/02/2018 changed writeintensity to show Adult double values
 * Revision 03/14/2018 fixed keybounce changed scanhnd to use average as base value
*/

//#include    "msp430f2274.h"
//#include    "msp430f2748.h"
//#include    "msp430fR5848.h"

#include    "msp430G2955.h"
//#include    "msp430G2855.h"

#include <stdint.h>
#include <stdbool.h>

#include    "charmap.h"
#include    "newimmumax2.h"

#define BYTE    unsigned char

void delay(int xdelay);
void pulsexfmr(void);
void RESET (void);
void Setup (void);
void ExitSleep (void);
void EnterSleep (void);
void write_data(unsigned char data1);
void write_command(unsigned char command);
void scanadc(unsigned int INCHAN);
void sdelay(int xdelay);
void sdelay1(int xdelay);
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
bool        haskey;
bool        rdbattery;
int         dlycnt;
int         dlycnt1;
int         dlycnt2;
int         dlycnt3;
int         clockcnt;
bool        callclock;
int         hasswitch;
bool        rewritescreen;
bool        scanit;
unsigned int        RUP;
unsigned int        LUP;
int         CHILD;
int         idata;
long        indata;
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

signed char intCtr;

unsigned char timercounter;

int         leftadcval;
int         rightadcval;
int         leftmax;
int         rightmax;
int         leftbar[11];
int         rightbar[11];
int         leftadcarray[13][5];
int         rightadcarray[13][5];
int         leftarray[13];
int         rightarray[13];
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
int         llastp4;
int         rlastp4;

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

void delay(int xdelay)                                                  // delay for xdelay ms
{
    dlycnt1 = xdelay;                                                   // use timer interrupt
    while(dlycnt1>0);
}

void delay2(int xdelay)                                                 // delay for xdelay ms
{
    dlycnt2 = xdelay;                                                   // use timer interrupt
    while(dlycnt2>0);
}


void sdelay(int xdelay)                                                 // delay for xdelay nops
{
int intii;
    for(intii=xdelay;intii>0;intii--)                                   // use nops
    {
        __no_operation();
    }
}

void sdelay1(int xdelay)                                                // delay for xdelay nops
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
    sdelay(3);
    P2OUT |= (BIT1);                                                    // set WR high P2.1
    sdelay(3);
    P2OUT |= (BIT0);                                                    // /CS high
}

void write_data(unsigned char data1)
{
    P4DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);   // Set P2.0,1,2,3,4,5 to output
    P4OUT = data1;                                                      //output to 8 bit data port
    P2OUT |= (BIT1 + BIT2 + BIT3);                                      //set RD,WR,D/C high
    P2OUT &= ~(BIT0);                                                   // /CS low
    P2OUT &= ~(BIT1);                                                   // set WR low P2.1
    sdelay(3);
    P2OUT |= (BIT1);                                                    // set WR high P2.1
    sdelay(3);
    P2OUT |= (BIT0);                                                    // /CS high

}

void read_data(void)
{
    P4DIR = 0x00;                                                       // set port to input
    P2OUT |= (BIT1 + BIT2);                                             //set RD,WR high
    P2OUT |= (BIT3);                                                    //set D/C high for data
    P2OUT &= ~(BIT0);                                                   // /CS low
    sdelay(2);
    P2OUT &= ~(BIT2);                                                   // set RD low P2.2
    sdelay(10);
    P2OUT |= (BIT2);                                                    // set RD high P2.2
    idata = P4IN;                                                       //input from 8 bit data port
    P2OUT |= (BIT0);                                                    // /CS back high

    P4DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 +BIT6 + BIT7);    // Set P2.0,1,2,3,4,5 to output
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
    delay(1);                                                           //delay 1 ms
    //   res=0;
    P3OUT &= ~(BIT5);                                                   //LCD LRESET Low
    delay(10);                                                          //delay 10 ms
    //   res=1;
    P3OUT |= (BIT5);                                                    //LCD LRESET back high
    delay(120);                                                         //delay 120 ms


//**********************************************************************//LCD SETING
    write_command(0x11);                                                //sleepout command turn off sleep
    delay(120);                                                         //Delay 120ms

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
    delay(100);                                                          // delay 20ms
    write_command(0x10);
    delay(100);                                                          // delay 20ms

}

//*********************************************************
void ExitSleep (void)

{
    write_command(0x11);
    delay(120);                                                         // delay 120ms
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
        if((pointer & 0x01)==0)                                     //skip the extra word from paint conversion
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
    if(powerison==0)
    {
        __bic_SR_register(LPM3_bits);                                   // CPU back on
        P1IFG=0;
        WDTCTL &= WDTHOLD;                                              // Generate a PUC reset
        while(1);

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
    if(dlycnt>0)
    {
        dlycnt-=1;
    }
    if(dlycnt1>0)
    {
        dlycnt1-=1;
    }
    if(dlycnt2>0)
    {
        dlycnt2-=1;
    }
    if(switchcnt>0)
    {
        switchcnt-=1;
    }
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
//    __bic_SR_register_on_exit(CPUOFF);                                // Enable CPU so the main while loop continues

}


int    xchangebits(int ibits)
{
int cii;
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

    iCols+=1;                                                         //write 2nd digit
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
        if (CHILD==1)
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
        if (CHILD==2)
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
    if(CHILD==2)
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
    if(CHILD==2)
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
    sdelay(130);                                                        // 0.6133 us each

    scanadc(INCH_5);                                                    // return value in ADCValue
    sdelay(10);
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
    sdelay(130);                                                        // 0.6133 us each

    scanadc(INCH_6);                                                    // return value in ADCValue
    sdelay(10);
    P2OUT &= ~(BIT5);                                                   // turn RHVTRIG off
    return  ADCValue;
}


void scanhand(void)
{
    unsigned int i, j;

    leftsetvoltage(7);
    delay2(150);
    leftselchan(12);
    delay2(50);
    leftadcval=leftscanchan();
    leftarray[12] = leftadcval;
    delay2(50);
    leftselchan(12);
    delay2(50);
    leftadcval=leftscanchan();
    leftarray[12] += leftadcval;
    leftarray[12] = leftarray[12]>>1;
    delay2(50);

    for(j=0;j<4;j++)
    {
        for(i=0;i<11;i++)
        {
            leftselchan(i);
            delay2(50);
            leftadcarray[i][j]=0;
            leftadcval = leftscanchan();
            leftadcarray[i][j] = leftadcval;
            delay2(50);
        }
    }

    leftmax=0;
    leftmin=2048;
    leftave=0;
    for(i=0;i<11;i++)
    {
        leftarray[i]=0;
        for(j=0;j<4;j++)
        {
            leftarray[i]+=leftadcarray[i][j];
        }
        leftarray[i] = leftarray[i]>>2;
        leftave+=leftarray[i];
        if(leftmax < leftarray[i])
            leftmax = leftarray[i];
        if(leftmin > leftarray[i])
            leftmin=leftarray[i];
    }
    leftave=leftave/11+1;

    // scan right handpiece next

    rightsetvoltage(7);
    delay2(150);
    rightselchan(12);
    delay2(50);
    rightadcval=rightscanchan();
    delay2(50);
    rightselchan(12);
    delay2(50);
    rightadcval=rightscanchan();
    rightarray[12] += rightadcval;
    rightarray[12] = rightarray[12]>>1;
    delay2(50);

    for(j=0;j<4;j++)
    {
        for(i=0;i<11;i++)
        {
            rightselchan(i);
            delay2(50);
            rightadcarray[i][j]=0;
            rightadcval = rightscanchan();
            rightadcarray[i][j] = rightadcval;
            delay2(50);
        }
    }

    rightmax=0;
    rightmin=2048;
    rightave=0;
    for(i=0;i<11;i++)
    {
        rightarray[i]=0;
        for(j=0;j<4;j++)
        {
            rightarray[i]+=rightadcarray[i][j];
        }
        rightarray[i] = rightarray[i]>>2;
        rightave+=rightarray[i];
        if(rightmax < rightarray[i])
            rightmax = rightarray[i];
        if(rightmin > rightarray[i])
            rightmin=rightarray[i];
    }
    rightave=rightave/11+1;
    voltageoff();

    leftrange = leftmax-leftmin;
    iPixC = 16;                                                         //16 pixels horizontal
    iPixR = 20;                                                         //20 pixels vertical

    iRows = 10;                                                         //display left hand intensity
    iCols = 1;
    write3digits(leftrange, 0xff, 0, 0xff);

    if(leftrange < 7)
    {
        leftrange=7;
    }

    rightrange = rightmax-rightmin;
    iRows = 10;                                                         //display right hand intensity
    iCols = 17;
    write3digits(rightrange, 0xff, 0x7f, 0);

    if(rightrange < 7)
    {
        rightrange = 7;
    }

    for (i=0;i<11;++i)                                                  //get values from adc
    {
        ltmp=leftarray[i]-leftave;

        if(ltmp > 0)
        {
            ltmp=leftarray[i]-leftmin;                                          //-leftmin;
        }
        else
        {
            ltmp=0;
        }
        leftbar[i]=ltmp*7/leftrange;

        ltmp=rightarray[i]-rightave;

        if(ltmp > 0)
        {
            ltmp=rightarray[i]-rightmin;                                         //-rightmin;
        }
        else
        {
            ltmp=0;
        }
        rightbar[i]=ltmp*7/rightrange;

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
    P1IE  = 0x7F;                                                       // 0-6 inputs interrupt
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
    dlycnt=0;
    dlycnt1=0;
    dlycnt2=0;
    clockcnt=512;
    callclock=false;
    hasswitch=0;                                                        // switches pressed counter
    haskey=false;
    CHILD=1;
    LUP=0;                                                              // for testing only, set to 0 for normal
    RUP=0;                                                              // for testing only, set to 0 for normal
    rewritescreen=true;
    UserMinutes=0;                                                      // Start with 30 minutes for testing only, set to 0 for normal
    ADCDone=true;
    rdbattery=true;
    p1iflag=0;
    beeperon=false;
    displaytimer=90;
    switchcnt=0;
    ltpoint=1;
    rtpoint=1;
    extmode=false;
    leftselchan(ltpoint);                                               // P3 and P4 to set TP end with P4=0
    rightselchan(rtpoint);
    scanit = false;

    return 0;
}


int main(void)
{
    unsigned int i, j;
    int     tmp;
    int     tmpswitch;
    unsigned long intii;
    unsigned int pointer;
    char LabelAry[8];

startup:
    WDTCTL = WDTPW | WDTHOLD;                                           // stop watchdog timer
    if(Init()>0)return 1;
    _enable_interrupt();

    initi();                                                            // initial LCD screen

    while(1)
    {
        if(powerison==1)
        {
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
                if(callclock)                                           // 0.5 sec
                {
                    if(LUP > 0)
                    {
                        leftsetvoltage(LUP);
                    }
                    if(RUP > 0)
                    {
                        rightsetvoltage(RUP);
                    }
                    if((LUP>0) || (RUP>0))
                    {
                        pulsexfmr();                                    // P2.4,5 to strobe LHVTRIG and HVTRIG
                        voltageoff();
                        callclock=false;

                    }
                }
                displaytimer = 90;
            }

            if(UserMinutes == MIN)
            {
                MIN = UserMinutes;
                SEC = 0;
                if(displaytimer==0)
                {
                    llastp4 = 0;
                    rlastp4 = 0;
                    powerison=0;                                        // power off mode
                    voltageoff();
                    beeper();
                    EnterSleep();                                       // display OFF
                    P1IE  = 0x7F;                                       // 0-6 inputs interrupt
                    __bis_SR_register(LPM3_bits+GIE);                   // enter LPM3 mode
                    while(1);
                }
            }

            if(rdbattery)                                               // read the battery value
            {
                scanadc(INCH_7);                                        // return value in ADCValue
                BATValue = ADCValue;                                    // save value in BATValue
            }

            if(rewritescreen==true)
            {
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
                write_command(0x29);                                    //display on
                write_command(0x2C);                                    //RAM write control

                pointer = 9599;

                for(intii=9600;intii>0;intii--)                         // 16 character 20 pixels= 320 columns
                {
                    tmp = newfacemono99[pointer] & 0x01;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x02;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x04;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x08;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x10;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x20;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x40;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    tmp = newfacemono99[pointer] & 0x80;                // one bit per pixel
                    writemonochar(tmp, 0xff, 0xff, 0x00);               // blue color
                    pointer-=1;

                }

                write_command(0x29);                                    //display on


                //write static texts on the LCD

                iPixC = 16;                                             //16 pixels horizontal per char
                iPixR = 20;                                             //20 pixels vertical per char

                iRows=7;
                iCols=13;
                ichar = 'M';
                charAll(0xff, 0xff, 0xff);                              //black color

                iCols++;
                ichar = 'I';
                charAll(0xff, 0xff, 0xff);                              //black color

                iCols++;
                ichar = 'N';
                charAll(0xff, 0xff, 0xff);                              //black color

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
                    charAll(0xff, 0xff, 0xff);                          //black color
                };

                iRows=1;
                iCols=14;
                ichar = '%';
                charAll(0xff, 0xff, 0xff);                              //black color

                displaytimer=90;                                        //90 sec LED on
                write_command(0x29);                                    //display on
                P3OUT |= (BIT2);                                        //turn on LLED
                rewritescreen=false;
                beeper();
            }

            // write left treatment point to LCD
            iPixC = 16;                                                 //16 pixels horizontal per char
            iPixR = 20;                                                 //20 pixels vertical per char

            iRows=4;                                                    //write ms digit of minute
            iCols=7;
            writedigits(ltpoint,0,0xff,0xff);                           //red color

            // write right treatment point to LCD
            iPixC = 16;                                                 //16 pixels horizontal per char
            iPixR = 20;                                                 //20 pixels vertical per char

            iRows=4;                                                    //write ms digit of minute
            iCols=15;
            writedigits(rtpoint,0,0xff,0xff);                           //red color


            // write running time to LCD
            iPixC = 16;                                                 //16 pixels horizontal per char
            iPixR = 20;                                                 //20 pixels vertical per char

            iRows=6;                                                    //write ms digit of minute
            iCols=10;
            writedigits(MIN,0,0xff,0xff);                               //red color

            iCols+=1;                                                   //write : after minute
            ichar = ':';
            charAll(0xff, 0xff, 0xff);                                  //black color

            iCols+=1;                                                   //write ms digit of second
            writedigits(SEC,0,0xff,0xff);                               //red color

            // write user selected time to LCD
            iPixC = 16;                                                 //16 pixels horizontal
            iPixR = 20;                                                 //20 pixels vertical

            iRows=7;                                                    //write ms digit of minute
            iCols=10;
            writedigits(UserMinutes,0xff,0xff,0xff);                    //black color

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

            iRows=1;                                                    //write to LCD
            iCols=11;
            if(BatPcnt < 10)
            {
                writedigits(BatPcnt,0,0xff,0xff);                       //red color
            }
            else
            {
                writedigits(BatPcnt,0xff,0xff,0);                       //green color
            }
        } //if(powerison==1)

        if(scanit == true)
        {
            scanit = false;

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

            scanhand();

            iPixC = 16;                                                 //16 pixels horizontal
            iPixR = 20;                                                 //20 pixels vertical

            ichar='Z';
            for (i=0;i<11;i++)                                          //display left hand scan result
            {
                iRows=10-i;
                ichar=' ';
                iCols=2;
                for (j=0;j<15;j++)
                {
                    charAll(0xff, 0xff, 0xff);
                    iCols+=1;
                }
                ichar='Z';
                iCols=2;
                for (j=0;j<leftbar[i];j++)
                {
                    charAll(0xff, 0xff, 0xff);
                    iCols+=1;
                }
            }

            for (i=0;i<11;i++)                                          //display right hand scan result
            {
                iRows=10-i;
                ichar=' ';
                iCols=8;                                               //last col is 18
                for (j=0;j<7;j++)
                {
                    charAll(0xff, 0xff, 0xff);
                    iCols+=1;
                }
                ichar='Z';
                iCols=17-rightbar[i];
                for (j=0;j<rightbar[i];j++)
                {
                    charAll(0xff, 0xff, 0xff);
                    iCols+=1;
                }
            }
            displaytimer = 180;
        } //if(scanit == true)


        if(hasswitch>0)
        {
            tmpswitch = 0xffff;
            while(tmpswitch != P1IN )
            {
                tmpswitch = P1IN;
                delay2(50);                                             // debounce delay
            }
//            while(P1IN != 0x7f)
//            {
//                delay2(50);
//            }
            switchcnt = 1000;

            hasswitch=0;
            haskey=true;
        } //if(hasswitch>0)

        if(haskey)
        {
            haskey=false;
            displaytimer=90;
            if((SwitchData & BIT0) == 0)                            // minute switch pressed
            {
                SwitchData = P1IN & BIT0;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT0;
                }
                SwitchData=0xff;

                UserMinutes += 15;                                  // add 15 minutes
                if(UserMinutes>90)
                {
                    UserMinutes=0;
                }// wrap around
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT0) == 0)

            if((SwitchData & BIT1) == 0)                            // CHILD switch pressed
            {
                SwitchData = P1IN & BIT1;                           // check for continuous pressing of child switch
                while(SwitchData == 0)                              // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT1;
                }
                SwitchData=0xff;

                if(switchcnt == 0)                                  // time out? extended mode
                {
                    extmode = true;
                    writechild();
                }
                else
                {
                    if(extmode == true)                             // in extmode, press child again turn off extmode
                    {
                        extmode = false;
                        writechild();
                    }
                    else
                    {
                        CHILD +=1;
                        if(CHILD >2)
                        {
                            CHILD=1;
                        }
                        writechild();
                    }
                }
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT1) == 0)

            if((SwitchData & BIT2) == 0)                            // Left Up switch pressed (BIT2)
            {
                SwitchData = P1IN & BIT2;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT2;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    ltpoint +=1;
                    if(ltpoint>11)
                        ltpoint=1;
                    leftselchan(ltpoint);
                }
                else
                {
                    LUP += 1;
                    if(CHILD==2)
                    {
                        if(LUP>14)
                            LUP=14;
                    }
                    if(CHILD==1)
                    {
                        if(LUP>7)
                            LUP=7;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT2) == 0)

            if((SwitchData & BIT3) == 0)                            // Right Up switch pressed (BIT3)
            {
                SwitchData = P1IN & BIT3;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT3;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    rtpoint +=1;
                    if(rtpoint>11)
                        rtpoint=1;
                    rightselchan(rtpoint);
                }
                else
                {
                    RUP += 1;
                    if(CHILD==2)
                    {
                        if(RUP>14)
                            RUP=14;
                    }
                    if(CHILD==1)
                    {
                        if(RUP>7)
                            RUP=7;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT3) == 0)

            if((SwitchData & BIT4) == 0)                            // Left Down switch pressed
            {
                SwitchData = P1IN & BIT4;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT4;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    if(ltpoint>1)
                    {
                        ltpoint -=1;
                    }
                    leftselchan(ltpoint);
                }
                else
                {
                    if(LUP > 0)
                    {
                        LUP -= 1;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT4) == 0)

            if((SwitchData & BIT5) == 0)                            // Right Down switch pressed
            {
                SwitchData = P1IN & BIT5;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT5;
                }
                SwitchData=0xff;

                if(extmode == true)
                {
                    if(rtpoint>1)
                    {
                        rtpoint -=1;
                    }
                    rightselchan(rtpoint);
                }
                else
                {
                    if(RUP > 0)
                    {
                        RUP -= 1;
                    }
                    writeintensity();
                }
                P3OUT |= (BIT2);                                    // turn on LLED
                beeper();
            } //if((SwitchData & BIT5) == 0)

            if((SwitchData & BIT6) == 0)                            // Power Switch pressed (BIT6)
            {
                SwitchData = P1IN & BIT6;                       // check for continuous pressing of power switch
                while(SwitchData == 0)                          // wait for switch to go back up
                {
                    SwitchData = P1IN & BIT6;
                }
                SwitchData=0xff;

                if(powerison==0)                                    // was in power down mode, so restart
                {
//                    beeper();
                    delay(500);
                    goto startup;
                }

                if(powerison==2)
                {
                    llastp4 = 0;
                    rlastp4 = 0;
                    powerison=0;                                    // power off mode
                    UserMinutes = 0;
                    voltageoff();
                    beeper();
                    EnterSleep();                               // display OFF
                    P1IE  = 0x7F;                                   // 0-6 inputs interrupt
                    __bis_SR_register(LPM3_bits+GIE);               // enter LPM3 mode
                    while(1);
                }

                if(powerison==1)
                {
                    if(switchcnt == 0)                              // time out?
                    {
                        powerison = 2;                              // set to scan mode
                        extmode = false;
                        beeper();
                        blankscreen();

                        // this section is to display diagnostic page
                        iPixC = 16;                                 //16 pixels horizontal
                        iPixR = 20;                                 //20 pixels vertical

                        for (i=11;i>0;--i)                          //display 1 to 11 number at the left side
                        {
                            iRows=11-i;
                            iCols=0;
                            if (i>9)                                //if it is a 2 digit number
                            {
                                ichar=digits[1];                    //write ms digit
                                charAll(0xff, 0xff, 0xff);
                                iCols +=1;                          //write ls digit
                                ichar=digits[i-10];
                                charAll(0xff, 0xff, 0xff);
                            }
                            else                                    //only one digit
                            {
                                iCols +=1;
                                ichar=digits[i];
                                charAll(0xff, 0xff, 0xff);
                            }
                        }

                        for (i=11;i>0;--i)                          //display 1 to 11 number at the right side
                        {
                            iRows=11-i;
                            iCols=17;
                            if (i>9)                                //if it is a 2 digit number
                            {
                                ichar=digits[1];                    //write ms digit
                                charAll(0xff, 0xff, 0xff);
                                iCols +=1;                          //write ls digit
                                ichar=digits[i-10];
                                charAll(0xff, 0xff, 0xff);
                            }
                            else                                    //only one digit
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

                        displaytimer=180;
                        P3OUT |= (BIT2);                                // turn on LLED
                        scanit = true;
                    }
                    else
                    {
                        llastp4 = 0;
                        rlastp4 = 0;
                        powerison=0;                                    // power off mode
                        voltageoff();
                        beeper();
                        EnterSleep();                                   // display OFF
                        P1IE  = 0x7F;                                   // 0-6 inputs interrupt
                        __bis_SR_register(LPM3_bits+GIE);               // enter LPM3 mode
                        while(1);
                    }
                } //if(powerison==1)
            } //if((SwitchData & BIT6) == 0)
            SwitchData = 0xff;
            P1IE  = 0x7F;                                               // enable 0-6 inputs interrupt
        } //if(haskey)
    } //while(1)
 } //main

