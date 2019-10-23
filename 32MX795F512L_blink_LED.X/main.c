/*
 * File:   main.c
 * Target: PIC32MX795F512L
 * Compiler: C32 v2.02, XC32 v1.44
 *
 * Description:
 *  Blink LEDs on PORTF bits RF2, RF4, RF8
 *
 * Notes:
 *  Define symbol USE_XTAL to use an external 8.0MHz crystal.
 *  Define symbol USE_PLL to enable the PLL and set clock to 80MHz.
 *  Does not use the PLIB library so should build with any Microchip compiler for PIC32.
 *  When building with XC32 v1.44 MPLAB v8.92 asserts an error dialog box.
 *
 *
 *                                                        PIC32MX795F512L
 *               +-----------+               +-----------+               +-----------+               +-----------+
 *        <*>  1 : RG15      :    PGC < > 26 : RB6/PGEC2 :        <*> 51 : RF3/USBID :        <*> 76 : RD1       :
 *    3v3 -->  2 : VDD       :    PGD < > 27 : RB7/PGED2 :   U1RX <*> 52 : RF2/U1RX  :        <*> 77 : RD2       :
 * LED_D5 <*>  3 : RE5       :        < > 28 : RA9       :   U1TX <*> 53 : RF8/U1TX  :        <*> 78 : RD3       :
 * LED_D6 <*>  4 : RE6       :        < > 29 : RA10      :   VBUS -*> 54 : VBUS      :        <*> 79 : RD12      :
 * LED_D7 <*>  5 : RE7       :    3v3 --> 30 : AVDD      :    3v3 --> 55 : VUSB3V3   :        <*> 80 : RD13      :
 *        <*>  6 : RC1       :    GND --> 31 : AVSS      :     D- < > 56 : RG3/D-    :        <*> 81 : RD4       :
 *        <*>  7 : RC2       :        < > 32 : RB8       :     D+ < > 57 : RG2/D+    :        <*> 82 : RD5       :
 *        <*>  8 : RC3       :        < > 33 : RB9       :        <*> 58 : RA2       :        <*> 83 : RD6       :
 *        <*>  9 : RC4       :        < > 34 : RB10      :        <*> 59 : RA3       :        <*> 84 : RD7       :
 *        <*> 10 : RG6       :        < > 35 : RB11      :        <*> 60 : RA4       :   10uF --> 85 : VCAP      :
 *        <*> 11 : RG7       :    GND --> 36 : VSS       :        <*> 61 : RA5       :    3v3 --> 86 : VDD       :
 *        <*> 12 : RG8       :    3v3 --> 37 : VDD       :    3v3 --> 62 : VDD       :        <*> 87 : RF0       :
 *    VPP -*> 13 : MCLR      :        <*> 38 : RA1       :        < > 63 : RC12/OSC1 :        <*> 88 : RF1       :
 *        <*> 14 : RG9       :        <*> 39 : RF13      :        < > 64 : RC15/OSC2 :        <*> 89 : RG1       :
 *    GND --> 15 : VSS       :        <*> 40 : RF12      :    GND --> 65 : VSS       :        <*> 90 : RG0       :
 *    3v3 --> 16 : VDD       :        < > 41 : RB12      :        <*> 66 : RA14      :        <*> 91 : RA6       :
 *        <*> 17 : RA0       :        < > 42 : RB13      :        <*> 67 : RA15      :        <*> 92 : RA7       :
 *        <*> 18 : RE8       :        < > 43 : RB14      :        <*> 68 : RD8       : LED_D0 <*> 93 : RE0       :
 *        <*> 19 : RE9       :        < > 44 : RB15      :        <*> 69 : RD9       : LED_D1 <*> 94 : RE1       :
 *        < > 20 : RB5       :    GND --> 45 : VSS       :        <*> 70 : RD10      :        <*> 95 : RG14      :
 *        < > 21 : RB4       :    3v3 --> 46 : VDD       :        <*> 71 : RD11      :        <*> 96 : RG12      :
 *        < > 22 : RB3       :        <*> 47 : RD14      :        <*> 72 : RD0       :        <*> 97 : RG13      :
 *        < > 23 : RB2       :        <*> 48 : RD15      :        < > 73 : RC13/SOSCi: LED_D2 <*> 98 : RE2       :
 * LED_G0 < > 24 : RB1/PGEC1 :        <*> 49 : RF4       :        < > 74 : RC14/SOSCo: LED_D3 <*> 99 : RE3       :
 * LED_G1 < > 25 : RB0/PGED1 : EN_5VO <*> 50 : RF5       :    GND --> 75 : VSS       : LED_D4 <*>100 : RE4       :
 *               +-----------+               +-----------+               +-----------+               +-----------+
 *        <*> = 5.0 volt tolerant input                      TQFP-100
 *
 *
 */
/*
 * Configuration words
 */
#pragma config FSRSSEL = PRIORITY_7     /* SRS Select (SRS Priority 7) */
#pragma config FMIIEN = OFF             /* Ethernet RMII/MII Enable (RMII Enabled) */
#pragma config FETHIO = OFF             /* Ethernet I/O Pin Select (Alternate Ethernet I/O) */
#pragma config FCANIO = OFF             /* CAN I/O Pin Select (Alternate CAN I/O) */
#pragma config FUSBIDIO = OFF           /* USB USID Selection (Controlled by Port Function) */
#pragma config FVBUSONIO = OFF          /* USB VBUS ON Selection (Controlled by Port Function) */
#pragma config FPLLIDIV = DIV_2         /* PLL Input Divider (2x Divider) */
#pragma config FPLLMUL = MUL_20         /* PLL Multiplier (20x Multiplier) */
#pragma config UPLLIDIV = DIV_2         /* USB PLL Input Divider (2x Divider) */
#pragma config UPLLEN = OFF             /* USB PLL Enable (Disabled and Bypassed) */
#pragma config FPLLODIV = DIV_1         /* System PLL Output Clock Divider (PLL Divide by 1) */
#pragma config FSOSCEN = OFF            /* Secondary Oscillator Enable (Disabled) */
#pragma config IESO = OFF               /* Internal/External Switch Over (Disabled) */
#pragma config OSCIOFNC = OFF           /* CLKO Output Signal Active on the OSCO Pin (Disabled) */
#pragma config FPBDIV = DIV_1           /* Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1) */
#pragma config WDTPS = PS1              /* Watchdog Timer Postscaler (1:1) */
#pragma config FWDTEN = OFF             /* Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls)) */
#pragma config ICESEL = ICS_PGx2        /* ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2) */
#pragma config PWP = OFF                /* Program Flash Write Protect (Disable) */
#pragma config BWP = OFF                /* Boot Flash Write Protect bit (Protection Disabled) */
#pragma config CP = OFF                 /* Code Protect (Protection Disabled) */
#pragma config FNOSC = FRC              /* Oscillator Selection Bits (Fast RC Osc) */
#pragma config FCKSM = CSECMD           /* Clock Switching and Monitor Selection (Clock Switch Enable, FSCM Disabled) */

#if defined(USE_XTAL)

#pragma config POSCMOD = XT             /* Primary Oscillator Configuration (XT osc mode) */
#define OSC_FREQUENCY       (8000000ul) /* External crystal frequency */
#define PLL_INPUT_DIVIDER   (2ul)
#define PLL_MULTIPLIER      (20ul)

#if defined(USE_PLL)
#define OSC_SELECTOR        (0b011)     /* Use Primary Oscillator + PLL*/
#else
#define OSC_SELECTOR        (0b010)     /* Use Primary Oscillator */
#endif

#else

#pragma config POSCMOD = OFF            /* Primary Oscillator Configuration (Primary osc disabled) */
#define OSC_FREQUENCY       (8000000ul) /* Internal Fast RC oscillator frequency */
#define PLL_INPUT_DIVIDER   (2ul)
#define PLL_MULTIPLIER      (20ul)

#if defined(USE_PLL)
#define OSC_SELECTOR        (0b001)     /* Use FRC + PLL */
#else
#define OSC_SELECTOR        (0b000)     /* Use FRC */
#endif

#endif

#if defined(__XC32)
#include <xc.h>
#elif defined(__C32__)
#include <p32xxxx.h>
#else
#error "Unknown PIC32 compiler"
#endif



#if defined(USE_PLL)
#define GetSystemClock()        ((OSC_FREQUENCY/PLL_INPUT_DIVIDER)*PLL_MULTIPLIER)
#define PB_CLOCK_DIVIDER        (1ul)
#else
#define GetSystemClock()        (OSC_FREQUENCY)
#define PB_CLOCK_DIVIDER        (1ul)
#endif
#define GetPeripheralClock()    (GetSystemClock()/PB_CLOCK_DIVIDER)
#define GetInstructionClock()   (GetSystemClock())


#define FLASH_SPEED_HZ          30000000ul /* Max Flash speed */
#define PB_BUS_MAX_FREQ_HZ      80000000ul /* Max Peripheral bus speed */

/***************************************************************
 * <combine with CHE_CONF_PF_ALL, CHE_CONF_PF_C, CHE_CONF_PF_NC>
 *
 * Flash Prefetch option - Values are mutually exclusive
 **************************************************************
 */
#define CHE_CONF_PF_DISABLE  (0 << _CHECON_PREFEN_POSITION)
#define CHE_CONF_PF_C        (1 << _CHECON_PREFEN_POSITION )
#define CHE_CONF_PF_NC       (2 << _CHECON_PREFEN_POSITION)
#define CHE_CONF_PF_ALL      (3 << _CHECON_PREFEN_POSITION)

/* Initialize this PIC */
void __attribute__ ((nomips16)) PIC_Init(void)
{
    /* Configure the device for maximum performance, */
    /* This function will change the program Flash wait states, */
    /* RAM wait state and enable prefetch cache */
    /* The PBDIV value must be already set via the pragma FPBDIV option in configuration words. */

    unsigned int sys_clock;
    unsigned int pb_clock;
    unsigned int int_status;
    unsigned int wait_states;
    register unsigned long cache_tmp;
    register unsigned long clock_switch_timeout;

    sys_clock = GetSystemClock();

    /* Save current interrupt enable state and disable interrupts */
    asm volatile("di    %0" : "=r"(int_status));

    /* Check that we can change the system clock frequency */
    if(!(OSCCONbits.CLKLOCK))
    {
        /* swich system clock to FRC, no PLL */
        SYSKEY = 0, SYSKEY = 0xAA996655, SYSKEY = 0x556699AA;
        OSCCONbits.NOSC = 0b000; /* select FRC as system oscillator */
        OSCCONbits.OSWEN = 1;    /* start a clock switch */
        SYSKEY = 0x33333333;
        for(clock_switch_timeout=80000; clock_switch_timeout; clock_switch_timeout--)
        {
            if(!(OSCCONbits.OSWEN)) break;
        }
    }
    /* Disable the JTAG interface */
    DDPCONbits.JTAGEN = 0;

    /* Disable the JTAG TDO output */
    DDPCONbits.TDOEN = 0;

    /* Disable the TRACE interface */
    DDPCONbits.TROEN = 0;

    /* Set RAM to use MAX wait states */
    BMXCONCLR = _BMXCON_BMXWSDRM_MASK;

    /* Set FLASH wait states for desired system clock */
    wait_states = 0;
    while(sys_clock > (FLASH_SPEED_HZ * (wait_states + 1)))
    {
        wait_states++;
    }

    /* turn on cache and set FLASH wait states */
    CHECON = CHE_CONF_PF_ALL | wait_states;

    /* Select Kseg0 coherency algorithm as 3, cacheable. */
    asm("mfc0 %0,$16,0" :  "=r"(cache_tmp));
    cache_tmp = (cache_tmp & ~7) | 3;
    asm("mtc0 %0,$16,0" :: "r" (cache_tmp));

    /* compute peripheral bus clock frequency from sys_clock */
    pb_clock = (sys_clock >> OSCCONbits.PBDIV);

    if(!OSCCONbits.CLKLOCK)
    {
        /* switch system clock to Primary oscillation amplifier with PLL */
        SYSKEY = 0, SYSKEY = 0xAA996655, SYSKEY = 0x556699AA;
#if   (PLL_INPUT_DIVIDER == 1)
        OSCCONbits.FRCDIV =0b000;
#elif (PLL_INPUT_DIVIDER == 2)
        OSCCONbits.FRCDIV =0b001;
#elif (PLL_INPUT_DIVIDER == 4)
        OSCCONbits.FRCDIV =0b010;
#elif (PLL_INPUT_DIVIDER == 8)
        OSCCONbits.FRCDIV =0b011;
#elif (PLL_INPUT_DIVIDER == 16)
        OSCCONbits.FRCDIV =0b100;
#elif (PLL_INPUT_DIVIDER == 32)
        OSCCONbits.FRCDIV =0b101;
#elif (PLL_INPUT_DIVIDER == 64)
        OSCCONbits.FRCDIV =0b110;
#elif (PLL_INPUT_DIVIDER == 256)
        OSCCONbits.FRCDIV =0b111;
#else
#error PLL input divider has bad value
#endif

#if   (PLL_MULTIPLIER == 15)
        OSCCONbits.PLLMULT =0b000;
#elif (PLL_MULTIPLIER == 16)
        OSCCONbits.PLLMULT =0b001;
#elif (PLL_MULTIPLIER == 17)
        OSCCONbits.PLLMULT =0b010;
#elif (PLL_MULTIPLIER == 18)
        OSCCONbits.PLLMULT =0b011;
#elif (PLL_MULTIPLIER == 19)
        OSCCONbits.PLLMULT =0b100;
#elif (PLL_MULTIPLIER == 20)
        OSCCONbits.PLLMULT =0b101;
#elif (PLL_MULTIPLIER == 21)
        OSCCONbits.PLLMULT =0b110;
#elif (PLL_MULTIPLIER == 24)
        OSCCONbits.PLLMULT =0b111;
#else
#error PLL multiplier has bad value
#endif
        OSCCONbits.PLLODIV = 0b000; /* set PLL output divider to 1:1 */
        OSCCONbits.NOSC = OSC_SELECTOR;
        OSCCONbits.OSWEN = 1;       /* start a clock switch */
        SYSKEY = 0x33333333;
        for(clock_switch_timeout=80000; clock_switch_timeout; clock_switch_timeout--)
        {
            if(!OSCCONbits.OSWEN) break;
        }
    }

    /* set GPIO pins for digital I/O */
    AD1PCFG = 0xFFFFUL;

    /* Restore interrupt state */
    if(int_status & 0x00000001)
    {
       asm volatile("ei    %0" : "=r"(int_status));
    }
}
/*
 * This function is a blocking spin wait used to burn up exection cysles.
 *
 * The delay is in microsecond units with a range from 0 to 107374182.
 * 
 * The maximum delay is 107.3741824 seconds with an 80MHz system clock.
 * Slower system clocks will support a longer maximum delay.
 * Very short delays, less than about 25us, are inaccurate.
 */
void DelayUS( unsigned long Delay )
{
#define TICKS_IN_ONE_MICROSECOND (GetSystemClock() / 2000000ul)
    unsigned long Time0, Time1;

    Time0 = _CP0_GET_COUNT(); /* 2 system clocks per count */

    if (Delay < (unsigned long)(0xFFFFFFFF) / TICKS_IN_ONE_MICROSECOND)
        Delay = Delay * TICKS_IN_ONE_MICROSECOND;
    else
        Delay = (unsigned long)(0xFFFFFFFF);

    for(;;)
    {
        Time1 = _CP0_GET_COUNT();
        Time1 = Time1 - Time0;      /* Get cycle from start of spin */
        if (Time1 >= Delay)
            break;
    }
}
/*
 * Main application
 */
#define LED_D0          LATEbits.LATE0

#define LED_D0_DIR      TRISEbits.TRISE0

void main(void)
{
    PIC_Init();

    LED_D0_DIR = 0;

    while(1)
    {
        LED_D0   = 0;
        DelayUS(500000);

        LED_D0   = 1;
        DelayUS(500000);
    }
}
