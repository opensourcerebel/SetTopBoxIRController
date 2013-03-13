#include <plib.h>
#include <stdio.h>
#include <stdlib.h>

// Configuration Bit settings
//
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 80 MHz
// WDT OFF
#ifndef OVERRIDE_CONFIG_BITS
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider

//oscillator
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor

//#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = ON            // Secondary Oscillator Enable (KLO was off)

//WDT
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale

//flash features
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#endif // OVERRIDE_CONFIG_BITS

#define SYS_CLOCK 80000000
#ifndef SYS_CLOCK
#error "Define SYS_CLOCK (ex. -DSYS_CLOCK=80000000) on compiler command line"
#endif
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK) // FPBDIV = DIV_1
#define GetInstructionClock()       (SYS_CLOCK)

#define mLED_1              LATBbits.LATB15
#define mLED_2              LATBbits.LATB12
#define mLED_3              LATBbits.LATB13

BOOL stateDownActive = 0;
LONG upTime = 0;
int headerCounter = 17;
int cmdCounter = 15;

int headerByte = 0;
int commandByte = 0;

int portD = 0;
volatile LONG irDurationCounter10us = 0;
volatile BOOL justWoke = FALSE;
volatile BOOL processIRCommand = 0;

BOOL irStartOK = FALSE;
BOOL irInitialOK = FALSE;
BOOL irFillerOK = FALSE;

unsigned short int channel4;

#define TRESHOLD_OFF  167 //530mA - usually 480
#define TRESHOLD_ON  167 //530mA - usually  580

#define runningAverageSampleSize  16
static int runningAverageSamples[runningAverageSampleSize]; // LastMeasurements
static int indexAverage = 0;
static int sumAverage = 0;
static BOOL filledAverageActive = FALSE;

#define	IR_TRANSMSIT_BUFFER_SIZE 71
//16 bits, unsigned int
unsigned short int irTransmitBuffer[IR_TRANSMSIT_BUFFER_SIZE];
unsigned char irTransmitBufferIndex = 0;
BOOL sendingIR = 0;

#define	INITIAL	 2859 // 9 000 uS
#define	BEGIN	 1375 // 4 400 uS
#define	FILL	12800 //41 000 uS
#define	END	  700 // 2 250 uS

#define	ZERO	200
#define	ONE	500

#define	TERMINATOR	175

#define IR_HDR_HANSUN 0b0000000011111110
#define	IR_CMD_OFF	        20655
#define	IR_CMD_AUTO_PRG_UP	10965
#define	IR_CMD_AUTO_PRG_DOWN    59925

#define IR_HDR_LED 0b0000000011110111
#define	IR_CMD_PLAY      61965
#define	IR_CMD_STOP      29325
#define	IR_CMD_FWD_LEFT  45645
#define	IR_CMD_FWD_RIGHT 13005

#define	IR_CMD_LED_ON     49215
#define	IR_CMD_LED_OFF    16575
#define	IR_CMD_LED_CLR1   57375
#define	IR_CMD_LED_CLR2   4335

#define	IR_CMD_PRG_UP	63495
#define	IR_CMD_PRG_DOWN	15045


#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define MAX_DUTY               2001

volatile BOOL stbTVRunning = FALSE;

volatile BOOL pirActive = FALSE;
volatile BOOL stbActive = FALSE;
volatile BOOL stbActivationInProgress = FALSE;
volatile BOOL upCommandActive = FALSE;
volatile BOOL downCommandActive = FALSE;

void initUART(void);
void writeString(const char *string);
void putCharacter(const char character);

void dbg(const char *string);
void dbgp(const char *string, int data);

void initUART(void)
{
    PORTFbits.RF4 = 1; //U2RX
    PORTFbits.RF5 = 0; //U2TX
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, 0);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, SYS_CLOCK, 115200);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}

void setupTimer2()
{
    //timer2 source is internal => on 80Mhz that is 12,5 for one click
    //multiply by 8 => timer 2 clicks on 100 nS
    //timer 2 counts 100 clicks and then interrupts on every 100nS * 100 = 10uS)
    OpenTimer2(T2_OFF | T2_SOURCE_INT | T2_PS_1_8, 100);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
}

void printDebug(const char *string)
{
    //printf(string);
}

void processSignal(int value)
{
    if (value)
    {
        printDebug("1");
    }
    else
    {
        printDebug("0");
    }
    //process signal
    if (headerCounter == 0)
    {
        printDebug(" ");
    }

    if (headerCounter <= 15 && headerCounter >= 0)
    {
        headerByte |= value << headerCounter;
    }
    else if (headerCounter < 0 && cmdCounter >= 0)
    {
        commandByte |= value << cmdCounter;

        cmdCounter--;
    }
    headerCounter--;
}

void resetIRHolders()
{
    irStartOK = FALSE;
    irInitialOK = FALSE;
    irFillerOK = FALSE;
    headerCounter = 15;
    cmdCounter = 15;
    headerByte = 0;
    commandByte = 0;
}

void switchMonitorOff()
{
    mPORTBClearBits(BIT_14);
}

void switchMonitorOn()
{
    mPORTBSetBits(BIT_14);
}

BOOL checkIRCommand(int cmd, int hdr)
{
    if (commandByte == cmd && headerByte == hdr)
    {
        return TRUE;
    }

    return FALSE;
}

BOOL upAutoSent()
{
    return checkIRCommand(IR_CMD_AUTO_PRG_UP, IR_HDR_HANSUN);
}

BOOL downAutoSent()
{
    return checkIRCommand(IR_CMD_AUTO_PRG_DOWN, IR_HDR_HANSUN);
}

BOOL ledOnSent()
{
    return checkIRCommand(IR_CMD_PLAY, IR_HDR_HANSUN);
}

BOOL ledOffSent()
{
    return checkIRCommand(IR_CMD_STOP, IR_HDR_HANSUN);
}

BOOL ledFWDLeftSent()
{
    return checkIRCommand(IR_CMD_FWD_LEFT, IR_HDR_HANSUN);
}

BOOL ledFWDRightSent()
{
    return checkIRCommand(IR_CMD_FWD_RIGHT, IR_HDR_HANSUN);
}

BOOL offCommandSent()
{
    return checkIRCommand(IR_CMD_OFF, IR_HDR_HANSUN);
}

void processIRSignal()
{
    LONG downTime = irDurationCounter10us;
    irDurationCounter10us = 0;
    //int value = 0;
    //printf("c:%d/%d\r\n", upTime, downTime);
    //printf("c:%d\r\n", downTime);

    //if the downtime is different than the known terminator
    if (downTime < 45 || 70 < downTime)
    {
        //is it arounf 9 000 us?
        if (200 < downTime && downTime < 1000)
        {
            //was there a previously long pulse?
            if (upTime > 3500 && upTime <= 4500)
            {
                //that is filler, usually 41 000 us
                printDebug("F");
                irFillerOK = TRUE;
                return;
            }
            else
            {
                //this is the initial run
                //usually 9 000 us
                printDebug("S");
                resetIRHolders();
                irStartOK = TRUE;
            }
        }
            //hm, unknown downtime duration
        else
        {
            //printf("RD:u-%d,d-%d\r\n", upTime, downTime);
            resetIRHolders();
        }
    }
        //this fall between the boundaries of a IR Terminator
    else
    {
        //        if(!irStartOK)
        //        {
        //            //wtf, terminator OK but initial signal missing!
        //            printDebug("W");
        //        }
        //        if(!irInitialOK)
        //        {
        //            //wtf, terminator OK but initial signal missing!
        //            printDebug("L");
        //        }

        if (upTime > 10 && upTime <= 80)
        {
            processSignal(0);
        }
        else if (upTime > 80 && upTime <= 200)
        {
            processSignal(1);
        }
        else if (upTime > 200 && upTime <= 300)
        {
            //end of package, usually 2 240us
            //printf("N\r\n");
            if (irStartOK && irInitialOK && irFillerOK)
            {
                printDebug("E\r\n");
                //we have a command for processing
                processIRCommand = 1;

                if (offCommandSent() && stbActivationInProgress)
                {
                    //printf("%d%d", offCommandSent(), stbActivationInProgress);
                    //printf("!\r\n");
                    switchMonitorOn();
                }
            }
            else
            {
                if (justWoke && cmdCounter == -1)
                {
                    justWoke = FALSE;
                    //seems the initial headers were missed becasue of the sleep time
                    processIRCommand = 1;
                    //printf("!%d%d", offCommandSent(), stbActivationInProgress);
                    if (offCommandSent() && stbActivationInProgress)
                    {
                        switchMonitorOn();
                    }
                }
                else
                {
                    //printf("!%d%d%d\r\n", irStartOK, irInitialOK, irFillerOK);
                }
            }
        }
        else if (upTime > 300 && upTime <= 600)
        {
            //usually 4 450 us, start if IR package
            printDebug("I");
            irInitialOK = TRUE;
            //printf("%d,%d\r\n", upTime, downTime);
        }
        else
        {

            //unkown uptime when the downtime is ok
            //printf("RU:u-%d,d-%d\r\n", upTime, downTime);

            resetIRHolders();
        }
    }
}

void handlePortStatus(BOOL portStatus)
{
    if (portStatus == 0)
    {
        if (!stateDownActive)//change! it have been up
        {
            upTime = irDurationCounter10us; //the time the signal was up
            //printf("u:%d\r\n", upTime);
            irDurationCounter10us = 0;
        }

        stateDownActive = TRUE;
    }
    else //if(portStatus == 1)
    {
        if (stateDownActive == TRUE)//change! it have been down
        {
            processIRSignal();
        }

        stateDownActive = FALSE;
    }
}

void initADC()
{
    //portb as anaog input
    mPORTBSetPinsAnalogIn(BIT_3);

    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | output in integer | trigger mode auto | enable  autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // define setup parameters for OpenADC10
    // ADC ref external    | disable offset test    | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
    // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

    // define setup parameters for OpenADC10
    // 				  use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    // set AN3
#define PARAM4	ENABLE_AN3_ANA

    // define setup parameters for OpenADC10
    // do not assign channels to scan
#define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // use ground as neg ref for A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF); // use ground as the negative reference
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using parameter define above

    EnableADC10(); // Enable the ADCd

    while (!mAD1GetIntFlag())
    {
    } // wait for the first conversion to complete so there will be valid data in ADC result registers
}

void delayMs(WORD delay)
{
    unsigned int int_status;
    while (delay--)
    {
        //int_status = INTDisableInterrupts();
        OpenCoreTimer(GetSystemClock() / 2000);
        //INTRestoreInterrupts(int_status);
        mCTClearIntFlag();
        while (!mCTGetIntFlag());
    }
    mCTClearIntFlag();
}

static int runningAverage(int M)
{
    // keep sum updated to improve speed.
    sumAverage -= runningAverageSamples[indexAverage];
    runningAverageSamples[indexAverage] = M;
    sumAverage += runningAverageSamples[indexAverage];

    indexAverage = indexAverage + 1;
    if (indexAverage >= runningAverageSampleSize)
    {
        indexAverage = 0;
        filledAverageActive = TRUE;
    }

    if (!filledAverageActive)
    {
        return -1;
    }

    return sumAverage >> 4;
}

void switchSTBOff()
{
    mPORTDClearBits(BIT_3);
}

void switchSTBOn()
{
    mPORTDSetBits(BIT_3);
}

void notifyOnProcessingCommandInProgress()
{
    stbActivationInProgress = TRUE;
    mPORTDSetBits(BIT_6);
}

void notifyOnCommandProcessed()
{
    stbActivationInProgress = FALSE;
    mPORTDClearBits(BIT_6);
}

void notifyOffProcessingCommandInProgress()
{
    mPORTDSetBits(BIT_5);
}

void notifyOffCommandProcessed()
{
    mPORTDClearBits(BIT_5);
}

#define IR_INPUT_READ              PORTDbits.RD7

void setupCNModuleAnd_IR_PIR_Input()
{
    //T2 used to sample IR Input signals
    setupTimer2();

    //IR INPUT
    mPORTDSetPinsDigitalIn(BIT_7); //D5 IR Input (CN16 module, RD7)
    //PIR input
    mPORTBSetPinsDigitalIn(BIT_2); //A1 PIR Input (CN4 module, RB2)

    // setup the change notice options
    //(ensure that CN continues working in sleep mode)
    mCNOpen(CN_OFF | CN_IDLE_CON, CN16_ENABLE | CN4_ENABLE, CN16_PULLUP_ENABLE | CN4_PULLUP_ENABLE);

    // read port(s) to clear mismatch
    mPORTDReadBits(BIT_7);
    mPORTBReadBits(BIT_2);
    mPORTBReadBits(BIT_4);

    // configure interrupts and clear change notice interrupt flag
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_3);
    mCNClearIntFlag(); // Clear interrupt flag
}

void enableCNModule(int userIsWatchingTV)
{
    if(userIsWatchingTV)
    {
        //do not enable PIR input during TV operation it is useless
        mCNOpen(CN_ON | CN_IDLE_CON, CN16_ENABLE, CN16_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    }
    else
    {
        mCNOpen(CN_ON | CN_IDLE_CON, CN16_ENABLE | CN4_ENABLE, CN16_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    }
    mCNIntEnable(1);
}

void disableCNModule()
{
    mCNClose();
}

void prepareTimer2AfterWake()
{
    TMR2 = 0;
    irDurationCounter10us = 0;
    T2CONbits.ON = 1; //enable IR sampling
}

void gotoSLEEP()
{
    if (processIRCommand)
    {
        printf("Ignore sleep, have IR to process\r\n");
        return;
    }


    printf("Sleep\r\n");
    delayMs(20);

    T2CONbits.ON = 0;
    enableCNModule(stbTVRunning);

    PowerSaveSleep();
    disableCNModule();

    prepareTimer2AfterWake();
}

void fillHeaderBuffer(unsigned int value)
{
    //0101000010101111
    int i = 3;
    int bitIdx = 15;
    //printf("hdr cmd:\r\n");
    for (; i < 34; i = i + 2)
    {
        if (CHECK_BIT(value, bitIdx))
        {
            //printf("1");
            irTransmitBuffer[i] = ONE;
        }
        else
        {
            //printf("0");
            irTransmitBuffer[i] = ZERO;
        }

        bitIdx--;
        irTransmitBuffer[i + 1] = TERMINATOR;
    }
    //printf("\r\n");
}

void prepareIRTransmitBuffer()
{
    //signal preparation - emmit
    irTransmitBuffer[0] = INITIAL;

    //signal begining
    irTransmitBuffer[1] = BEGIN;
    irTransmitBuffer[2] = TERMINATOR;

    int i = 3;
    //signal address
    fillHeaderBuffer(IR_HDR_HANSUN);

    //signal data
    i = 35;
    for (; i < 66; i = i + 2)
    {
        irTransmitBuffer[i] = ZERO;
        irTransmitBuffer[i + 1] = TERMINATOR;
    }
    //signal fill
    irTransmitBuffer[67] = FILL;
    irTransmitBuffer[68] = INITIAL;

    //signal end
    irTransmitBuffer[69] = END;
    irTransmitBuffer[70] = TERMINATOR;
}

void fillCommandBuffer(unsigned short int value)
{
    //0101000010101111
    int i = 35;
    int bitIdx = 15;
    //printf("off cmd:\r\n");
    for (; i < 66; i = i + 2)
    {
        if (CHECK_BIT(value, bitIdx))
        {
            //printf("1");
            irTransmitBuffer[i] = ONE;
        }
        else
        {
            //printf("0");
            irTransmitBuffer[i] = ZERO;
        }

        bitIdx--;
        irTransmitBuffer[i + 1] = TERMINATOR;
    }
    //printf("\r\n");
}

void setupPWMForIR()
{
    // init OC3 module
    OpenOC3(OC_OFF | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // init Timer3 mode and period (PR2) 40 kHz freq: 1 / 40 kHz = (X + 1) / 80MHz * 1
    //80 000 / 20 = X + 1 = 3999
    //80 000 / 38 = X + 1 = 2104
    //80 000 / 36 = X + 1 = 2221
    //80 000 / 40 = X + 1 = 2001
    OpenTimer3(T3_ON | T3_PS_1_1 | T3_SOURCE_INT, MAX_DUTY);

    SetDCOC3PWM(PR3 / 2);
}

void setupTimer4()
{
    //1 click of timer 4 is 3 200 nS (clock on 80 Mhz is 12,5 nS and prescaller is 256)
    OpenTimer4(T4_OFF | T4_SOURCE_INT | T4_PS_1_256, 0);
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_4);
    T4CONbits.ON = 0;
}

void setupIRTransmit()
{
    setupTimer4(); //timer 4 used to measure modulated pulses legth
    setupPWMForIR(); //use OC3 on D0
}

void sendIRCommandBlocking()
{
    T2CONbits.ON = 0;
    disableCNModule();
    sendingIR = 1;
    PR4 = 100;
    irTransmitBufferIndex = 0;
    TMR4 = 0;
    T4CONbits.ON = 1;
    printf("Sending IR ... ");
    while (sendingIR);
    printf("Done\r\n");
    T2CONbits.ON = 1;
}

int getAverageValue()
{
    int currentAverage = 0;
    int channel4 = 0;

    while (1)
    {
        delayMs(100);
        channel4 = ReadADC10(0);
        currentAverage = runningAverage(channel4);

        if (indexAverage == 15 && currentAverage != -1)
        {
            //printf("%.2f|%d=%d|\r\n", currentAverage * 3.18, currentAverage, channel4);
            printf("%.2f ", currentAverage * 3.18);
            return currentAverage;
        }
    }
}

void initRunningAverage()
{
    int i = 0;
    for (; i < runningAverageSampleSize; i++)
    {
        runningAverageSamples[i] = 0;
    }
}

BOOL waitForSTBToStart(int reads)
{
    //one read is ~1,6 s
    int counter = 0;
    while (getAverageValue() < TRESHOLD_ON)
    {
        counter++;
        if (reads > 0 && counter > reads)
        {
            printf("Timeout start\r\n");
            return FALSE;
        }
    }
    return TRUE;
}

BOOL waitForSTBToGoIDLE(int reads)
{
    //one read is ~1,6 s
    int counter = 0;
    while (getAverageValue() > TRESHOLD_OFF)
    {
        counter++;
        if (reads > 0 && counter > reads)
        {
            printf("Timeout idle\r\n");
            return FALSE;
        }
    }
    return TRUE;
}

void initLEDs()
{
    mPORTDClearBits(BIT_1); //yellow
    mPORTGClearBits(BIT_6); //green
    notifyOnCommandProcessed();
    notifyOffCommandProcessed();
}

void setupCNModuleAndPIRInput()
{
    //PIR Input and notification A1
    mPORTBSetPinsDigitalIn(BIT_2); //A1 PIR Input (CN4 module)
    mCNOpen(CN_OFF | CN_IDLE_CON | CN_IDLE_CON, CN4_ENABLE, CN4_PULLUP_ENABLE);
    // read port(s) to clear mismatch
    mPORTBReadBits(BIT_2);
    // configure interrupts and clear change notice interrupt flag
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_3);
    mCNClearIntFlag(); // Clear interrupt flag
}

static volatile int rtcAlarm = 0; // ISR

void configureRTC()
{
    RtccInit();
    while (RtccGetClkStat() != RTCC_CLK_ON);
    //printf("RTCC OK\r\n");

    // set time and date
    // time is MSb: hour, min, sec, rsvd. date is MSb: year, mon, mday, wday.
    // please note that the rsvd field has to be 0 in the time field!
    // Time is in Hex!
    RtccOpen(0x17040000, 0x08100604, 0);

    // set the RTCC priority in the INT controller
    INTSetVectorPriority(INT_RTCC_VECTOR, INT_PRIORITY_LEVEL_5);
    // set the RTCC sub-priority in the INT controller
    INTSetVectorSubPriority(INT_RTCC_VECTOR, INT_SUB_PRIORITY_LEVEL_1);
    // enable the RTCC event interrupts in the INT controller.
    INTEnable(INT_RTCC, INT_ENABLED);

    // enable alaram or endless alaram -
    // it allows the alarm rpt to move back to the beginning
    //RtccChimeDisable(); - this is now changeable
    // how much repats - 0 means run the alarm only once
    //RtccSetAlarmRptCount(0);
    //the above means once we have alarm interrupt the alarm will be disabled

    // used for marching (i.e. window) when cpmparing
    RtccSetAlarmRpt(RTCC_RPT_MON);
}

void setupHardware()
{
    SYSTEMConfig(SYS_CLOCK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    mPORTDSetPinsDigitalOut(BIT_1); //yellow LED
    mPORTGSetPinsDigitalOut(BIT_6); //green LED

    //A1 PIR Input (CN4 module, RB2)
    //A2 STB current consumption (AN3)
    //D0 IR Output
    mPORTDSetPinsDigitalOut(BIT_3); //D1 STB IRF control
    mPORTDSetPinsDigitalIn(BIT_4); //D2 BUT OTG
    mPORTDSetPinsDigitalOut(BIT_5); //D3 SHD RED LED Command OFF acqusition notificaiton
    mPORTDSetPinsDigitalOut(BIT_6); //D4 SHD GREEN LED Command ON acqusition notificaiton
    //D5 IR input (RD7)
    //D6 SHD BUT1
    //D7 SHD BUT2
    mPORTBSetPinsDigitalOut(BIT_14); //D9 monitor relay control

    DDPCONbits.JTAGEN = 0;

    initLEDs();
    initUART();

    initADC();
    setupCNModuleAnd_IR_PIR_Input();
    //setupCNModuleAndPIRInput();
    configureRTC();
    setupIRTransmit();

    switchMonitorOff();
    switchSTBOff();

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    T2CONbits.ON = 1;
}

void stopRTCAlarm()
{
    RtccAlarmDisable();
    rtcAlarm = 0;
}

void activateSTB()
{
    if (stbActive)
    {
        printf("STB already active\r\n");
        return;
    }

    //    switchSTBOn();
    //    stbActive = TRUE;
    //    return;
    notifyOnProcessingCommandInProgress();
    //mPORTGSetBits(BIT_6); //green LED ON
    printf("turn STB on\r\n");
    switchSTBOn();
    //start monitor
    printf("Wait for STB start ... ");
    //wait for the device to start
    waitForSTBToStart(0);
    printf("OK\r\n");
    printf("Wait for STB idle ... ");
    //wait for the device to go idle
    waitForSTBToGoIDLE(-1);
    printf("OK\r\n");
    printf("STB Active!\r\n");
    notifyOnCommandProcessed();

    stbActive = TRUE;
}

void deactivateSTB()
{
    //mPORTGClearBits(BIT_6); //green LED OFF
    stopRTCAlarm();
    if (stbTVRunning)
    {
        printf("Ignore STB Deactivation!\r\n");
        return;
    }

    switchSTBOff();
    stbActive = FALSE;
}

void makeSTBRun()
{
    //start STB    
    activateSTB();


    BOOL ommitIRCommands = FALSE;
    if (stbActive)
    {
        if (waitForSTBToStart(2))
        {
            //will get here if the STB was activated via PIR
            //mean while the user clicked OFF btn
            //if the STB is started we do not emit
            printf("Will not send IR start as device is started\r\n");
            ommitIRCommands = TRUE;
        }
    }

    if (!ommitIRCommands)
    {
        fillCommandBuffer(IR_CMD_OFF);
        do
        {
            sendIRCommandBlocking();
            //wait for the device to start again
        }
        while (!waitForSTBToStart(2));
    }

    stbTVRunning = 1;
}

void notifySwitchOffTimeout()
{
    int i = 0;
    for (; i < 3; i++)
    {
        notifyOffCommandProcessed();
        delayMs(300);
        notifyOffProcessingCommandInProgress();
        delayMs(300);
    }
}

void makeSTBStop()
{
    printf("Wait for STB idle ... ");
    //wait for the device to go idle
    BOOL result = waitForSTBToGoIDLE(7);
    if (!result)
    {
        notifySwitchOffTimeout();
    }
    printf("OK\r\n");
    switchSTBOff();
    //make sure STB is really off
    delayMs(1000);
    stbTVRunning = 0;
    stbActive = 0;
}

BOOL checkPIR()
{
    return PORTBbits.RB2;
    //return PORTDbits.RD5;
}

void initializeVariables()
{
    processIRCommand = 0;
    stbTVRunning = 0;

    prepareIRTransmitBuffer();
    //fillCommandBuffer(20655);
    initRunningAverage();

    stbActive = FALSE;
    pirActive = checkPIR();
    stbActivationInProgress = FALSE;
    upCommandActive = FALSE;
    downCommandActive = FALSE;
}

void readCNAndClrIsr()
{
    int portStatus = 0;
    portStatus = mPORTDRead(); //clear mismatch
    portStatus = mPORTBRead(); //clear mismatch
    mCNClearIntFlag();
}

void setupOffAlarm()
{
    printf("setupOffAlarm\r\n");
    //mPORTDClearBits(BIT_1); //yellow LED

    rtccTime tm, tAlrm; // time structure
    rtccDate dt, dAlrm; // date structure

    tm.l = 0x00000000;
    dt.l = 0x00000004;
    tAlrm.l = tm.l;
    dAlrm.l = dt.l;
    //setup alaram to trigger if for specified time period no movement is detected
    //then we have to shutdown the device
    rtcAlarm = 0;
    RtccSetTimeDate(tm.l, dt.l);

    //tAlrm.sec = 0x05; //time of the alarm
    tAlrm.min = 0x10; //time of the alarm
    RtccChimeDisable();
    RtccSetAlarmRptCount(0);
    RtccSetAlarmTimeDate(tAlrm.l, dAlrm.l); // set the alarm time

    RtccAlarmEnable(); // enable the alarm
}

void setupAutoIRAlarm()
{
    printf("setupAutoIRAlarm\r\n");
    //mPORTDClearBits(BIT_1); //yellow LED

    rtccTime tm, tAlrm; // time structure
    rtccDate dt, dAlrm; // date structure

    tm.l = 0x00000000;
    dt.l = 0x00000004;
    tAlrm.l = tm.l;
    dAlrm.l = dt.l;
    //setup alaram to trigger if for specified time period no movement is detected
    //then we have to shutdown the device
    rtcAlarm = 0;
    RtccSetTimeDate(tm.l, dt.l);

    tAlrm.sec = 0x04; //time of the alarm
    RtccChimeDisable();
    RtccSetAlarmRptCount(0);
    RtccSetAlarmTimeDate(tAlrm.l, dAlrm.l); // set the alarm time

    RtccAlarmEnable(); // enable the alarm
}

BOOL isUpCommandActive()
{
    return upCommandActive;
}

BOOL isDownCommandActive()
{
    return downCommandActive;
}

void initiateStartup()
{
    notifyOnProcessingCommandInProgress();
    printf("Initiating start procedure\r\n");
    printf("turn Monitor on\r\n");
    switchMonitorOn();

    makeSTBRun();

    printf("STB and Monitor are now operational\r\n");
    notifyOnCommandProcessed();
}

void initiateStop()
{
    notifyOffProcessingCommandInProgress();
    printf("Initiating stop procedure\r\n");
    switchMonitorOff();

    makeSTBStop();

    printf("STB and Monitor are now shut\r\n");
    notifyOffCommandProcessed();
}

void handleMovement()
{
    printf("Movement\r\n");
    pirActive = TRUE;
    if(stbActive && !stbTVRunning)
    {
        //STB is active, but TV is not running - diable the RTC
        //we want to disable RTC only in this case if we  process it
        //during stbTVRunning then we might stop auto IR signals
        //when there is a movement
        //okay, disable RTC off comamnds
        stopRTCAlarm();
    }
    activateSTB();
}

void handleMovementMissing()
{
    printf("No Movement\r\n");
    pirActive = FALSE;

    if (stbTVRunning)
    {
        //the TV is running, do not prepare a trigger to stop the STB!
        printf("Ingore STB shut off request via PIR\r\n");
        return;
    }

    if (stbActive)
    {
        stopRTCAlarm();
        setupOffAlarm();
    }
    else
    {
        printf("No need for STB auto off\r\n");
    }
}

void pumpUpCommand()
{
    delayMs(200);
    fillCommandBuffer(IR_CMD_PRG_UP);
    sendIRCommandBlocking();
    setupAutoIRAlarm();
}

void pumpDownCommand()
{
    delayMs(200);
    fillCommandBuffer(IR_CMD_PRG_DOWN);
    sendIRCommandBlocking();
    setupAutoIRAlarm();
}

void pumpLEDOnCommand()
{
    delayMs(200);

    fillHeaderBuffer(IR_HDR_LED);
    fillCommandBuffer(IR_CMD_LED_ON);
    sendIRCommandBlocking();
    fillHeaderBuffer(IR_HDR_HANSUN);
}

void pumpLEDClr1Command()
{
    delayMs(200);

    fillHeaderBuffer(IR_HDR_LED);
    fillCommandBuffer(IR_CMD_LED_CLR1);
    sendIRCommandBlocking();
    fillHeaderBuffer(IR_HDR_HANSUN);
}

void pumpLEDClr2Command()
{
    delayMs(200);

    fillHeaderBuffer(IR_HDR_LED);
    fillCommandBuffer(IR_CMD_LED_CLR2);
    sendIRCommandBlocking();
    fillHeaderBuffer(IR_HDR_HANSUN);
}

void pumpLEDOffCommand()
{
    delayMs(200);

    fillHeaderBuffer(IR_HDR_LED);
    fillCommandBuffer(IR_CMD_LED_OFF);
    sendIRCommandBlocking();
    fillHeaderBuffer(IR_HDR_HANSUN);
}

void initiateAutoUp()
{
    stopRTCAlarm();
    if (upCommandActive)
    {
        //again up - stopit
        printf("Stop - auto down\r\n");
        upCommandActive = FALSE;
        downCommandActive = FALSE;        
    }
    else
    {
        printf("Start - auto up\r\n");
        upCommandActive = TRUE;
        downCommandActive = FALSE;
        pumpUpCommand();
    }
}

void initiateAutoDown()
{
    stopRTCAlarm();
    if (downCommandActive)
    {
        //again down - stop it!
        printf("Stop - auto down\r\n");
        upCommandActive = FALSE;
        downCommandActive = FALSE;
    }
    else
    {
        printf("Start - auto down\r\n");
        upCommandActive = FALSE;
        downCommandActive = TRUE;        
        pumpDownCommand();
    }
}

void handleRTC()
{
    printf("RTC Alarm\r\n");
    if (!stbTVRunning)
    {
        printf("OFF\r\n");
        deactivateSTB();
    }
    else
    {
        if (isUpCommandActive())
        {
            printf("UP\r\n");
            pumpUpCommand();
        }
        else if (isDownCommandActive())
        {
            printf("DOWN\r\n");
            pumpDownCommand();
        }
        else
        {
            printf("WTF?\r\n");
            //wtf???
        }
    }
}

void handleIR()
{
    processIRCommand = 0;

    printf("CMD:%d-%d|%d%d%d\r\n", headerByte, commandByte, irStartOK, irInitialOK, irFillerOK);
    if (offCommandSent())
    {
        printf("OFF IR Receied\r\n");
        //T2CONbits.ON = 0;

        if (!stbTVRunning)
        {
            //first disable any pending RTC off alarms
            stopRTCAlarm();
            initiateStartup();
        }
        else
        {
            initiateStop();
        }
    }
    else if (upAutoSent() && stbTVRunning)
    {
        initiateAutoUp();
    }
    else if (downAutoSent() && stbTVRunning)
    {
        initiateAutoDown();
    }
    else if (ledOnSent())
    {
        stopRTCAlarm();
        pumpLEDOnCommand();
    }
    else if (ledOffSent())
    {
        stopRTCAlarm();
        pumpLEDOffCommand();
    }
    else if (ledFWDLeftSent())
    {
        stopRTCAlarm();
        pumpLEDClr1Command();
    }
    else if (ledFWDRightSent())
    {
        stopRTCAlarm();
        pumpLEDClr2Command();
    }
    else
    {
        printf("Other\r\n");
    }

    resetIRHolders();
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl3) changeNoticeHandlerForWake(void)
{
    //printf("\r\n+C%d%d\r\n", PORTBbits.RB2, PORTDbits.RD7);
    justWoke = TRUE;

    readCNAndClrIsr();
}

void __ISR(_TIMER_2_VECTOR, ipl2) interruptForCheckingIRSignals(void)
{
    if (IFS0bits.T2IF)
    {
        if (!processIRCommand)
        {
            //IFS0bits.T2IF = 0;
            //printf("i\r\n");
            irDurationCounter10us++;

            BOOL status = IR_INPUT_READ;
            if (portD != status)
            {
                handlePortStatus(status);

                if (status)
                {
                    //mPORTBClearBits(BIT_12);
                    mPORTDClearBits(BIT_1);
                }
                else
                {
                    //mPORTBSetBits(BIT_12);
                    mPORTDSetBits(BIT_1);
                }

                portD = status;
            }
        }
        mT2ClearIntFlag();
    }
}

void __ISR(_TIMER_4_VECTOR, ipl4) interruptForTransmitingIRSignals(void)
{
    if (mT4GetIntFlag())
    {
        if (irTransmitBufferIndex >= IR_TRANSMSIT_BUFFER_SIZE)
        {
            OC3CONbits.ON = 0;
            irTransmitBufferIndex = 0;
            T4CONbits.ON = 0;
            sendingIR = 0;
            mPORTGClearBits(BIT_6);
            //printf("S\r\n");
        }
        else
        {
            //printf("T\r\n");
            mPORTGToggleBits(BIT_6);
            //toggle OC3
            OC3CONINV = 0b1000000000000000;
            //OC3CONbits.ON = !OC3CONbits.ON;
            //mPORTBToggleBits(BIT_13);
            PR4 = irTransmitBuffer[irTransmitBufferIndex++];
            TMR4 = 0;
        }
        mT4ClearIntFlag();
    }
}

void __ISR(_RTCC_VECTOR, ipl5) RtccIsr(void)
{
    //    if (mRTCCGetIntFlag())
    //    {
    //        printf("\r\nRT\r\n");
    //    }

    // once we get in the RTCC ISR we have to clear the RTCC int flag
    INTClearFlag(INT_RTCC);

    rtcAlarm = 1; // show we're progressing somehow...
    //no need to call this
    //chmie 0 and alarm 0 => it will be automatically disabled!
    //RtccAlarmDisable();
    //printf("AIsr\r\n");
}

int main(void)
{
    setupHardware();
    printf("\r\n++++++++++++++++++++++\r\n");
    printf("Megalan Controller 2.3\r\n");
    printf("++++++++++++++++++++++\r\n");
    initializeVariables();
    //wait pir to stabilize
    delayMs(1000);
    

    while (1)
    {
        printf("\r\nAwake\r\n");

        if (processIRCommand)
        {
            handleIR();
        }

        if (checkPIR() && !pirActive)
        {
            handleMovement();
        }
        else if (!checkPIR() && pirActive)
        {
            handleMovementMissing();
        }

        if (rtcAlarm)
        {
            handleRTC();
        }

        gotoSLEEP();
        //wait a bit
        //otherwise we may put the device
        //to sleep while it is receiving commands!
        delayMs(500);
    }
}