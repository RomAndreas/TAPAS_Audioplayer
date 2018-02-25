//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "SFO_V6.h"

#define EPWM_MIN_DB 1
#define EPWM_MIN_DB_FE 1
#define EPWM_Period    512
#define PCM_SampleRate 22050

// use this to write the program to the flash
//#define FLASH

#pragma CODE_SECTION(Mcbsp_RxINTA_ISR, "ramfuncs")
#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs")

// Prototype statements for functions found within this file.
void mcbsp_init_spiSLAVE(void);

void InitEPwm3Phase_new(void);
void writePCMtoPWM16Bit_allEqualDuty(Uint16 rdata);

__interrupt void Mcbsp_RxINTA_ISR(void);
__interrupt void cpu_timer0_isr(void);

void InitGPIO16();
void setGPIO16();
void clearGPIO16();

void error(void);

// Global variables used in this example
/* ringbuffer for buffering audio-data*/
Uint16 pcmBuffer[PCM_SampleRate/4];
Uint16 readIndex = 0;
Uint16 writeIndex = 0;
Uint16 bufferLevel = 0;

Uint16 DutyFine;
Uint16 status;

// Global data variables used for this example
Uint16 rdata;    // Received Data
Uint16 received = 0;

// The following declarations are required in order to use the SFO
// library functions:
//
int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.

// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
             {  &EPwm1Regs,&EPwm1Regs,&EPwm2Regs,&EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs};

#ifdef FLASH
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr){
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
   return;
}
#endif

void main(void)
{


#ifdef FLASH

// Copy time critical code and Flash setup code to RAM
// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
MemCopy((Uint16 *)&RamfuncsLoadStart,(Uint16 *)&RamfuncsLoadEnd,(Uint16 *)&RamfuncsRunStart);

InitFlash();                // initialize the flash

#endif

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO: 

// For this case just init GPIO pins for ePWM4, ePWM5, ePWM6
// These functions are in the F2806x_EPwm.c file
   InitEPwm4Gpio();
   InitEPwm5Gpio();
   InitEPwm6Gpio();
   InitMcbspaGpio();
   InitGPIO16();
   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.MRINTA= &Mcbsp_RxINTA_ISR;
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// This function can be found in F2806x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0
// 90MHz CPU Freq, Period 1000000/PCM_SampleRate (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 90, (1000000.0 / (float)PCM_SampleRate));

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the
// below settings must also be updated.
   CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm3Phase_new();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   
   mcbsp_init_spiSLAVE();

// Step 5. User specific code, enable interrupts

// Enable CPU int1 which is connected to CPU-Timer 0,
   IER |= M_INT1;

// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;

// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER6.bit.INTx5=1;     // Enable PIE Group 6, INT 5
   IER |= 0x20;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Initialize both PWMs to 50% to start at U_out = 0
   (*ePWM[4]).CMPA.half.CMPA = (*ePWM[4]).TBPRD / 2;
   (*ePWM[5]).CMPA.half.CMPA = (*ePWM[5]).TBPRD / 2;
   (*ePWM[6]).CMPA.half.CMPA = (*ePWM[6]).TBPRD / 2;

   status = SFO_INCOMPLETE;

   // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
   // MEP_ScaleFactor/HRMSTEP must be filled with calibrated value in order to
   // use in equations below.

    while  (status== SFO_INCOMPLETE){  // Call until complete
        status = SFO();
        if (status == SFO_ERROR) {
            error();    // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
        }               // exceeds maximum of 255.
    }

    EALLOW;

    // init the buffer
    int i;
    for(i=0; i < (PCM_SampleRate/4); i++){
        pcmBuffer[i] = 32768;
    }

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
       // Call the scale factor optimizer lib function SFO()
       // periodically to track for any change due to temp/voltage.
       // This function generates MEP_ScaleFactor by running the
       // MEP calibration module in the HRPWM logic. This scale
       // factor can be used for all HRPWM channels. The SFO()
       // function also updates the HRMSTEP register with the
       // scale factor value.

           status = SFO(); // in background, MEP calibration module continuously updates MEP_ScaleFactor
           if (status == SFO_ERROR) {
               error();   // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
           }              // exceeds maximum of 255.

   }

} 

void InitEPwm3Phase_new(void){
    // datasheet S322

    // EPWM Module 4 config
    (*ePWM[4]).TBPRD = EPWM_Period;                 // set Period
    (*ePWM[4]).TBPHS.half.TBPHS = 0;                // Set Phase register to zero
    (*ePWM[4]).TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Symmetrical mode : TB_COUNT_UPDOWN
    (*ePWM[4]).TBCTL.bit.PHSEN = TB_DISABLE;        // Master module
    (*ePWM[4]).TBCTL.bit.PRDLD = TB_SHADOW;
    (*ePWM[4]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync down-stream module
    (*ePWM[4]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[4]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    (*ePWM[4]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[4]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[4]).AQCTLA.bit.CAU = AQ_CLEAR;           // set actions for EPWM1A
    (*ePWM[4]).AQCTLA.bit.ZRO = AQ_SET;
    (*ePWM[4]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    (*ePWM[4]).DBCTL.bit.POLSEL = DB_ACTV_HIC;      // Active Hi complementary
    (*ePWM[4]).DBFED = EPWM_MIN_DB_FE;              // FED = EPWM_MIN_DB_FE TBCLKs
    (*ePWM[4]).DBRED = EPWM_MIN_DB;                 // RED = EPWM_MIN_DB TBCLKs

    // EPWM Module 5 config
    (*ePWM[5]).TBPRD = EPWM_Period;                 // set Period
    (*ePWM[5]).TBPHS.half.TBPHS = 0;                // Set Phase register to zero
    (*ePWM[5]).TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Symmetrical mode
    (*ePWM[5]).TBCTL.bit.PHSEN = TB_ENABLE;         // Slave module
    (*ePWM[5]).TBCTL.bit.PRDLD = TB_SHADOW;
    (*ePWM[5]).TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // sync flow-through
    (*ePWM[5]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[5]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    (*ePWM[5]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[5]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[5]).AQCTLA.bit.CAU = AQ_CLEAR;           // set actions for EPWM2A
    (*ePWM[5]).AQCTLA.bit.ZRO = AQ_SET;
    (*ePWM[5]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    (*ePWM[5]).DBCTL.bit.POLSEL = DB_ACTV_HIC;      // Active Hi complementary
    (*ePWM[5]).DBFED = EPWM_MIN_DB_FE;              // FED = EPWM_MIN_DB_FE TBCLKs
    (*ePWM[5]).DBRED = EPWM_MIN_DB;                 // RED = EPWM_MIN_DB TBCLKs

    // EPWM Module 6 config
    (*ePWM[6]).TBPRD = EPWM_Period;                 // set Period
    (*ePWM[6]).TBPHS.half.TBPHS = 0;                // Set Phase register to zero
    (*ePWM[6]).TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Symmetrical mode
    (*ePWM[6]).TBCTL.bit.PHSEN = TB_ENABLE;         // Slave module
    (*ePWM[6]).TBCTL.bit.PRDLD = TB_SHADOW;
    (*ePWM[6]).TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // sync flow-through
    (*ePWM[6]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[6]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    (*ePWM[6]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[6]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // load on CTR=Zero
    (*ePWM[6]).AQCTLA.bit.CAU = AQ_CLEAR;           // set actions for EPWM3A
    (*ePWM[6]).AQCTLA.bit.ZRO = AQ_SET;
    (*ePWM[6]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    (*ePWM[6]).DBCTL.bit.POLSEL = DB_ACTV_HIC;      // Active Hi complementary
    (*ePWM[6]).DBFED = EPWM_MIN_DB_FE;              // FED = EPWM_MIN_DB_FE TBCLKs
    (*ePWM[6]).DBRED = EPWM_MIN_DB;                 // RED = EPWM_MIN_DB TBCLKs

    // Now configure the HRPWM4 resources
    EALLOW;
    (*ePWM[4]).HRCNFG.all = 0x0;                // Clear all bits first
    (*ePWM[4]).HRCNFG.bit.EDGMODE = HR_FEP;     // Control falling edge position
    (*ePWM[4]).HRCNFG.bit.CTLMODE = HR_CMP;     // CMPAHR controls the MEP.
    (*ePWM[4]).HRCNFG.bit.HRLOAD = HR_CTR_ZERO; // Shadow load on CTR=Zero.
    (*ePWM[4]).HRCNFG.bit.SELOUTB = HR_NORM_B;
    (*ePWM[4]).HRCNFG.bit.AUTOCONV = 1;         // enable Autoconvert
    EDIS;

    // Now configure the HRPWM5 resources
    EALLOW;
    (*ePWM[5]).HRCNFG.all = 0x0;                // Clear all bits first
    (*ePWM[5]).HRCNFG.bit.EDGMODE = HR_FEP;     // Control falling edge position
    (*ePWM[5]).HRCNFG.bit.CTLMODE = HR_CMP;     // CMPAHR controls the MEP.
    (*ePWM[5]).HRCNFG.bit.HRLOAD = HR_CTR_ZERO; // Shadow load on CTR=Zero.
    (*ePWM[5]).HRCNFG.bit.SELOUTB = HR_NORM_B;
    (*ePWM[5]).HRCNFG.bit.AUTOCONV = 1;         // enable Autoconvert
    EDIS;

    // Now configure the HRPWM6 resources
    EALLOW;
    (*ePWM[6]).HRCNFG.all = 0x0;                // Clear all bits first
    (*ePWM[6]).HRCNFG.bit.EDGMODE = HR_FEP;     // Control falling edge position
    (*ePWM[6]).HRCNFG.bit.CTLMODE = HR_CMP;     // CMPAHR controls the MEP.
    (*ePWM[6]).HRCNFG.bit.HRLOAD = HR_CTR_ZERO; // Shadow load on CTR=Zero.
    (*ePWM[5]).HRCNFG.bit.SELOUTB = HR_NORM_B;
    (*ePWM[6]).HRCNFG.bit.AUTOCONV = 1;         // enable Autoconvert
    EDIS;
}

void mcbsp_init_spiSLAVE(){
    // Steps refer to datasheet, page 969
    // Step 1. Place the transmitter and receiver in reset
    McbspaRegs.SPCR2.bit.XRST=0; // Transmitter reset
    McbspaRegs.SPCR1.bit.RRST=0; // Receiver reset

    // Step 2. Place the sample rate generator in reset
    McbspaRegs.SPCR2.bit.GRST=0; // Sample Rate generator Reset
    McbspaRegs.SPCR2.bit.FRST=0; // Frame Sync generator reset

    // Step 3. Program registers that affect SPI operation.
    // -> McBSP as an SPI Slave ( Section 15.7.7)
    McbspaRegs.SPCR1.bit.CLKSTP  = 0b10;
    McbspaRegs.PCR.bit.CLKXP = 0;
    McbspaRegs.PCR.bit.CLKRP = 0;
    McbspaRegs.PCR.bit.CLKXM = 0;   // McBSP is SPI-Slave
    McbspaRegs.XCR2.bit.XPHASE = 0;
    McbspaRegs.RCR2.bit.RPHASE = 0;
    McbspaRegs.XCR1.bit.XFRLEN1 = 0;
    McbspaRegs.RCR1.bit.RFRLEN1 = 0;

    /* 0b000(8Bit) 0b001(12Bit) 0b010(16Bit)*/
    McbspaRegs.XCR1.bit.XWDLEN1 = 0b010;
    McbspaRegs.RCR1.bit.RWDLEN1 = 0b010;

    McbspaRegs.PCR.bit.SCLKME = 0;
    McbspaRegs.SRGR2.bit.CLKSM = 1;
    McbspaRegs.SRGR1.bit.CLKGDV = 1;
    McbspaRegs.PCR.bit.FSXM = 0;
    McbspaRegs.PCR.bit.FSXP = 1;
    McbspaRegs.XCR2.bit.XDATDLY = 0b00;
    McbspaRegs.RCR2.bit.RDATDLY = 0b00;

    // Step 4. Enable the sample rate generator.
    McbspaRegs.SPCR2.bit.GRST=1;
    delay_loop();                   // Wait at least 2 SRG clock cycles

    // Step 5. Enable the transmitter and receiver.
    McbspaRegs.SPCR2.bit.XRST=1;
    McbspaRegs.SPCR1.bit.RRST=1;
    delay_loop();                   // Wait at least 2 SRG clock cycles

    // Step 6. If necessary, enable the frame-synchronization logic of the sample rate generator.
    /* McbspaRegs.SPCR2.bit.FRST=1;*/

   McbspaRegs.MFFINT.bit.RINT = 1; // Enable Receive Interrupts
}

// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:
Uint16 dacValue;
__interrupt void cpu_timer0_isr(void){
    // show activity on GPIO16
    setGPIO16();

    // do buffer Level range check
    if(bufferLevel > 0){
        dacValue = pcmBuffer[readIndex];
        writePCMtoPWM16Bit_allEqualDuty(dacValue);
        bufferLevel--;

        // do index range check
        if(readIndex > ((PCM_SampleRate/4)-1) ){
            readIndex = 0;
        }else{
            readIndex++;
        }
    }
    CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

   // show activity on GPIO16
   clearGPIO16();
}

__interrupt void Mcbsp_RxINTA_ISR(void){

    rdata=McbspaRegs.DRR1.all;

    // do buffer Level range check
    if(rdata != 0xFFFF){
        bufferLevel++;
        pcmBuffer[writeIndex] = rdata;

        // do index range check
        if(writeIndex > ((PCM_SampleRate/4)-1) ){
            writeIndex = 0;
        }else{
            writeIndex++;
        }
    }else{
        // ignore data, because its a dummy
    }

    // send a message to Pi, when to deliver new data
    if(bufferLevel < ((PCM_SampleRate/4) - 50) ){
        McbspaRegs.DXR1.all = 0xBEEF;
    }else{
        McbspaRegs.DXR1.all = 0xF001;
    }

    received ++;

    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void error (void) {
    ESTOP0;         // Stop here and handle error
}

Uint16 CMPA_reg_val;
Uint16 CMPAHR_reg_val;

void writePCMtoPWM16Bit_allEqualDuty(Uint16 rdata){

    Uint32 temp;
    DutyFine = rdata;

    CMPA_reg_val = ((long)DutyFine * (*ePWM[4]).TBPRD)>>16;
    temp = ((long)DutyFine * (*ePWM[4]).TBPRD) ;
    temp = temp - ((long)CMPA_reg_val<<16);

    CMPAHR_reg_val = temp;
    (*ePWM[4]).CMPA.all = ((long)CMPA_reg_val)<<16 | CMPAHR_reg_val; // loses lower 8-bits

    CMPA_reg_val = ((long)DutyFine * (*ePWM[5]).TBPRD)>>16;
    temp = ((long)DutyFine * (*ePWM[5]).TBPRD) ;
    temp = temp - ((long)CMPA_reg_val<<16);

    CMPAHR_reg_val = temp;
    (*ePWM[5]).CMPA.all = ((long)CMPA_reg_val)<<16 | CMPAHR_reg_val; // loses lower 8-bits

    CMPA_reg_val = ((long)DutyFine * (*ePWM[6]).TBPRD)>>16;
    temp = ((long)DutyFine * (*ePWM[6]).TBPRD) ;
    temp = temp - ((long)CMPA_reg_val<<16);

    CMPAHR_reg_val = temp;
    (*ePWM[6]).CMPA.all = ((long)CMPA_reg_val)<<16 | CMPAHR_reg_val; // loses lower 8-bits
}

void InitGPIO16(){
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;     // Disable Pullup for GPIO19
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // Clear output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // GPIO19 = GPIO19
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;     // GPIO19 = output
    EDIS;
}

void setGPIO16(){
    GpioDataRegs.GPASET.bit.GPIO16 = 1;
}

void clearGPIO16(){
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
}
