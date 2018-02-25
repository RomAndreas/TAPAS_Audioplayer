//###########################################################################
// Description:
//
// This example calls the log base 10 function(CLAlog10) of the CLA
// math library on a test vector and compares the result to the
// equivalent "math.h" routine
//
//###########################################################################
// $TI Release: CLA Math Library for CLA C Compiler V4.00 $
// $Release Date: December 10, 2011 $
//###########################################################################

#include "DSP28x_Project.h"
#include "CLAShared.h"
#include "math.h"

#define CLA1_RAM0_ENABLE	0
#define CLA1_RAM1_ENABLE	1

#define EPSILON				1e-2

// Constants
#define WAITSTEP	asm(" RPT #255||NOP")

//Global Variables
//Task 1 (C) Variables
float y[BUFFER_SIZE];
#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM")
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM")
float fResult;

//Golden Test Values
float test_values[BUFFER_SIZE] = {
1, 1.779528, 2.559055, 3.338583, 4.118110, 
4.897638, 5.677166, 6.456693, 7.236220, 8.015748, 
8.795276, 9.574803, 10.35433, 11.13386, 11.91339, 
12.69291, 13.47244, 14.25197, 15.03150, 15.81102, 
16.59055, 17.37008, 18.14961, 18.92913, 19.70866, 
20.48819, 21.26772, 22.04725, 22.82677, 23.60630, 
24.38583, 25.16535, 25.94488, 26.72441, 27.50394, 
28.28346, 29.06299, 29.84252, 30.62205, 31.40158, 
32.18110, 32.96063, 33.74016, 34.51968, 35.29921, 
36.07874, 36.85827, 37.63779, 38.41732, 39.19685, 
39.97638, 40.75591, 41.53543, 42.31496, 43.09449, 
43.87402, 44.65354, 45.43307, 46.21260, 46.99213, 
47.77165, 48.55118, 49.33071, 50.11024, 50.88976, 
51.66929, 52.44882, 53.22835, 54.00787, 54.78740, 
55.56693, 56.34646, 57.12598, 57.90551, 58.68504, 
59.46457, 60.24409, 61.02362, 61.80315, 62.58268, 
63.36221, 64.14173, 64.92126, 65.70079, 66.48032, 
67.25984, 68.03937, 68.81890, 69.59843, 70.37795, 
71.15748, 71.93700, 72.71654, 73.49606, 74.27559, 
75.05511, 75.83465, 76.61417, 77.39370, 78.17323, 
78.95276, 79.73228, 80.51181, 81.29134, 82.07087, 
82.85040, 83.62992, 84.40945, 85.18898, 85.96851, 
86.74803, 87.52756, 88.30708, 89.08662, 89.86614, 
90.64567, 91.42519, 92.20473, 92.98425, 93.76378, 
94.54330, 95.32284, 96.10236, 96.88189, 97.66142, 
98.44095, 99.22047,	100
};	

float res_expected[BUFFER_SIZE]={
0, 0.2503047, 0.4080796, 0.5235621, 0.6146979, 
0.6899867, 0.7541316, 0.8100101, 0.8595118, 0.9039441, 
0.9442495, 0.9811298, 1.015122, 1.046646, 1.076035, 
1.103561, 1.129446, 1.153875, 1.177002, 1.198960, 
1.219861, 1.239802, 1.258867, 1.277131, 1.294657, 
1.311504, 1.327721, 1.343354, 1.358444, 1.373028, 
1.387137, 1.400803, 1.414052, 1.426908, 1.439395, 
1.451533, 1.463340, 1.474835, 1.486034, 1.496951, 
1.507601, 1.517995, 1.528147, 1.538067, 1.547765, 
1.557251, 1.566535, 1.575624, 1.584527, 1.593251, 
1.601803, 1.610191, 1.618419, 1.626494, 1.634422, 
1.642208, 1.649856, 1.657372, 1.664760, 1.672025, 
1.679170, 1.686200, 1.693117, 1.699926, 1.706630, 
1.713233, 1.719736, 1.726143, 1.732457, 1.738681, 
1.744816, 1.750867, 1.756834, 1.762720, 1.768527, 
1.774258, 1.779914, 1.785498, 1.791011, 1.796454, 
1.801830, 1.807141, 1.812387, 1.817571, 1.822693, 
1.827756, 1.832760, 1.837708, 1.842599, 1.847437, 
1.852221, 1.856952, 1.861633, 1.866264, 1.870846, 
1.875380, 1.879868, 1.884309, 1.888706, 1.893058, 
1.897367, 1.901634, 1.905860, 1.910044, 1.914189, 
1.918295, 1.922362, 1.926391, 1.930383, 1.934339, 
1.938260, 1.942145, 1.945995, 1.949812, 1.953596, 
1.957347, 1.961066, 1.964753, 1.968409, 1.972035, 
1.975631, 1.979197, 1.982734, 1.986243, 1.989723, 
1.993176, 1.996601, 2
};

//Linker defined vars
extern Uint16 Cla1Prog_Start;

//CLA ISRs
interrupt void cla1_task1_isr( void );


//! Main Function
void main(void)
{
   //! Step 1: Setup the system clock
   /*! Disable the watchdog timer, initialize the system clock,
    *  PLL and configure the peripheral clock.
    */
	InitSysCtrl();

   //! Step 2: Initialize PIE control
   /*! Intialize PIE control, disable all interrupts and
    * then copy over the PIE Vector table from BootROM to RAM
    */
   DINT;
   InitPieCtrl();
   IER = 0x00000000;
   IFR = 0x00000000;
   InitPieVectTable();
	
   /*! Assign user defined ISR to the PIE vector table */
	EALLOW;
    PieVectTable.CLA1_INT1  = &cla1_task1_isr;
	EDIS;

    /*! Compute all CLA task vectors */ 
    EALLOW;
    Cla1Regs.MVECT1 = ((Uint16)Cla1Task1 - (Uint16)&Cla1Prog_Start);
    EDIS;
   
    //! Step 3 : Mapping CLA tasks
    /*! All tasks are enabled and will be started by an ePWM trigger
     *  Map CLA program memory to the CLA and enable software breakpoints
     */
    EALLOW;
	Cla1Regs.MPISRCSEL1.bit.PERINT1SEL 	= CLA_INT1_NONE;
	Cla1Regs.MIER.all 		 		    = 0x00FF;
	EDIS;	
    
 
    /*! Enable CLA interrupts at the group and subgroup levels */
    PieCtrlRegs.PIEIER11.all       = 0xFFFF;
    IER = (M_INT11 );
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    /*!Switch the CLA program space to the CLA and enable software forcing
     * Also switch over CLA data ram 0 and 1
     */
    EALLOW;
    Cla1Regs.MMEMCFG.bit.PROGE 	= 1;
    Cla1Regs.MCTL.bit.IACKE	= 1;
    Cla1Regs.MMEMCFG.bit.RAM0E	= CLA1_RAM0_ENABLE;
    Cla1Regs.MMEMCFG.bit.RAM1E	= CLA1_RAM1_ENABLE;
    EDIS;	

    // Invoke Task(s)
    test_run();

    // Report Results
    test_report();

	//Forever loop
	while(1){
	 asm(" NOP");
	}

}

//C28 C tasks
void test_run(void)
{
	int i;
	
	for(i=0;i<BUFFER_SIZE;i++)
	{
	 fVal = test_values[i];
	 Cla1ForceTask1andWait();
     WAITSTEP;
	 y[i] = fResult; 
	}   
}

void test_report(void)
{
	unsigned int i;
	unsigned int pass_count=0;
	unsigned int fail_count=0;
	float fError[BUFFER_SIZE];
	float fErrMetric;
	for(i=0;i<BUFFER_SIZE;i++){
	  fError[i] = fabs(res_expected[i]-y[i]);
	  fErrMetric = fError[i];
	  if( fErrMetric < EPSILON){
		pass_count++;
	  }else{
		fail_count++;
	  }
	}
	if(fail_count) test_error(); 
}

void test_error( void )
{
	asm(" ESTOP0");	
}



//CLA ISR's
interrupt void cla1_task1_isr( void)
{
        PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

