//###########################################################################
// Description:
//
// This example calls the sine function(CLAsin) of the CLA
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
#pragma DATA_SECTION(fAngle,"CpuToCla1MsgRAM")
float fAngle;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM")
float fResult;

//Golden Test Values
float test_values[BUFFER_SIZE] = {
-3.141593, -3.092119, -3.042645, -2.993171, -2.943697, 
-2.894223, -2.844749, -2.795275, -2.745801, -2.696327, 
-2.646854, -2.597380, -2.547906, -2.498432, -2.448958, 
-2.399484, -2.350010, -2.300536, -2.251062, -2.201589, 
-2.152115, -2.102641, -2.053167, -2.003693, -1.954219, 
-1.904745, -1.855271, -1.805797, -1.756323, -1.706850, 
-1.657376, -1.607902, -1.558428, -1.508954, -1.459480, 
-1.410006, -1.360532, -1.311058, -1.261584, -1.212111, 
-1.162637, -1.113163, -1.063689, -1.014215, -0.9647411, 
-0.9152672, -0.8657932, -0.8163193, -0.7668455, -0.7173715, 
-0.6678976, -0.6184238, -0.5689499, -0.5194759, -0.4700021, 
-0.4205281, -0.3710543, -0.3215804, -0.2721064, -0.2226326, 
-0.1731586, -0.1236847, -0.07421085, -0.02473695, 0.02473695, 
0.07421085, 0.1236847, 0.1731586, 0.2226326, 0.2721064, 
0.3215804, 0.3710543, 0.4205281, 0.4700021, 0.5194759, 
0.5689499, 0.6184238, 0.6678976, 0.7173715, 0.7668455, 
0.8163193, 0.8657932, 0.9152672, 0.9647411, 1.014215, 
1.063689, 1.113163, 1.162637, 1.212111, 1.261584, 
1.311058, 1.360532, 1.410006, 1.459480, 1.508954, 
1.558428, 1.607902, 1.657376, 1.706850, 1.756323, 
1.805797, 1.855271, 1.904745, 1.954219, 2.003693, 
2.053167, 2.102641, 2.152115, 2.201589, 2.251062, 
2.300536, 2.350010, 2.399484, 2.448958, 2.498432, 
2.547906, 2.597380, 2.646854, 2.696327, 2.745801, 
2.795275, 2.844749, 2.894223, 2.943697, 2.993171, 
3.042645, 3.092119, 3.141593
};	

float res_expected[BUFFER_SIZE]={
-1.224647e-16, -0.04945372, -0.09878642, -0.1478774, -0.1966064, 
-0.2448544, -0.2925031, -0.3394361, -0.3855383, -0.4306971, 
-0.4748020, -0.5177448, -0.5594208, -0.5997276, -0.6385669, 
-0.6758435, -0.7114661, -0.7453477, -0.7774054, -0.8075605, 
-0.8357394, -0.8618732, -0.8858978, -0.9077544, -0.9273896, 
-0.9447554, -0.9598091, -0.9725141, -0.9828392, -0.9907590, 
-0.9962543, -0.9993117, -0.9999235, -0.9980884, -0.9938107, 
-0.9871011, -0.9779758, -0.9664573, -0.9525737, -0.9363590, 
-0.9178528, -0.8971006, -0.8741529, -0.8490661, -0.8219014, 
-0.7927254, -0.7616096, -0.7286299, -0.6938671, -0.6574063, 
-0.6193367, -0.5797516, -0.5387476, -0.4964253, -0.4528881, 
-0.4082426, -0.3625982, -0.3160663, -0.2687610, -0.2207980, 
-0.1722946, -0.1233696, -0.07414275, -0.02473443, 0.02473443, 
0.07414275, 0.1233696, 0.1722946, 0.2207980, 0.2687610, 
0.3160663, 0.3625982, 0.4082426, 0.4528881, 0.4964253, 
0.5387476, 0.5797516, 0.6193367, 0.6574063, 0.6938671, 
0.7286299, 0.7616096, 0.7927254, 0.8219014, 0.8490661, 
0.8741529, 0.8971006, 0.9178528, 0.9363590, 0.9525737, 
0.9664573, 0.9779758, 0.9871011, 0.9938107, 0.9980884, 
0.9999235, 0.9993117, 0.9962543, 0.9907590, 0.9828392, 
0.9725141, 0.9598091, 0.9447554, 0.9273896, 0.9077544, 
0.8858978, 0.8618732, 0.8357394, 0.8075605, 0.7774054, 
0.7453477, 0.7114661, 0.6758435, 0.6385669, 0.5997276, 
0.5594208, 0.5177448, 0.4748020, 0.4306971, 0.3855383, 
0.3394361, 0.2925031, 0.2448544, 0.1966064, 0.1478774, 
0.09878642, 0.04945372, 1.224647e-16
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
	 fAngle = test_values[i];
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

