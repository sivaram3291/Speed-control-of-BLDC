// Code created by Chamarthi Sivarama Raju, one can use this code for controlling the speed of BLDC motor for their requirement.

// The aim of this code example is tuning the PI controller for BLDC motor drive for a step reference.

// The code segments to initialize are 1. Hall signal inputs.
//                                     2. PWM channels.
//                                     3. Performing the commutation by updating the duty ratios of the channel's in the PWM handler.



const uint32_t ENCODER_CPR = 400;                            // Cycles per revolution; this depends on your encoder
const uint32_t ENCODER_EDGES_PER_ROTATION = ENCODER_CPR * 4; // PPR = CPR * 4
const uint32_t ENCODER_SAMPLES_PER_SECOND = 100;              // this will need to be tuned depending on your use case...


void Hall_signal_input_enable(void)
{
  // configuration of the hall sensor inputs signals. (The pins are controlled by PIOD controller)
  pmc_enable_periph_clk(ID_PIOD);                                                         // Master clock is fed to the PIOD controller
  PIOD->PIO_PER      = (PIO_PER_P0) | (PIO_PER_P1) | (PIO_PER_P2);                        // Enabling the PIOD control.
  PIOD->PIO_ODR      = 0xFFFFFFFF;                                                        // Disabling the output.
  PIOD->PIO_PUER     = 0x00000007;                                                        // Enabling the pullup resistors.
  PIOD->PIO_IFER     = (PIO_IFER_P0) | (PIO_IFER_P1) | (PIO_IFER_P2);
}

void QDEC_enable (void)
{ 
  // Initialize the Timer/counter-2 for Quadrature decoder based speed counting.
    // Enable the clock for channel 6,7 and 8 
    REG_PMC_PCER1 |= PMC_PCER1_PID33                  // TC6 power ON ; Timer counter 2 channel 0 is TC6
                     | PMC_PCER1_PID34                // TC7 power ON ; Timer counter 2 channel 1 is TC7
                     | PMC_PCER1_PID35;               // TC8 power ON ; Timer counter 2 channel 2 is TC8
    // Setting the channel-8 in waveform mode for triggering the counter in channel-6 using TIOA8.
    TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // Select Mck/2(42MHz)
                                | TC_CMR_WAVE               // Waveform mode
                                | TC_CMR_ACPC_TOGGLE        // Toggle TIOA of TC2 (TIOA8) on RC compare match
                                | TC_CMR_WAVSEL_UP_RC;      // UP mode with automatic trigger on RC Compare match
    // The RC register value will decide the speed updation period for the channel-6.
    TC2->TC_CHANNEL[2].TC_RC = F_CPU / 2 / ENCODER_SAMPLES_PER_SECOND;  // F_CPU = 84 MHz
    // Setting the T/C-2 channel-0 in the capture mode for speed counting.
    TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_ABETRG               // TIOA8 is used as an external trigger.
                                | TC_CMR_TCCLKS_XC0         // External clock selected (which is PHedges of the QDEC filter)
                                | TC_CMR_LDRA_EDGE          // RA loading on each edge of TIOA8 (external trigger)
                                | TC_CMR_ETRGEDG_RISING     // External TC trigger by edge selection of TIOA8
                                | TC_CMR_CPCTRG;            // RC Compare match resets the counter and starts the counter clock
    // Acessing the Block mode register of the T/C-2 for the enabling the QDEC for the speed measurement.
    TC2->TC_BMR = TC_BMR_QDEN                               // Enable QDEC (filter, edge detection and quadrature decoding)
                  | TC_BMR_SPEEDEN                          // Enable the speed measure on channel 0, the time base being provided by channel 2.
                  | TC_BMR_EDGPHA                           // Edges are detected on both PHA and PHB
                  | TC_BMR_MAXFILT(1);                      // Pulses with a period shorter than MAXFILT+1 peripheral clock cycles are discarded
    // enabling the clock for all the channels in the T/C-2.
    TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
    TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
    TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void speed_display_init(void)
{
  // Initilaize the DACC for speed display
    PMC->PMC_PCER1                   |=  PMC_PCER1_PID38;
    DACC->DACC_CR                     = (DACC_CR_SWRST);                                           // Resetting the DAC.
    DACC->DACC_MR                    |= (DACC_MR_TRGEN_EN)|                                        // DACC is trigger enabled.
                                        (DACC_MR_TRGSEL(1))|                                       // Here TIOA0 is the triggering signal for DACC.
                                        (DACC_MR_WORD_WORD)|                                       // Enabling word word transfer.
                                        (DACC_MR_TAG_EN)|                                          // writing the values in tag mode.
                                        (DACC_MR_REFRESH(1))|
                                        (DACC_MR_STARTUP_8)|
                                        (DACC_MR_MAXS);
    DACC->DACC_ACR                    =  DACC_ACR_IBCTLCH0(0b11)|
                                         DACC_ACR_IBCTLCH1(0b11);
    DACC->DACC_IDR                    =  0xFFFFFFFF ;
    DACC->DACC_CHER                   = (DACC_CHER_CH0)|((DACC_CHER_CH1));
  // Initialize the channel-0 in T/C-0 for the triggering signal generation of DACC
    pmc_set_writeprotect(false);
    PMC->PMC_PCER0                   |= PMC_PCER0_PID27;
    TC0->TC_CHANNEL[0].TC_CCR        |= TC_CCR_CLKDIS;
    TC0->TC_CHANNEL[0].TC_CMR        |= TC_CMR_WAVE|
                                        TC_CMR_TCCLKS_TIMER_CLOCK1|
                                        TC_CMR_WAVSEL_UP_RC;
    TC0->TC_CHANNEL[0].TC_RA          = 105;                          // 50 % duty ratio.
    TC0->TC_CHANNEL[0].TC_RC          = 210;                          // The TIOA0 signal at 200kHz frequency.
    TC0->TC_CHANNEL[0].TC_CMR        |= TC_CMR_ACPA_SET|
                                        TC_CMR_ACPC_CLEAR;
    TC0->TC_CHANNEL[0].TC_CCR        |= TC_CCR_SWTRG|
                                        TC_CCR_CLKEN;
}

void dac_write (long val_0, long val_1)
{
  val_0 = (val_1 << 16)|(1 << 28)|(val_0);
  DACC->DACC_CDR = val_0 ;
}

// The following macros are PWM_OOV register values.
#define T2_ON    0x00040000
#define T4_ON    0x00010000
#define T6_ON    0x00020000

// intializing the polling variable
static volatile int V = 0;


void setup() 
{
  // Add the starting delay for experimental convenience
  delay(2000);
  
  // Quadrature decoder initialization.
  QDEC_enable();  
  
  // hall signal inputs initialization.
  Hall_signal_input_enable();

  // Enabling the DACC value for speed displaying.
  speed_display_init();
  
  // PWM Module configuration for driving the motor.
  pmc_enable_periph_clk(PWM_INTERFACE_ID);                                                // Enabling the master clock to the PWM module.

  REG_PWM_WPCR    = 0x50574DF0;                                                           // Disabling the write protection of the PWM registers

  // connecting the channel outputs to the MCU pins.
  REG_PIOC_PDR   |= PIO_ABSR_P2 | PIO_ABSR_P3 | PIO_ABSR_P4 | PIO_ABSR_P5 | PIO_ABSR_P6 | PIO_ABSR_P7;
  REG_PIOC_ABSR  |= PIO_ABSR_P2 | PIO_ABSR_P3 | PIO_ABSR_P4 | PIO_ABSR_P5 | PIO_ABSR_P6 | PIO_ABSR_P7;
  REG_PIOC_PUDR   = 0x0000FFFF;

  // First disabling the channel's 0,1 and 2.
  REG_PWM_DIS     = 0x000000FF;

  // Synchronizing the PWM channels 0,1 and 2.
  REG_PWM_SCM    |= (PWM_SCM_SYNC0) | (PWM_SCM_SYNC1) | (PWM_SCM_SYNC2) | (PWM_SCM_UPDM_MODE0); // Here the duty cycle register is updated manually and the
                                                                                                // channels are updated manually.

  // initializing the channel zero
  REG_PWM_CMR0   |= (PWM_CMR_CPRE_MCK) | (PWM_CMR_DTE) | (PWM_CMR_CPOL);                  // Initializing the channel-0.
  REG_PWM_CPRD0   = PWM_CPRD_CPRD(8400);                                                  // PWM period is 100us approximately.
  REG_PWM_CDTY0   = PWM_CDTY_CDTY(0);
  REG_PWM_DT0     = PWM_DT_DTH(100) | PWM_DT_DTL(100);

  REG_PWM_CMPM0   = PWM_CMPM_CEN;                                                         // PWM Comparison 0 Mode Register

  REG_PWM_CMPV0   = PWM_CMPV_CV(8000);                                                    // comparison value for the event happening.
  // The comparision value should be less than the channel-0 period value.

  REG_PWM_IER2   |= (PWM_IER2_CMPM0);                                                     // Generates the PWM event comparision-0 based interrupt.
  NVIC_EnableIRQ(PWM_IRQn);

  // channel 1
  REG_PWM_CMR1   |= (PWM_CMR_DTE) | (PWM_CMR_CPOL);
  REG_PWM_CPRD1   = PWM_CPRD_CPRD(8400);
  REG_PWM_CDTY1   = PWM_CDTY_CDTY(0);
  REG_PWM_DT1     = PWM_DT_DTH(100) | PWM_DT_DTL(100);

  // channel 2
  REG_PWM_CMR2   |= (PWM_CMR_DTE) | (PWM_CMR_CPOL);
  REG_PWM_CPRD2   = PWM_CPRD_CPRD(8400);
  REG_PWM_CDTY2   = PWM_CDTY_CDTY(0);
  REG_PWM_DT2     = PWM_DT_DTH(100) | PWM_DT_DTL(100);

  // The Low side switches are controlled by the Override value of corresponding channel.
  REG_PWM_OS      =  0x00070000;
  REG_PWM_OOV     =  0x00000000;

  // Enabling the PWM channels
  REG_PWM_ENA     = 0x00000001;  
}

void loop() 
{
  // Do nothing.
}

void PWM_Handler(void)
{
  // Here the duty cycles are updated.
  static uint32_t FLAG = 0, duty = 0,count = 0;
  count++;
  FLAG = REG_PWM_ISR2;
  
  if ((FLAG) & (PWM_IER2_CMPM0))
  {
    // here we perfom the hall sensor polling operation.
    V = ((0b0111) & (PIOD->PIO_PDSR));
    switch (V)
    {
      case 0b0100:
        // for 001, T3 and T2 should be driven.
        REG_PWM_OOV       = T2_ON;
        REG_PWM_CDTYUPD0  = 0;                                // T1 PWM
        REG_PWM_CDTYUPD1  = duty;                             // T3 OFF
        REG_PWM_CDTYUPD2  = 0;                                // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      case 0b0010:
        // for 010, T1 and T6 should be driven.
        REG_PWM_OOV       = T6_ON;
        REG_PWM_CDTYUPD0  = duty;                             // T1 PWM
        REG_PWM_CDTYUPD1  = 0;                                // T3 OFF
        REG_PWM_CDTYUPD2  = 0;                                // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      case 0b0110:
        // for 011, T1 and T2 should be driven.
        REG_PWM_OOV       = T2_ON;
        REG_PWM_CDTYUPD0  = duty;                             // T1 PWM
        REG_PWM_CDTYUPD1  = 0;                                // T3 OFF
        REG_PWM_CDTYUPD2  = 0;                                // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      case 0b0001:
        // for 100, T5 and T4 should be driven.
        REG_PWM_OOV       = T4_ON;
        REG_PWM_CDTYUPD0  = 0;                                // T1 PWM
        REG_PWM_CDTYUPD1  = 0;                                // T3 OFF
        REG_PWM_CDTYUPD2  = duty;                             // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      case 0b0101:
        // for 101, T3 and T4 should be driven.
        REG_PWM_OOV       = T4_ON ;
        REG_PWM_CDTYUPD0  = 0;                                // T1 PWM
        REG_PWM_CDTYUPD1  = duty;                             // T3 OFF
        REG_PWM_CDTYUPD2  = 0;                                // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      case 0b0011:
        // for 110, T5 and T6 should be driven.
        REG_PWM_OOV       = T6_ON;
        REG_PWM_CDTYUPD0  = 0;                                // T1 PWM
        REG_PWM_CDTYUPD1  = 0;                                // T3 OFF
        REG_PWM_CDTYUPD2  = duty;                             // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
      default:
        // All switches are off.
        REG_PWM_OOV       = 0x00000000;
        REG_PWM_CDTYUPD0  = PWM_CDTYUPD_CDTYUPD(0);           // T1 PWM
        REG_PWM_CDTYUPD1  = PWM_CDTYUPD_CDTYUPD(0);           // T3 OFF
        REG_PWM_CDTYUPD2  = PWM_CDTYUPD_CDTYUPD(0);           // T5 OFF
        REG_PWM_SCUC      = PWM_SCUC_UPDULOCK;
        break;
    }

     // limited time period based BLDC motor drive application code
     if(count<= 49900)
     {
      
     // Time stepping based controller code
     if(count%50 == 0)                      // for 5 milli second sampling interval for executing the PI controller
     {
      
     // Speed profile for generating the reference input
     static int r_speed = 0;
     if(r_speed < 400)                     // This will increase the reference speed upto 400 rpm and then stays at the constant.
     {
      ++r_speed;
      ++r_speed;
     }
     
     // speed calculation
     static int my_reg = 0, my_rps = 0, my_rpm = 0;
     static float Pterm = 0,speed_error = 0,I_state = 0, I_term = 0;
     my_reg = TC2->TC_CHANNEL[0].TC_RA;                                                // Reading the speed value in the register.
     my_rps = ((my_reg / (ENCODER_EDGES_PER_ROTATION * 1.0)) * ENCODER_SAMPLES_PER_SECOND);
     my_rpm =  -my_rps * 60;
     dac_write(2*my_rpm,0);                                                            // The speed is displayed through DACC

     // PI-Speed Controller
     speed_error = r_speed - my_rpm;
     Pterm = 2*speed_error*8400;
     I_state += speed_error;
     
     if(I_state < 0)                                                  // Clamping the integrator
     {
       I_state = 0; 
     }
     if(I_state > 8400)                                               // Here i can limit the value upto 100 for unity duty ratio.
     {
       I_state = 8400; 
     }
     I_term = I_state*0.5;                                            // The integrator gain

     duty = (Pterm + I_term);
     if(duty < 0)
     {
      duty = 0;
     }
     if(duty > 8400)
     {
      duty = 8400;
     }
     }
     }
    if(count == 50000)
    {
        duty = 0;
        count = 49999;
    }
  }
}


// Code is working fine
// One can directly load the code into Arduino Due and work in their projects
