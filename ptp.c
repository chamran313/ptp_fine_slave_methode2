
#define ETH_PTP_PositiveTime      ((uint32_t)0x00000000)  /*!< Positive time value */
#define ETH_PTP_NegativeTime      ((uint32_t)0x80000000)  /*!< Negative time value */
#define biliard      1000000000
#define minus			   1
#define plus			   0
//#define Base_addend  2386092942  for 120mhz
#define Base_addend     3314017975  // for 166.6mhz  -> 6ns ts acc
#define max_allowed_offset		10000 // 10us
//#define synq_interval		710000000	//800ms -- 767 is without pkt loss-- (54+2+1+(3*6)+1) * 9.25


void ETH_EnablePTPTimeStampUpdate(void) {
    /* Enable the PTP system time update with the Time Stamp Update register value */
    ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU;
}


void ETH_SetPTPTimeStampUpdate(uint32_t Sign, uint32_t SecondValue, uint32_t SubSecondValue)
{
  
  /* Set the PTP Time Update High Register */
  ETH->PTPTSHUR = SecondValue;
  
  /* Set the PTP Time Update Low Register with sign */
  ETH->PTPTSLUR = Sign | SubSecondValue;   
}


/*******************************************************************************
 * Function Name  : ETH_PTPTimeStampUpdateOffset
 * Description    : Updates time base offset
 * Input          : Time offset with sign
 * Output         : None
 * Return         : None
 *******************************************************************************/
void ETH_PTPTime_UpdateOffset(int32_t seconds, int32_t nanoseconds) {
    uint32_t Sign;
    uint32_t SecondValue;
    uint32_t NanoSecondValue;
    //uint32_t SubSecondValue;
    //uint32_t addend;

    /* determine sign and correct Second and Nanosecond values */
    if (seconds < 0 || (seconds == 0 && nanoseconds < 0)) {
        Sign = ETH_PTP_NegativeTime;
        SecondValue = -seconds;
        NanoSecondValue = -nanoseconds;
    } else {
        Sign = ETH_PTP_PositiveTime;
        SecondValue = seconds;
        NanoSecondValue = nanoseconds;
    }

    /* convert nanosecond to subseconds */
    //SubSecondValue = ETH_PTPNanoSecond2SubSecond(NanoSecondValue);

    /* read old addend register value*/
    //addend = ETH_GetPTPRegister(ETH_PTPTSAR);

    //while (ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);

		
		
		while( ETH->PTPTSCR & 0x8) {}
    while (ETH->PTPTSCR & 0x4) {}

    /* Write the offset (positive or negative) in the Time stamp update high and low registers. */
    ETH_SetPTPTimeStampUpdate(Sign, SecondValue, NanoSecondValue);
    /* Set bit 3 (TSSTU) in the Time stamp control register. */
    ETH_EnablePTPTimeStampUpdate();
    /* The value in the Time stamp update registers is added to or subtracted from the system */
    /* time when the TSSTU bit is cleared. */
    while (ETH->PTPTSCR & 0x8);

    /* write back old addend register value */
    //ETH_SetPTPTimeStampAddend(addend);
    //ETH_EnablePTPTimeStampAddend();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ETH_SetPTPTimeStampAddend(uint32_t Value)
{
  /* Set the PTP Time Stamp Addend Register */
  ETH->PTPTSAR = Value;    
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void ETH_EnablePTPTimeStampAddend(void)
{
  /* Enable the PTP block update with the Time Stamp Addend register value */
  ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;    
}


void ETH_InitializePTPTimeStamp(void)
{
  /* Initialize the PTP Time Stamp */
  ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;    
}
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void adj_freq( uint32_t addend ) 
{
	 while(  ETH->PTPTSCR & 0x00000020 );
	 ETH_SetPTPTimeStampAddend( addend ); 
	 ETH_EnablePTPTimeStampAddend();
	 while(  ETH->PTPTSCR & 0x00000020 );

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t my_abs(int32_t a)
{
 int32_t  b = 0;
	
 if( a < 0 )	b = -a;
 else  b = a;
	
return b;

}



