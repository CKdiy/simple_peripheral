/*
  Filename:       auxadc.c
  Revised:        $Date: 2018-01-14  $
  Revision:       $Revision:  $
 */
#include "Board.h"
#include <ti/sysbios/knl/Task.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>
#include "auxadc.h"

/* AUXAdc functions */
static void usDelay(volatile uint16_t delay);
/*********************************************************************
 * @fn      adc_OneShot_Read
 *
 * @brief   Collect voltage
 *
 * @param   none
 *
 * @return  none
 */
uint16_t adc_OneShot_Read(void)
{
    uint32_t turnedOnClocks = 0;
	uint32_t adcValue = 0;
	float res=0.00;
	float vbat = 0.00;
	
    /* Config clock */
    /* Only turn on clocks that are not already enabled. Not thread-safe, obviously. */
    turnedOnClocks |= AUXWUCClockStatus(AUX_WUC_ADI_CLOCK) ? 0 : AUX_WUC_ADI_CLOCK;
    turnedOnClocks |= AUXWUCClockStatus(AUX_WUC_ANAIF_CLOCK) ? 0 : AUX_WUC_ANAIF_CLOCK;
    /* Enable clocks and wait for ready */
    AUXWUCClockEnable(turnedOnClocks);  
    while(AUX_WUC_CLOCK_OFF == AUXWUCClockStatus(turnedOnClocks));
	
    /* Seclect auxIO */
    AUXADCSelectInput(ADC_COMPB_IN_AUXIO7);

    /* Enable */
    AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_10P6_US, AUXADC_TRIGGER_MANUAL);
	
	usDelay(100);
    AUXADCGenManualTrigger();    	

    adcValue = AUXADCReadFifo()*10; 
	
    AUXADCDisable();
	
	AUXWUCClockDisable(turnedOnClocks);
	
	res = (float)adcValue;
	
	vbat =((float)(res/4096))*43 + 6;
	
	vbat = vbat * 1.35;
	
	vbat += 3;
	
    return (uint16_t)vbat;
}

static void usDelay(volatile uint16_t delay)
{
	volatile uint16_t i;
	while(delay--)
	{
	  	i = 100;
		for(;i>0;i--);
	}
}