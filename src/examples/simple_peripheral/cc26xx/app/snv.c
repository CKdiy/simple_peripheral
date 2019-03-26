/*
 * snv.c
 * Description: nvram managerment for system
 */

#include <string.h>
#include "osal_snv.h"
#include "snv.h"
#include "ibeaconcfg.h"
#include "comdef.h"

static const block_info_t fb[] =
{
    /* block ID (must < 0xf)    size */
	{BLE_NVID_DEVINF_START,       44},
};

#define FB_SIZE         (sizeof(fb)/sizeof(block_info_t))

uint8_t nvBuf[50] = {0};

static int Nvram_Block_Init(uint8_t nvid, uint8_t len);
static int Nvram_Block_Check(uint8_t nvid, uint8_t len);
static int DevInf_Snv_Init(uint8_t len);
/* Init nvram */
void Nvram_Init(void)
{  
	 uint8_t i;
	 
	 for(i=0; i<FB_SIZE; i++)
	 {
    	if (Nvram_Block_Check(fb[i].NVid, fb[i].size) != 0)
		{
			Nvram_Block_Init(fb[i].NVid, fb[i].size);
		}
	 }
}

static int Nvram_Block_Check(uint8_t nvid, uint8_t len)
{
    uint32_t	crc = 0;
	uint8_t		status;
	
    if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	memset(nvBuf, 0, sizeof(nvBuf));
	
	status = osal_snv_read(nvid, len, (void *)nvBuf);
	if(status != SUCCESS)
	 	return -1;
	
	memcpy((void *)&crc, nvBuf, sizeof(crc));
    if (crc != crc32(0, &nvBuf[sizeof(crc)], len - sizeof(crc)))
        return -1;
    
    return 0;
}

static int Nvram_Block_Init(uint8_t nvid, uint8_t len)
{	
	if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	if(BLE_NVID_DEVINF_START == nvid)
	{
		return DevInf_Snv_Init(len);
	}
	 
    return 0;
}

static int DevInf_Snv_Init(uint8_t len)
{
 	uint8_t		status;
	
	uint8_t default_Major[]={0x27, 0x18}; //10008
	uint8_t default_Minor[]={0x27, 0x18}; //10008
	uint8_t default_Uuid[]={0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
	                        0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25};
	
	snvinf_t *ptr = (snvinf_t *)nvBuf;
	
	memset(nvBuf, 0, sizeof(nvBuf));
	
	ptr->ibeaconinf_config.atFlag        = 0x00;
	ptr->ibeaconinf_config.txPower       = 0;
	ptr->ibeaconinf_config.txInterval    = 3;
	ptr->ibeaconinf_config.initFlag      = 0xFF - 1;
	memcpy(ptr->ibeaconinf_config.majorValue, default_Major, sizeof(uint16_t));
	memcpy(ptr->ibeaconinf_config.minorValue, default_Minor, sizeof(uint16_t));
    memcpy(ptr->ibeaconinf_config.uuidValue,  default_Uuid,  sizeof(default_Uuid));
	memcpy(ptr->ibeaconinf_config.hwvr, "0001", sizeof(uint32_t));
	memcpy(ptr->ibeaconinf_config.mDate, "2019-04-01", MANUFACTURE_DATE_LEN);
	
	ptr->crc32 = crc32(0, (uint8_t *)(&ptr->ibeaconinf_config), len - sizeof(uint32_t));
	
	status = osal_snv_write(BLE_NVID_DEVINF_START, len,  ptr);
	if(status != SUCCESS)
		return -1;
	
	return 0;
}

int Ble_ReadNv_Inf(uint8_t nvid, uint8_t *readbuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
		{
			len =  fb[i].size; 
			break;
		}
	}
	
	if(i == FB_SIZE)
		len = 47;  
	
	status = osal_snv_read(nvid, len, (void *)readbuf);	
	if(status != SUCCESS)
		return -1;

	return 0;
}

int Ble_WriteNv_Inf(uint8_t nvid, uint8_t *writebuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	uint32_t	crc = 0;    
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
		{
			len =  fb[i].size; 
			break;
		}
	}
	
	if(i == FB_SIZE)
		len = 47;  
	
	memset(nvBuf, 0, sizeof(nvBuf));
	
	crc = crc32(0, writebuf, len - sizeof(crc));
	memcpy(nvBuf, (void *)&crc, sizeof(crc));
	memcpy(nvBuf + sizeof(crc), writebuf, len - sizeof(crc));
	  
	status = osal_snv_write(nvid, len, (void *)nvBuf);	
	if(status != SUCCESS)
		return -1;
	
	return 0;
}