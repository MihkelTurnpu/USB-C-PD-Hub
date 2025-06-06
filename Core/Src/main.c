/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"              // For printf                           
#include "stm32g4xx_hal.h"      // For HAL functions                    
#include <stdbool.h>            // For boolean type         
#include "TPS_low_reg_bin.h"    // For TPS26750 patch data       
#include <string.h>             // For memcpy 
#include <stdarg.h>             // For va_list, va_start, va_end
#include "pd_handler.h"         // For PD handling functions
#include <stdint.h>             // For uint8_t, uint32_t types

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern const char tps25750x_lowRegion_i2c_array[];  // TPS26750 patch data
extern int gSizeLowRegionArray;                     // Size of patch data array
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;

I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* Buffer for I2C data */
uint8_t i2c_buf[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC5_Init(void);
/* USER CODE BEGIN PFP */


//swo initialization
void SWO_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR  = 0xC5ACCE55;
  ITM->TCR  = ITM_TCR_ITMENA_Msk | ITM_TCR_TSENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_TraceBusID_Msk;
  ITM->TER  = 1;
}

//swo printf function
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// I2C3 scan function
void I2C3_Scan(void)
{
    printf("Scanning I2C3...\r\n");

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        uint8_t addr_8bit = addr << 1;  // STM32 HAL expects 8-bit address

        if (HAL_I2C_IsDeviceReady(&hi2c3, addr_8bit, 3, 100) == HAL_OK)
        {
            printf("Found device at 0x%02X (8-bit: 0x%02X)\r\n", addr, addr_8bit);
        }
    }

    printf("Scan complete.\r\n");
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Print TX Source PDOs from register 0x32
// shows what PDOs are available for the sink to choose from
void print_tx_source_pdos(uint8_t pdAddr)
{
    uint8_t dev = pdAddr << 1;
    uint8_t  cmd = 23;
    uint8_t  raw[1 + 23]; // [0]=length prefix, [1..23]=payload
    uint8_t* d = &raw[1]; // d[0]..d[22]

    // 1) Ask for 23 bytes
    HAL_I2C_Mem_Write(&hi2c3, dev, 0x32,
                      I2C_MEMADD_SIZE_8BIT,
                      &cmd, 1, HAL_MAX_DELAY);

    // 2) Read back prefix + 23 bytes
    HAL_I2C_Mem_Read(&hi2c3, dev, 0x32,
                     I2C_MEMADD_SIZE_8BIT,
                     raw, sizeof(raw), HAL_MAX_DELAY);

    // 3) Decode Number Valid PDOs = bits 2..0 of byte#1
    uint8_t num_pdos = d[0] & 0x07;
    printf("TX Source: %u valid PDO(s)\n", num_pdos);

    // 4) Extract & print PDO1–PDO5 (each 4 bytes, little-endian)
    for (int i = 0; i < 5; i++) {
        // Each PDO_i lives in bits [24+32*(i) … 55+32*(i)] ⇒ raw bytes d[3+4*i]..d[6+4*i]
        uint32_t pdo = (uint32_t)d[3 + 4*i]
                     | (uint32_t)d[4 + 4*i] << 8
                     | (uint32_t)d[5 + 4*i] << 16
                     | (uint32_t)d[6 + 4*i] << 24;
        printf("  PDO #%d = 0x%08lX\n", i+1, (unsigned long)pdo);
    }
}

/* ---------------------------------------------------------------------------*/
// Print active PDO contract (48 bits) from register 0x34
// for debugging purposes
void print_active_pdo_contract(uint8_t pdAddr)
{
    uint8_t  dev = pdAddr << 1;
    uint8_t  cmd = 6;          // 6 data bytes (bits 47:0)
    uint8_t  raw[7];           // [0]=length prefix, [1..6]=data bytes
    uint64_t reg = 0;

    // 1) send length prefix
    HAL_I2C_Mem_Write(&hi2c3, dev, 0x34, I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);

    // 2) read back prefix + 6 data bytes
    HAL_I2C_Mem_Read(&hi2c3, dev, 0x34, I2C_MEMADD_SIZE_8BIT, raw, 7, HAL_MAX_DELAY);

    // 3) assemble into a 48-bit (in a 64-bit) register value
    for (int i = 0; i < 6; i++) {
        reg |= (uint64_t)raw[1 + i] << (8 * i);
    }

    // 4) extract fields
    uint16_t first_pdo_ctrl = (reg >> 32) & 0x03FF;         // bits 41:32
    uint32_t active_pdo     = (uint32_t)(reg & 0xFFFFFFFF); // bits 31:0

    // 5) print
    printf("Active PDO Contract @0x34:\n");
    printf("  First PDO Control Bits = 0x%03X  (%u)\n", first_pdo_ctrl, first_pdo_ctrl);
    printf("  Active PDO = 0x%08lX\n",(unsigned long)active_pdo);
}
/* ---------------------------------------------------------------------------*/
//for masking interrupts in INT_MASK1 register (11 bytes - prefix)
//used for debugging purposes
void TPS26750_MaskInterrupts(uint8_t pdAddr)
{
    uint8_t devAddr = pdAddr << 1;

    // INT_MASK1 is 88 bits (11 bytes) long + 1 prefix byte;
    uint8_t data[12] = { 0 };
    data[0] = 11;                // total mask-bytes to follow
    data[1] = (1U << 3);         // enable interrupt bit 3  (Plug Insert/Removal)
    data[2] = (1U << (13 - 8));  // enable interrupt bit 13 (New Contract as Provider)
    data[6] = (1U << (42 - 40)); // enable interrupt bit 42 (Sink Transition Completed)

    // Write INT_MASK1 (offset = 0x16)
    HAL_I2C_Mem_Write(&hi2c3, devAddr,0x16,I2C_MEMADD_SIZE_8BIT,data,sizeof(data),100);
}
/* ---------------------------------------------------------------------------*/
// Clear all bits in INT_EVENT1 register (11 bytes - prefix)
void clear_all_INT_EVENT1_bits(uint8_t i2c_addr)
{
    uint8_t buf[12];
    buf[0] = 11;  // Length prefix: 11 data bytes to follow

    // Set all bits to 1 — this clears all events in INT_EVENT1
    for (int i = 1; i <= 11; ++i) {
        buf[i] = 0xFF;
    }

    HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x18, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 100);
	printf("Cleared all INT_EVENT1 bits for device 0x%02X\n", i2c_addr);

}
/* ---------------------------------------------------------------------------*/
// Print raw bits of INT_EVENT1 register (11 bytes - prefix)
//used for debugging purposes
void print_INT_EVENT1_raw_bits(const uint8_t bits_with_prefix[11])
{
    printf("INT_EVENT1 bits: ");
    const uint8_t *bits = &bits_with_prefix[1];  // Skip the first byte (length prefix)
    for (int byte = 10; byte >= 0; --byte) {
        for (int bit = 7; bit >= 0; --bit) {
            printf("%c", (bits[byte] & (1 << bit)) ? '1' : '0');
        }
    }
    printf("\n");
}

/* ---------------------------------------------------------------------------*/
// Interrupt handler for USB events
// This function is called when an interrupt occurs on GPIO pins 10 or 11
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    uint8_t buf[5];           // For writing length
    uint8_t raw[5];           // Includes prefix + 4 data bytes
    uint8_t data[4];          // Final parsed register data
    uint8_t int_event1_raw[12]; // 1 prefix + 11 bytes data
    uint32_t int_event1 = 0;
    uint32_t power_status = 0;
    //printf("EXTI fired: GPIO_Pin = %u\n", GPIO_Pin);

    if (GPIO_Pin == GPIO_PIN_10)
    {
    	//handle_pd1_event();
    	// Read INT_EVENT1_RAW (0x14), 11 bytes + prefix
		buf[0] = 11;
		HAL_I2C_Mem_Write(&hi2c3, 0x21 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
		HAL_I2C_Mem_Read(&hi2c3, 0x21 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, int_event1_raw, 12, 100);
		print_INT_EVENT1_raw_bits(&int_event1_raw[1]);
		//read_pd_hard_reset_reason(0x23);  // For TPS26750_2

		// Read INT_EVENT1 (0x14)
		buf[0] = 4;
		HAL_I2C_Mem_Write(&hi2c3, 0x21 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
		HAL_I2C_Mem_Read(&hi2c3, 0x21 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);
		memcpy(data, &raw[1], 4);
		int_event1 = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

		// Read STATUS (0x1A)
		buf[0] = 4;
		HAL_I2C_Mem_Write(&hi2c3, 0x21 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
		HAL_I2C_Mem_Read(&hi2c3, 0x21 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);
		memcpy(data, &raw[1], 4);
		power_status = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

		if ((int_event1 & (1 << 5))||(power_status & (1 << 0)))
		{
			printf("TPS26750_1: Sink Transition completed\n");
			handle_pd1_event();
			clear_all_INT_EVENT1_bits(0x21);
		}
		else if ((data[0] & 0x01) == 0)
		{

			printf("TPS26750_1: Plug Disconnected\n");
			uint8_t reg06;
			HAL_I2C_Mem_Read(&hi2c3, 0x75 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
			reg06 &= ~0x80;
			HAL_I2C_Mem_Write(&hi2c3, 0x75 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
			clear_all_INT_EVENT1_bits(0x21);
		}

    }
    else if (GPIO_Pin == GPIO_PIN_11)
    {
    	//handle_pd2_event();
        // Read INT_EVENT1_RAW (0x14), 11 bytes + prefix
        buf[0] = 11;
        HAL_I2C_Mem_Write(&hi2c3, 0x23 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, 0x23 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, int_event1_raw, 12, 100);
        print_INT_EVENT1_raw_bits(&int_event1_raw[1]);
        //read_pd_hard_reset_reason(0x23);  // For TPS26750_2

        // Read INT_EVENT1 (0x14)
        buf[0] = 4;
        HAL_I2C_Mem_Write(&hi2c3, 0x23 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, 0x23 << 1, 0x14, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);
        memcpy(data, &raw[1], 4);
        int_event1 = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

        // Read STATUS (0x1A)
        buf[0] = 4;
        HAL_I2C_Mem_Write(&hi2c3, 0x23 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, 0x23 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);
        memcpy(data, &raw[1], 4);
        power_status = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

        if ((int_event1 & (1 << 5)) || (power_status & (1 << 0)))
        {
            printf("TPS26750_2: Plug 2 connected\n");
            handle_pd2_event();
            clear_all_INT_EVENT1_bits(0x23);

        }
        else if ((data[0] & 0x01) == 0)
        {
            printf("TPS26750_2: Plug Disconnected\n");
            uint8_t reg06;
            HAL_I2C_Mem_Read(&hi2c3, 0x74 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
            reg06 &= ~0x80;
            HAL_I2C_Mem_Write(&hi2c3, 0x74 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
            clear_all_INT_EVENT1_bits(0x23);
        }
    }
}

/* ---------------------------------------------------------------------------*/
void mask_all_interrupts(uint8_t i2c_addr)
{
	 uint8_t buf[5];
	    buf[0] = 4;  // Length prefix
	    buf[1] = 0x00;
	    buf[2] = 0x00;
	    buf[3] = 0x00;
	    buf[4] = 0x00;

	    HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 100);
	   // printf("All interrupts masked on device 0x%02X\n", i2c_addr);

	    // Read INT_MASK1 to confirm it was set to 0
	    buf[0] = 4;
	    HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);

	    uint8_t raw[5];
	    HAL_I2C_Mem_Read(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);

	    //printf("INT_MASK1 for 0x%02X: %02X %02X %02X %02X\n", i2c_addr, raw[1], raw[2], raw[3], raw[4]);  // Skip prefix
	}
/* ---------------------------------------------------------------------------*/
void unmask_all_interrupts(uint8_t i2c_addr)
{
	 uint8_t buf[5];
	    buf[0] = 4;  // Length prefix
	    buf[1] = 0x01;
	    buf[2] = 0x01;
	    buf[3] = 0x01;
	    buf[4] = 0x01;

	    HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 100);
	    //printf("All interrupts masked on device 0x%02X\n", i2c_addr);

	    // Read INT_MASK1 to confirm it was set to 0
	    buf[0] = 4;
	    HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);

	    uint8_t raw[5];
	    HAL_I2C_Mem_Read(&hi2c3, i2c_addr << 1, 0x16, I2C_MEMADD_SIZE_8BIT, raw, 5, 100);

	    //printf("INT_MASK1 for 0x%02X: %02X %02X %02X %02X\n",i2c_addr, raw[1], raw[2], raw[3], raw[4]);  // Skip prefix
	}
/* ---------------------------------------------------------------------------*/

void Read_TPS_Mode(uint8_t address)
{
    uint8_t mode[4] = {0};
    HAL_StatusTypeDef ret;

    printf("Reading MODE register from TPS at 0x%02X...\r\n", address);

    ret = HAL_I2C_Mem_Read(&hi2c3, (address << 1), 0x03, I2C_MEMADD_SIZE_8BIT, mode, 5, 100);

    if (ret == HAL_OK)
    {
        printf("TPS Mode = %c%c%c%c\r\n", mode[1], mode[2], mode[3], mode[4]);
    }
    else
    {
        printf("Failed to read MODE register at 0x%02X (I2C error)\r\n", address);
    }
}

/* ---------------------------------------------------------------------------*/

bool Upload_TPS26750_Patch(I2C_HandleTypeDef *hi2c, uint8_t dev7bitAddr, const uint8_t *patch_data, uint32_t patch_size)
{
    uint8_t devAddr = dev7bitAddr << 1; // 8-bit address
    uint8_t data1[7];
    uint8_t cmd1[5];
    uint32_t timeout_ms;
    uint8_t patch_slave_addr = 0x50 << 1; // 8-bit
    //uint32_t offset = 0;

    printf("[PATCH] Starting patch upload sequence for 0x%02X...\n", dev7bitAddr);

    // -------- Step 1: Start Patch Burst Mode --------
    data1[0] = 6;  // Length of following data
    data1[1] = (uint8_t)(patch_size & 0xFF);
    data1[2] = (uint8_t)((patch_size >> 8) & 0xFF);
    data1[3] = (uint8_t)((patch_size >> 16) & 0xFF);
    data1[4] = (uint8_t)((patch_size >> 24) & 0xFF);
    data1[5] = 0x50;   // Patch target address
    data1[6] = 0x64;   // 5 seconds timeout

    if (HAL_I2C_Mem_Write(hi2c, devAddr, 0x09, I2C_MEMADD_SIZE_8BIT, data1, sizeof(data1), HAL_MAX_DELAY) != HAL_OK)
    {
        printf("[PATCH] Failed writing DATA1\n");
        return false;
    }

    HAL_I2C_Mem_Read(hi2c, devAddr, 0x09, I2C_MEMADD_SIZE_8BIT, data1, sizeof(data1), HAL_MAX_DELAY);
    printf("[PATCH] DATA1 = %02X %02X %02X %02X %02X %02X %02X\n", data1[0], data1[1], data1[2], data1[3], data1[4], data1[5], data1[6]);

    cmd1[0] = 4;
    cmd1[1] = 'P';
    cmd1[2] = 'B';
    cmd1[3] = 'M';
    cmd1[4] = 's';
    if (HAL_I2C_Mem_Write(hi2c, devAddr, 0x08, I2C_MEMADD_SIZE_8BIT, cmd1, sizeof(cmd1), HAL_MAX_DELAY) != HAL_OK)
    {
        printf("[PATCH] Failed sending PBMs\n");
        return false;
    }

    HAL_I2C_Mem_Read(hi2c, devAddr, 0x08, I2C_MEMADD_SIZE_8BIT, cmd1, sizeof(cmd1), HAL_MAX_DELAY);
    printf("[PATCH] CMD1 = %02X %c%c%c%c\n", cmd1[0], cmd1[1], cmd1[2], cmd1[3], cmd1[4]);

    // -------- Step 2: Wait for Patch Start Status --------
    timeout_ms = 500;
    uint32_t elapsed = 0;
    uint8_t patch_status[7];
    do {
        HAL_I2C_Mem_Read(hi2c, devAddr, 0x09, I2C_MEMADD_SIZE_8BIT, patch_status, sizeof(patch_status), HAL_MAX_DELAY);
        printf("[PATCH] Output DATA1 = %02X %02X %02X %02X %02X %02X %02X\n", patch_status[0], patch_status[1], patch_status[2], patch_status[3], patch_status[4], patch_status[5], patch_status[6]);
        HAL_Delay(5);
        elapsed += 5;
    } while ((patch_status[1] != 0x00) && (elapsed < timeout_ms));

    if (patch_status[1] != 0x00)
    {
        printf("[ERROR] PatchStartStatus = 0x%02X\n", patch_status[1]);
        return false;
    }

    printf("[PATCH] PBMs acknowledged. Uploading patch to 0x50...\n");

    // -------- Step 3: Write Patch Data to 0x50 in 4 equal bursts --------
    #define TOTAL_BURSTS 4
    uint32_t chunk_size = patch_size / TOTAL_BURSTS;
    uint32_t offset = 0;

    for (int burst = 0; burst < TOTAL_BURSTS; burst++)
    {
        // On last burst, send all remaining data in case of rounding
        uint32_t this_chunk = (burst < TOTAL_BURSTS - 1) ? chunk_size : (patch_size - offset);

        if (HAL_I2C_Master_Transmit(hi2c, patch_slave_addr, (uint8_t *)(patch_data + offset), this_chunk, HAL_MAX_DELAY) != HAL_OK)
        {
            printf("[ERROR] Failed patch write (burst #%d at offset %lu)\n", burst + 1, (unsigned long)offset);
            printf("[ERROR] Sending PBMe...\n");
                    cmd1[0] = 4; cmd1[1] = 'P'; cmd1[2] = 'B'; cmd1[3] = 'M'; cmd1[4] = 'e';
                    HAL_I2C_Mem_Write(hi2c, devAddr, 0x08, I2C_MEMADD_SIZE_8BIT, cmd1, sizeof(cmd1), HAL_MAX_DELAY);
                    printf("[PATCH] PBMe sent, patch upload sequence stopped!\n");
                    HAL_I2C_Mem_Read(hi2c, devAddr, 0x09, I2C_MEMADD_SIZE_8BIT, data1, sizeof(data1), HAL_MAX_DELAY);
                    printf("[PATCH] DATA1 = %02X %02X %02X %02X %02X %02X %02X\n", data1[0], data1[1], data1[2], data1[3], data1[4], data1[5], data1[6]);
        }

        printf("[PATCH] Burst #%d sent: %lu bytes\n", burst + 1, (unsigned long)this_chunk);
        offset += this_chunk;

    }
    // -------- Step 3: Write Patch Data to 0x50 --------
    printf("[PATCH] Patch data written. Sending PBMc...\n");
    HAL_Delay(1);
    // -------- Step 4: Patch Burst Mode Complete (PBMc) --------
    cmd1[0] = 4; cmd1[1] = 'P'; cmd1[2] = 'B'; cmd1[3] = 'M'; cmd1[4] = 'c';
    if (HAL_I2C_Mem_Write(hi2c, devAddr, 0x08, I2C_MEMADD_SIZE_8BIT, cmd1, sizeof(cmd1), HAL_MAX_DELAY) != HAL_OK)
    {
        printf("[ERROR] Failed sending PBMc\n");
        return false;
    }

    elapsed = 0;
    do {
        HAL_I2C_Mem_Read(hi2c, devAddr, 0x08, I2C_MEMADD_SIZE_8BIT, cmd1, sizeof(cmd1), HAL_MAX_DELAY);
        printf("[PATCH] Output CMD1 = %02X %c%c%c%c\n", cmd1[0], cmd1[1], cmd1[2], cmd1[3], cmd1[4]);
        if (cmd1[1] == 0 && cmd1[2] == 0 && cmd1[3] == 0 && cmd1[4] == 0)
            break;
        HAL_Delay(20);
        elapsed += 20;
    } while (elapsed < timeout_ms);

    if (elapsed >= timeout_ms)
    {
        printf("[ERROR] PBMc timeout\n");
        return false;
    }

    //----------Step 5: Read data1 -------------------------

    HAL_I2C_Mem_Read(hi2c, devAddr, 0x09, I2C_MEMADD_SIZE_8BIT, data1, sizeof(data1), HAL_MAX_DELAY);
    printf("[PATCH] DATA1 = %02X %02X %02X %02X %02X %02X %02X\n", data1[0], data1[1], data1[2], data1[3], data1[4], data1[5], data1[6]);
    return true;
}

/* ---------------------------------------------------------------------------*/

/*
//TPS55288 fault interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t fault;
    uint8_t addr;
    const char* label;

    if (GPIO_Pin == GPIO_PIN_5)  // PC5 → TPS55288 at address 0x75
    {
        addr = 0x75 << 1;
        label = "TPS55288_1 (PC5)";
    }
    else if (GPIO_Pin == GPIO_PIN_0)  // PB0 → TPS55288 at address 0x74
    {
        addr = 0x74 << 1;
        label = "TPS55288_2 (PB0)";
    }
    else
    {
        return; // unknown pin, ignore
    }

    // Read latched fault status
    if (HAL_I2C_Mem_Read(&hi2c3, addr, 0x0D, I2C_MEMADD_SIZE_8BIT, &fault, 1, 100) != HAL_OK) {
        printf("❌ Failed to read fault status from %s\n", label);
        return;
    }

    printf("%s Fault Detected (0x0D): 0x%02X\n", label, fault);

    if (fault & (1 << 7)) printf(" → VOUT OVP\n");
    if (fault & (1 << 6)) printf(" → VOUT UVP\n");
    if (fault & (1 << 5)) printf(" → IOUT OCP\n");
    if (fault & (1 << 4)) printf(" → Thermal Shutdown\n");
    if (fault & (1 << 3)) printf(" → VIN UVP\n");
    if (fault & (1 << 2)) printf(" → VOUT < 0.3V\n");
    if (fault & (1 << 1)) printf(" → Startup Fault\n");

    // Optional: Clear latched fault bits
    uint8_t clear = 0x00;
    HAL_I2C_Mem_Write(&hi2c3, addr, 0x0D, I2C_MEMADD_SIZE_8BIT, &clear, 1, 100);
}
*/

/* ---------------------------------------------------------------------------*/

void read_and_print_INT_EVENT1(uint8_t i2c_addr)
{
    uint8_t buf[1] = {11};          // Request to read 11 data bytes
    uint8_t raw[12];                // 1 prefix + 11 data bytes
    uint8_t result[11];             // Final 11-byte register content

    // Write length prefix to register 0x14
    if (HAL_I2C_Mem_Write(&hi2c3, i2c_addr << 1, 0x14,
                          I2C_MEMADD_SIZE_8BIT, buf, 1, 100) != HAL_OK)
    {
        printf("Failed to write INT_EVENT1 read length to 0x%02X\n", i2c_addr);
        return;
    }

    // Read 1 prefix + 11 actual data bytes
    if (HAL_I2C_Mem_Read(&hi2c3, i2c_addr << 1, 0x14,
                         I2C_MEMADD_SIZE_8BIT, raw, 12, 100) != HAL_OK)
    {
        printf("Failed to read INT_EVENT1 from 0x%02X\n", i2c_addr);
        return;
    }

    memcpy(result, &raw[1], 11);  // Skip prefix byte

    // Print each byte as hex
    printf("INT_EVENT1 [0x14] from 0x%02X: ", i2c_addr);
    for (int i = 10; i >= 0; --i) {
        printf("%02X ", result[i]);
    }
    printf("\n");

    // Print full binary representation
    printf("Bits: ");
    for (int i = 10; i >= 0; --i) {
        for (int b = 7; b >= 0; --b) {
            printf("%c", (result[i] & (1 << b)) ? '1' : '0');
        }
    }
    printf("\n");
}
/* ---------------------------------------------------------------------------*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SWO_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC4_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC5_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);
  //I2C3_Scan();

  Read_TPS_Mode(0x21);  // for first TPS
  Read_TPS_Mode(0x23);  // for second TPS
  read_and_print_INT_EVENT1(0x21);  // for TPS26750_1
  read_and_print_INT_EVENT1(0x23);  // for TPS26750_2
  Upload_TPS26750_Patch(&hi2c3, 0x21, (const uint8_t *)tps25750x_lowRegion_i2c_array, gSizeLowRegionArray);
  Upload_TPS26750_Patch(&hi2c3, 0x23, (const uint8_t *)tps25750x_lowRegion_i2c_array, gSizeLowRegionArray);
  HAL_Delay(10);
  Read_TPS_Mode(0x21);  // for first TPS
  Read_TPS_Mode(0x23);  // for second TPS
  //mask_all_interrupts(0x21);  // TPS26750_1
  //mask_all_interrupts(0x23);  // TPS26750_2
  //TPS26750_MaskInterrupts(0x21);
  //TPS26750_MaskInterrupts(0x23);
  clear_all_INT_EVENT1_bits(0x21);  // Clear for TPS26750_1
  clear_all_INT_EVENT1_bits(0x23);  // Clear for TPS26750_2
  //read_and_print_INT_EVENT1(0x21);  // for TPS26750_1
  //read_and_print_INT_EVENT1(0x23);  // for TPS26750_2
  //clear_all_INT_EVENT1_bits(0x21);  // Clear for TPS26750_1
  //clear_all_INT_EVENT1_bits(0x23);  // Clear for TPS26750_2
  print_tx_source_pdos(0x21);
  print_tx_source_pdos(0x23);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
    /* USER CODE END WHILE */
    HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00300617;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TPS26750_1_Pin TPS26750_2_Pin */
  GPIO_InitStruct.Pin = TPS26750_1_Pin|TPS26750_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
 __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
