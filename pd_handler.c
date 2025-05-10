// pd_handler.c
#include "pd_handler.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;

// Static PDO voltage table (PDO2-5)
static const uint32_t pdo_voltages[] = {8300, 14300, 19300};

static void apply_voltage(uint8_t pdAddr, uint8_t tpsAddr)
{
 //   static uint32_t last_voltage_applied = 0;

    uint8_t  devAddr = pdAddr << 1;
    uint8_t  len = 16;          // we want all 16 data bytes (128 bits)
    uint8_t  raw[1 + 16];           // [0]=length prefix, [1..16]=payload
    uint32_t rdo_req;
    uint8_t  pdo_number;

    // 1) Ask for 16 bytes of the Active RDO Contract
    HAL_I2C_Mem_Write(&hi2c3, devAddr, 0x35, I2C_MEMADD_SIZE_8BIT, &len, 1, 100);

    // 2) Read back prefix + 16 payload bytes
    HAL_I2C_Mem_Read(&hi2c3, devAddr, 0x35, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 100);

    // 3) Assemble bits 127:96 → raw[13..16]
    rdo_req  = ((uint32_t)raw[16] << 24);
    rdo_req |= ((uint32_t)raw[15] << 16);
    rdo_req |= ((uint32_t)raw[14] <<  8);
    rdo_req |= ((uint32_t)raw[13] <<  0);

    // 4) Extract PDO index from bits 31:28 of the Requested RDO
    pdo_number = (rdo_req >> 28) & 0x0F;

    printf("Requested RDO = 0x%08lX → PDO #%u\n", (unsigned long)rdo_req, pdo_number);

    // 5) Validate (only 1..4 supported by our voltage table)
    if (pdo_number < 2 || pdo_number > 4)
    {
        printf("PDO #%u out of range, defaulting to #1\n", pdo_number);
        //pdo_number = 1;
        return;
    }
    /* ---------------------------------------------------------------------------*/
    //TPS55288 configutration
    // 6) Look up target voltage and program the TPS
    uint32_t target_voltage = pdo_voltages[pdo_number - 2];
    printf("Applying PDO #%u → %lu mV on TPS 0x%02X\n", pdo_number, target_voltage, tpsAddr);

    // Convert to DAC code for TPS55288
    uint16_t ref_code = target_voltage / 20;
    if (ref_code > 0x3FF) ref_code = 0x3FF;
    uint8_t dac_buf[2] = {ref_code & 0xFF, (ref_code >> 8) & 0x03};

    // Write REF to TPS55288
    HAL_I2C_Mem_Write(&hi2c3, tpsAddr << 1, 0x00, I2C_MEMADD_SIZE_8BIT, dac_buf, 2, 100);

    // Enable OE bit (bit 7) in REG06
    uint8_t reg06;
    HAL_I2C_Mem_Read(&hi2c3, tpsAddr << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
    reg06 |= 0x80;
    HAL_I2C_Mem_Write(&hi2c3, tpsAddr << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 100);
}
/* ---------------------------------------------------------------------------*/
void handle_pd1_event(void) {
    apply_voltage(0x21, 0x75);
}

void handle_pd2_event(void) {
    apply_voltage(0x23, 0x74);
}

