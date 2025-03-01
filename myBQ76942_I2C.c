/*
 * BQ76942_I2C.c
 *
 *  Created on: Dec 20, 2024
 *      Author: ADMIN
 */

#include "myBQ76942_I2C.h"
//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage[16] = {0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float Temperature[3]     = {0, 0, 0};
uint16_t Stack_Voltage   = 0x00;
uint16_t Pack_Voltage    = 0x00;
uint16_t LD_Voltage      = 0x00;
int16_t Pack_Current     = 0x00;
uint16_t AlarmBits       = 0x00;

uint16_t cellActiveBalancing;
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;      // Permanent Fail Status Register A
uint8_t value_PFStatusB;      // Permanent Fail Status Register B
uint8_t value_PFStatusC;      // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t UV_Fault             = 0;  // under-voltage fault state
uint8_t OV_Fault             = 0;  // over-voltage fault state
uint8_t SCD_Fault            = 0;  // short-circuit fault state
uint8_t OCD_Fault            = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0;  // Set to 1 if any protection triggers

uint8_t LD_ON = 0;  // Load Detect status bit
uint8_t DSG   = 0;  // discharge FET state
uint8_t CHG   = 0;  // charge FET state
uint8_t PCHG  = 0;  // pre-charge FET state
uint8_t PDSG  = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int;   // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Frac;  // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Time;  // in AFE_READPASSQ func

unsigned char RX_32Byte[32] = {0x00};
unsigned char RX_data[2]    = {0x00};

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for (i = 0; i < len; i++) checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return (checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
    unsigned char i;
    unsigned char crc = 0;
    while (len-- != 0) {
        for (i = 0x80; i != 0; i /= 2) {
            if ((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= 0x107;
            } else
                crc *= 2;

            if ((*ptr & i) != 0) crc ^= 0x107;
        }
        ptr++;
    }
    return (crc);
}

void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    uint8_t I2Ctxbuff[9] = {0x00};
    I2Ctxbuff[0] = reg_addr;
    for (uint8_t i = 0; i < count; i++) {
        I2Ctxbuff[i + 1] = reg_data[i];
    }
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Handler, \
    		(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Address_Write), I2Ctxbuff, count + 1, 100);
    // Checking the status of Communication - I2C
    if (status != HAL_OK) {
        // Address faults or errors if needed
        HAL_I2C_Master_Abort_IT(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Handler, \
        		(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Address_Write)); // Cancle the communications if having errors
    }
    // Assure the I2C communications to be free
    while (HAL_I2C_GetState(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Handler) != HAL_I2C_STATE_READY)
        ;
}

void I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    HAL_StatusTypeDef status;

    // Send the address of register needed to be read
    status = HAL_I2C_Master_Transmit(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Handler, \
    		I2C_BQ796x2_CfgParam.I2C_BQ796x2_Address_Write, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        // Handling errors if needed
        return;
    }
    // Reading the data in Read mode
    status = HAL_I2C_Master_Receive(I2C_BQ796x2_CfgParam.I2C_BQ796x2_Handler, \
    		I2C_BQ796x2_CfgParam.I2C_BQ796x2_Address_Read, reg_data, count, 100);
    if (status != HAL_OK) {
        // Handling errors if needed
        return;
    }
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{  //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {                       //Read
        I2C_ReadReg(command, RX_data, 2);  //RX_data is a global variable
        //        delay_cycles(64000);delay_cycles(64000);
        //        delay_cycles(40000); // for 400k Test
        HAL_Delay(2);
        HAL_Delay(2);  //success in 100k
    }
    if (type == W) {  //write
        //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command, TX_data, 2);
        HAL_Delay(2);
        HAL_Delay(2);
    }
}

void CommandSubcommands(uint16_t command)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    I2C_WriteReg(0x3E, TX_Reg, 2);
    HAL_Delay(2);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R) {  //read
        I2C_WriteReg(0x3E, TX_Reg, 2);
        HAL_Delay(2);
        I2C_ReadReg(0x40, RX_32Byte, 32);  //RX_32Byte is a global variable
    } else if (type == W) {
        //FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 3);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        HAL_Delay(1);
    } else if (type == W2) {  //write data with 2 bytes
        //CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 4);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        HAL_Delay(1);
    }
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2]  = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff;  //1st byte of data

    switch (datalen) {
        case 1:  //1 byte datalength
            I2C_WriteReg(0x3E, TX_RegData, 3);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] =
                0x05;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            HAL_Delay(2);
            break;
        case 2:  //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 4);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] =
                0x06;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            HAL_Delay(2);
            break;
        case 4:  //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 6);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] =
                0x08;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            HAL_Delay(2);
            break;
    }
}

//************************************BQ769X2 Functions*********************************
void BQ769x2_Init_Original()
{
    // Configures all parameters in device RAM

    // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
    // See TRM for full description of CONFIG_UPDATE mode
    CommandSubcommands(SET_CFGUPDATE);
    HAL_Delay(8);

    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    //BQ769x2_SetRegister(PowerConfig, 0x2D80, 2); -> Original
    BQ769x2_SetRegister(PowerConfig, 0x2982, 2);
    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    BQ769x2_SetRegister(REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    BQ769x2_SetRegister(REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    BQ769x2_SetRegister(TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    BQ769x2_SetRegister(TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    BQ769x2_SetRegister(HDQPinConfig, 0x00,1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // If you connect all J23 and J14 on EVM, enable all cells(all is in simulation mode).
    BQ769x2_SetRegister(VCellMode, 0xFFFF, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    //BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1); -> Original
    BQ769x2_SetRegister(EnabledProtectionsA, 0x88, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    // BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1); -> Original
    BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    BQ769x2_SetRegister(BalancingConfiguration, 0x0F, 1);

    // If you connect all J23 and J14 on EVM, enable all cells(all is in simulation mode). Every cell voltage is 750mV
    // Set the minimum cell balance voltage in charge - 0x933B = 750_mV-25 mV
    BQ769x2_SetRegister(CellBalanceMinCellVCharge, 2500, 2);
    BQ769x2_SetRegister(CellBalanceMinDeltaCharge, 20, 2);

    //BQ769x2_SetRegister(CellBalanceMinCellVCharge, 3900, 2);
    // Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    BQ769x2_SetRegister(CellBalanceMinCellVRelax, 2500, 2);
    // BQ769x2_SetRegister(CellBalanceMinCellVRelax, 3900, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x0D (657.8 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    //FIXME: BQ769x2_SetRegister(CUVThreshold, 0x0D, 1); -> Original
    BQ769x2_SetRegister(CUVThreshold, 0x32, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x10 (809.6 mV)
    // COV Threshold is this value multiplied by 50.6mV
    // FIXME:     BQ769x2_SetRegister(COVThreshold, 0x10, 1); - Original
    BQ769x2_SetRegister(COVThreshold, 0xFF, 1); // 0x64 -> 5V

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x01 (2 A) (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    //Fixme: Original: BQ769x2_SetRegister(OCCThreshold, 0x01, 1);
    BQ769x2_SetRegister(OCCThreshold, 0x10, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = -1 (-2 A) (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    //Fixme: Original: BQ769x2_SetRegister(OCD1Threshold, 0xFF, 1);
    BQ769x2_SetRegister(OCD1Threshold, 0xFF, 1);

    // Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    //Fixme: Original: BQ769x2_SetRegister(SCDThreshold, 0x01, 1);
    BQ769x2_SetRegister(SCDThreshold, 0x19, 1);	// For 5mOhm

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 ï¿½s; min value of 1
    BQ769x2_SetRegister(SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);
    // Set min cell Balance

    HAL_Delay(8);
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    CommandSubcommands(EXIT_CFGUPDATE);
    HAL_Delay(8);
    CommandSubcommands(FET_ENABLE);
    HAL_Delay(8);
    CommandSubcommands(ALL_FETS_ON);
    HAL_Delay(8);
    CommandSubcommands(SLEEP_DISABLE);
    HAL_Delay(8);

#define CellBalanceMinCellVCharge 0x933B      //Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
#define CellBalanceMinDeltaCharge 0x933D      //Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
#define CellBalanceStopDeltaCharge 0x933E      //Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
#define CellBalanceMinCellVRelax 0x933F      //Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
#define CellBalanceMinDeltaRelax 0x9341      //Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
#define CellBalanceStopDeltaRelax 0x9342
}

void BQ769x2_Init() {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	// See TRM for full description of CONFIG_UPDATE mode
	CommandSubcommands(SET_CFGUPDATE);

	// After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
	// programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
	// An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
	// a full description of the register and the bits will pop up on the screen.

	// 'Power Config' - 0x9234 = 0x2D80
	// Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
  	// Set wake speed bits to 00 for best performance
	BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);

	// 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	BQ769x2_SetRegister(REG0Config, 0x01, 1);

	// 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
	BQ769x2_SetRegister(REG12Config, 0x0D, 1);

	// Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);

	// Set up ALERT Pin - 0x92FC = 0x2A
	// This configures the ALERT pin to drive high (REG1 voltage) when enabled.
	// The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
	BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

	// Set TS1 to measure Cell Temperature - 0x92FD = 0x07
	BQ769x2_SetRegister(TS1Config, 0x07, 1);

	// Set TS3 to measure FET Temperature - 0x92FF = 0x0F
	BQ769x2_SetRegister(TS3Config, 0x0F, 1);

	// Set HDQ to measure Cell Temperature - 0x9300 = 0x07
	BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

	// 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
	BQ769x2_SetRegister(VCellMode, 0x0000, 2);

	// Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
	// Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
	// COV (over-voltage), CUV (under-voltage)
	//BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);
	BQ769x2_SetRegister(EnabledProtectionsA, 0x00, 1);
	// Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
	// Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
	// OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
	//BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);
	BQ769x2_SetRegister(EnabledProtectionsB, 0x00, 1);
	// 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
	BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);

	// Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
	// Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
	BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);

	// Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
	// CUV Threshold is this value multiplied by 50.6mV
	BQ769x2_SetRegister(CUVThreshold, 0x31, 1);

	// Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
	// COV Threshold is this value multiplied by 50.6mV
	BQ769x2_SetRegister(COVThreshold, 0x55, 1);

	// Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
	BQ769x2_SetRegister(OCCThreshold, 0x05, 1);

	// Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);

	// Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
	BQ769x2_SetRegister(SCDThreshold, 0x05, 1);

	// Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 µs; min value of 1
	BQ769x2_SetRegister(SCDDelay, 0x03, 1);

	// Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
	// If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
	BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	CommandSubcommands(EXIT_CFGUPDATE);
    HAL_Delay(8);
    CommandSubcommands(FET_ENABLE);
    HAL_Delay(8);
    CommandSubcommands(ALL_FETS_ON);
    HAL_Delay(8);
    CommandSubcommands(SLEEP_DISABLE);
    HAL_Delay(8);
}

// ********************************* BQ769x2 Status and Fault Commands   *****************************************

void BQ769x2_ReadAlarmStatus()
{
    // Read this register to find out why the ALERT pin was asserted
    DirectCommands(AlarmStatus, 0x00, R);
    AlarmBits = (uint16_t) RX_data[1] * 256 + (uint16_t) RX_data[0];
}

void BQ769x2_ReadSafetyStatus()
{
    // Read Safety Status A/B/C and find which bits are set
    // This shows which primary protections have been triggered
    DirectCommands(SafetyStatusA, 0x00, R);
    value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
    //Example Fault Flags
    UV_Fault  = ((0x4 & RX_data[0]) >> 2);
    OV_Fault  = ((0x8 & RX_data[0]) >> 3);
    SCD_Fault = ((0x8 & RX_data[1]) >> 3);
    OCD_Fault = ((0x2 & RX_data[1]) >> 1);
    DirectCommands(SafetyStatusB, 0x00, R);
    value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);
    DirectCommands(SafetyStatusC, 0x00, R);
    value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);
    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) >
        1) {
        ProtectionsTriggered = 1;
    } else {
        ProtectionsTriggered = 0;
    }
}

void BQ769x2_ReadPFStatus()
{
    // Read Permanent Fail Status A/B/C and find which bits are set
    // This shows which permanent failures have been triggered
    DirectCommands(PFStatusA, 0x00, R);
    value_PFStatusA = ((RX_data[1] << 8) + RX_data[0]);
    DirectCommands(PFStatusB, 0x00, R);
    value_PFStatusB = ((RX_data[1] << 8) + RX_data[0]);
    DirectCommands(PFStatusC, 0x00, R);
    value_PFStatusC = ((RX_data[1] << 8) + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************

uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
    //RX_data is global var
    DirectCommands(command, 0x00, R);
    if (command >= Cell1Voltage &&
        command <= Cell16Voltage) {  //Cells 1 through 16 (0x14 to 0x32)
        return (RX_data[1] * 256 + RX_data[0]);  //voltage is reported in mV
    } else {                                     //stack, Pack, LD
        return 10 * (RX_data[1] * 256 +
                        RX_data[0]);  //voltage is reported in 0.01V units
    }
}

void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    unsigned char x;
    int cellvoltageholder = Cell1Voltage;  //Cell1Voltage is 0x14
    for (x = 0; x < 9; x++) {             //Reads all cell voltages
        CellVoltage[x]    = BQ769x2_ReadVoltage(cellvoltageholder);
        cellvoltageholder = cellvoltageholder + 2;
    }
    Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
    Pack_Voltage  = BQ769x2_ReadVoltage(PACKPinVoltage);
    LD_Voltage    = BQ769x2_ReadVoltage(LDPinVoltage);
}

void BQ769x2_ReadCurrent()
// Reads PACK current
{
    DirectCommands(CC2Current, 0x00, R);
    Pack_Current =
        (int16_t)((uint16_t) RX_data[1] * 256 +
                  (uint16_t) RX_data[0]);  // current is reported in mA
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);
    //RX_data is a global var
    return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) -
           273.15;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadAllTemperatures()
{
    Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
    Temperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
    Temperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
}

void BQ769x2_ReadPassQ()
{  // Read Accumulated Charge and Time from DASTATUS6
    Subcommands(DASTATUS6, 0x00, R);
    AccumulatedCharge_Int  = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16) +
                             (RX_32Byte[1] << 8) + RX_32Byte[0]);  //Bytes 0-3
    AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16) +
                              (RX_32Byte[5] << 8) + RX_32Byte[4]);  //Bytes 4-7
    AccumulatedCharge_Time =
        ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16) + (RX_32Byte[9] << 8) +
            RX_32Byte[8]);  //Bytes 8-11
}

void BQ769x2_EnableAllFETs()  // Enable FET and Trun on all FET
{
    CommandSubcommands(FET_ENABLE);
    HAL_Delay(1);
    CommandSubcommands(ALL_FETS_ON);
    HAL_Delay(1);
}

//************************************Command with input variables******************************************

void _Battery_Reading_cellVoltage(Battery_TypeDef *thisBattery)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    unsigned char x;
    int cellvoltageholder = Cell1Voltage;  //Cell1Voltage is 0x14
    for (x = 0; x < thisBattery->numberOfCell; x++) {             //Reads all cell voltages
        thisBattery->cellVoltage[x]    = BQ769x2_ReadVoltage(cellvoltageholder);
        cellvoltageholder = cellvoltageholder + 2;
    }
}

// Reads charger voltage from Charger
void _Battery_Reading_chargerVoltage(Battery_TypeDef *thisBattery)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    thisBattery->chargerVoltage  = BQ769x2_ReadVoltage(PACKPinVoltage);
}

// Reads total battery voltage
void _Battery_Reading_batVoltage(Battery_TypeDef *thisBattery)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
	thisBattery->batVoltage = BQ769x2_ReadVoltage(StackVoltage);
}

// Reads current through battery
void _Battery_Reading_batCurrent(Battery_TypeDef *thisBattery)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    DirectCommands(CC2Current, 0x00, R);
    thisBattery->batCurrent =
        (int16_t)((uint16_t) RX_data[1] * 256 +
                  (uint16_t) RX_data[0]);  // current is reported in mA
}

// Reads all temperatures in BMS circuit
void _Battery_Reading_batTemperature(Battery_TypeDef *thisBattery)
{
    thisBattery->batTemperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
    thisBattery->batTemperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
    thisBattery->batTemperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
}

// Find the maximum cell voltage
void _Battery_Reading_cellVoltageMaxMin(Battery_TypeDef *thisBattery){
    uint16_t cellVoltage_max = thisBattery->cellVoltage[0]; // Khởi tạo max bằng phần tử đầu tiên
    uint16_t cellVoltage_min = thisBattery->cellVoltage[0]; // Khởi tạo min bằng phần tử đầu tiên

    for (int i = 1; i < thisBattery->numberOfCell; i++) {
        if (thisBattery->cellVoltage[i] > cellVoltage_max) {
        	cellVoltage_max = thisBattery->cellVoltage[i]; // Cập nhật max nếu phần tử lớn hơn
        }
        if (thisBattery->cellVoltage[i] < cellVoltage_min) {
            cellVoltage_min = thisBattery->cellVoltage[i]; // Cập nhật min nếu phần tử nhỏ hơn
        }
    }
    thisBattery->maxCellVoltage = cellVoltage_max;
    thisBattery->minCellVoltage = cellVoltage_min;
}

// Find the maximum delta cell voltages
void _Battery_Reading_deltacellVoltageMax(Battery_TypeDef *thisBattery){
	thisBattery->deltaCellVoltage = thisBattery->maxCellVoltage - thisBattery->minCellVoltage;
}

//************************************End of BQ769x2 Measurement Commands******************************************
void BQ769x2_Measure_Variables(){
	BQ769x2_ReadFETStatus();
	HAL_Delay(10);
    BQ769x2_ReadAlarmStatus();  // write 0x62, read 2 bytes.
    HAL_Delay(10);
    BQ769x2_ReadSafetyStatus();  // write 0x03, read 2 bytes. write 0x05, read 2 bytes. write 0x07, read 2 bytes.
    HAL_Delay(10);
    BQ769x2_ReadPFStatus();  // write 0x0B, read 2 bytes. write 0x0D, read 2 bytes. write 0x0F, read 2 bytes.
    HAL_Delay(10);
    BQ769x2_ReadAllVoltages();  // write 0x14, read 2 bytes. From 0x14 - 0x38
    //HAL_Delay(10);
    BQ769x2_ReadCurrent();  // write 0x3A, read 2 bytes.
    //HAL_Delay(10);
    BQ769x2_ReadAllTemperatures();  // write 0x70, read 2 bytes. write 0x72, read 2 bytes. write 0x74, read 2 bytes.
    //HAL_Delay(10);
   BQ769x2_ReadPassQ();  // write 0x3E with 0x0076, read 0x40 with 32 bytes.
    //HAL_Delay(10);
}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_ReadFETStatus() {
	// Read FET Status to see which FETs are enabled
	DirectCommands(FETStatus, 0x00, R);
	FET_Status = (RX_data[1]*256 + RX_data[0]);
	DSG = ((0x4 & RX_data[0])>>2);// discharge FET state
  	CHG = (0x1 & RX_data[0]);// charge FET state
  	PCHG = ((0x2 & RX_data[0])>>1);// pre-charge FET state
  	PDSG = ((0x8 & RX_data[0])>>3);// pre-discharge FET state
}

void BQ769x2_ReadBalancingActive()
// Reads no of cells undergoing balancing.
{
	DirectCommands(CB_ACTIVE_CELLS, 0x00, R);
	cellActiveBalancing = (RX_data[1]*256 + RX_data[0]);
	return;
}








