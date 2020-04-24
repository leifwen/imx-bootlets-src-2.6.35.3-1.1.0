/*
 * iMX28 Boot Prep
 *
 * Copyright (c) 2008-2012 Freescale Semiconductor
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include "regsclkctrl.h"
#include "regsemi.h"
#include "regsdram.h"
#include "regspower.h"
#include "regsuartdbg.h"
#include "regspinctrl.h"
#include "regsdigctl.h"
#include "regsocotp.h"
#include <stdarg.h>
//------------------------------------------------------------------------------------------//
#define PIN_DRIVE_12mA     2
#define EMI_PIN_DRIVE_20mA 2
#define EMI_PIN_DRIVE_ADDRESS          EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_CONTROL          EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_DUALPAD          EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_DATA_SLICE_3     EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_DATA_SLICE_2     EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_DATA_SLICE_1     EMI_PIN_DRIVE_20mA
#define EMI_PIN_DRIVE_DATA_SLICE_0     EMI_PIN_DRIVE_20mA
//------------------------------------------------------------------------------------------//
void DDR2EmiController_IS43DR16320D_3DBLI_200MHz(void){
	volatile unsigned int * DRAM_REG = (volatile unsigned int*) HW_DRAM_CTL00_ADDR;
	DRAM_REG[0]  = 0x00000000;
	DRAM_REG[16] = 0x00000000;//
	DRAM_REG[21] = 0x00000000;//
	DRAM_REG[22] = 0x00000000;//
	DRAM_REG[23] = 0x00000000;//
	DRAM_REG[24] = 0x00000000;//
	DRAM_REG[25] = 0x00000000;//
	DRAM_REG[26] = 0x00010101;//
	DRAM_REG[27] = 0x01010101;//
	DRAM_REG[28] = 0x000f0f01;//
	DRAM_REG[29] = 0x0102020a;//
	DRAM_REG[31] = 0x00000101;//
	DRAM_REG[32] = 0x00000100;//
	DRAM_REG[33] = 0x00000100;//
	DRAM_REG[34] = 0x01000000;//
	DRAM_REG[35] = 0x00000002;//
	DRAM_REG[36] = 0x01010000;//
	DRAM_REG[37] = 0x07080403;//
	DRAM_REG[38] = 0x06005003;//
	DRAM_REG[39] = 0x0A0000C8;//
	DRAM_REG[40] = 0x02009C40;//
	DRAM_REG[41] = 0x0002030B;//
	DRAM_REG[42] = 0x0036B008;//
	DRAM_REG[43] = 0x03150612;//
	DRAM_REG[44] = 0x02030202;//
	DRAM_REG[45] = 0x001f001c;//
	DRAM_REG[48] = 0x00011900;//
	DRAM_REG[49] = 0xffff0303;//
	DRAM_REG[50] = 0x00012100;//
	DRAM_REG[51] = 0xffff0303;//
	DRAM_REG[52] = 0x00012100;//
	DRAM_REG[53] = 0xffff0303;//
	DRAM_REG[54] = 0x00012100;//
	DRAM_REG[55] = 0xffff0303;//
	DRAM_REG[56] = 0x00000003;//
	DRAM_REG[58] = 0x00000000;//
	DRAM_REG[66] = 0x00000612;//
	DRAM_REG[67] = 0x01000f02;//
	DRAM_REG[69] = 0x00000200;//
	DRAM_REG[70] = 0x00020007;//
	DRAM_REG[71] = 0xf4004a27;//
	DRAM_REG[72] = 0xf4004a27;//
	DRAM_REG[75] = 0x07400300;//
	DRAM_REG[76] = 0x07400300;//
	DRAM_REG[79] = 0x00000005;//
	DRAM_REG[80] = 0x00000000;//
	DRAM_REG[81] = 0x00000000;//
	DRAM_REG[82] = 0x01000000;//
	DRAM_REG[83] = 0x01020408;//
	DRAM_REG[84] = 0x08040201;//
	DRAM_REG[85] = 0x000f1133;//
	DRAM_REG[87] = 0x00001f04;//
	DRAM_REG[88] = 0x00001f04;//
	DRAM_REG[91] = 0x00001f04;//
	DRAM_REG[92] = 0x00001f04;//
	DRAM_REG[162] = 0x00010000;//
	DRAM_REG[163] = 0x00030404;//
	DRAM_REG[164] = 0x00000003;//
	DRAM_REG[171] = 0x01010000;//
	DRAM_REG[172] = 0x01000000;//
	DRAM_REG[173] = 0x03030000;//
	DRAM_REG[174] = 0x00010303;//
	DRAM_REG[175] = 0x01020202;//
	DRAM_REG[176] = 0x00000000;//
	DRAM_REG[177] = 0x02030303;//
	DRAM_REG[178] = 0x21002103;//
	DRAM_REG[179] = 0x00061200;//
	DRAM_REG[180] = 0x06120612;//
	DRAM_REG[181] = 0x04420442;//
	DRAM_REG[182] = 0x00000000;//
	DRAM_REG[183] = 0x00040004;//
	DRAM_REG[184] = 0x00000000;//
	DRAM_REG[185] = 0x00000000;//
	DRAM_REG[186] = 0x00000000;//
	DRAM_REG[187] = 0x00000000;//
	DRAM_REG[188] = 0x00000000;//
	DRAM_REG[189] = 0xffffffff;//
}
//------------------------------------------------------------------------------------------//
void poweron_pll(void){
	HW_CLKCTRL_PLL0CTRL0_SET(BM_CLKCTRL_PLL0CTRL0_POWER);
}
//------------------------------------------------------------------------------------------//
void turnon_mem_rail(int mv){
	unsigned int value;
//	HW_POWER_CTRL_CLR(BM_POWER_CTRL_CLKGATE);

	value = BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT| BM_POWER_VDDMEMCTRL_ENABLE_LINREG| BM_POWER_VDDMEMCTRL_PULLDOWN_ACTIVE| (mv-1700)/50;

	HW_POWER_VDDMEMCTRL_WR(value);
	DDelay(20000);
	value &= ~(BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT | BM_POWER_VDDMEMCTRL_PULLDOWN_ACTIVE);
	HW_POWER_VDDMEMCTRL_WR(value);
}
//------------------------------------------------------------------------------------------//
void set_emi_frac(unsigned int div){
	HW_CLKCTRL_FRAC0_SET(BM_CLKCTRL_FRAC0_EMIFRAC);
	div = (~div);
	HW_CLKCTRL_FRAC0_CLR(BF_CLKCTRL_FRAC0_EMIFRAC(div));
}
//------------------------------------------------------------------------------------------//
void init_clock(void){
	HW_CLKCTRL_FRAC0_SET(BM_CLKCTRL_FRAC0_CLKGATEEMI);

	set_emi_frac(21);

	HW_CLKCTRL_FRAC0_CLR(BM_CLKCTRL_FRAC0_CLKGATEEMI);
	DDelay(11000);

	HW_CLKCTRL_EMI_WR(BF_CLKCTRL_EMI_DIV_XTAL(1)| BF_CLKCTRL_EMI_DIV_EMI(2));

	/*choose ref_emi*/
	HW_CLKCTRL_CLKSEQ_CLR(BM_CLKCTRL_CLKSEQ_BYPASS_EMI);

	DPrintf("FRAC 0x%x\r\n" , HW_CLKCTRL_FRAC0_RD());
}
//------------------------------------------------------------------------------------------//
void disable_emi_padkeepers(void){
#if 0
    HW_PINCTRL_CTRL_CLR(BM_PINCTRL_CTRL_SFTRST | BM_PINCTRL_CTRL_CLKGATE);

    HW_PINCTRL_PULL3_SET(
    BM_PINCTRL_PULL3_BANK3_PIN17 |
    BM_PINCTRL_PULL3_BANK3_PIN16 |
    BM_PINCTRL_PULL3_BANK3_PIN15 |
    BM_PINCTRL_PULL3_BANK3_PIN14 |
    BM_PINCTRL_PULL3_BANK3_PIN13 |
    BM_PINCTRL_PULL3_BANK3_PIN12 |
    BM_PINCTRL_PULL3_BANK3_PIN11 |
    BM_PINCTRL_PULL3_BANK3_PIN10 |
    BM_PINCTRL_PULL3_BANK3_PIN09 |
    BM_PINCTRL_PULL3_BANK3_PIN08 |
    BM_PINCTRL_PULL3_BANK3_PIN07 |
    BM_PINCTRL_PULL3_BANK3_PIN06 |
    BM_PINCTRL_PULL3_BANK3_PIN05 |
    BM_PINCTRL_PULL3_BANK3_PIN04 |
    BM_PINCTRL_PULL3_BANK3_PIN03 |
    BM_PINCTRL_PULL3_BANK3_PIN02 |
    BM_PINCTRL_PULL3_BANK3_PIN01 |
    BM_PINCTRL_PULL3_BANK3_PIN00);
#endif
}
//------------------------------------------------------------------------------------------//
#define PIN_VOL(pin , v) ((v) ? (pin) : 0)
//------------------------------------------------------------------------------------------//
void init_emi_pin(int pin_voltage,int pin_drive){
#ifdef MEM_MDDR
	disable_emi_padkeepers();
	HW_PINCTRL_EMI_DS_CTRL_WR(
		BF_PINCTRL_EMI_DS_CTRL_ADDRESS_MA(EMI_PIN_DRIVE_ADDRESS)     |
		BF_PINCTRL_EMI_DS_CTRL_CONTROL_MA(EMI_PIN_DRIVE_CONTROL)     |
		BF_PINCTRL_EMI_DS_CTRL_DUALPAD_MA(EMI_PIN_DRIVE_DUALPAD)     |
		BF_PINCTRL_EMI_DS_CTRL_SLICE3_MA(EMI_PIN_DRIVE_DATA_SLICE_3) |
		BF_PINCTRL_EMI_DS_CTRL_SLICE2_MA(EMI_PIN_DRIVE_DATA_SLICE_2) |
		BF_PINCTRL_EMI_DS_CTRL_SLICE1_MA(EMI_PIN_DRIVE_DATA_SLICE_1) |
		BF_PINCTRL_EMI_DS_CTRL_SLICE0_MA(EMI_PIN_DRIVE_DATA_SLICE_0));

	/* Configure Bank-3 Pins 0-15 as EMI pins*/
	HW_PINCTRL_MUXSEL10_CLR(
		BM_PINCTRL_MUXSEL10_BANK5_PIN00 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN01 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN02 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN03 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN04 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN05 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN06 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN07 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN08 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN09 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN10 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN11 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN12 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN13 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN14 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN15);

	HW_PINCTRL_MUXSEL11_CLR(
		BM_PINCTRL_MUXSEL11_BANK5_PIN16 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN17 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN18 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN19 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN20 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN21 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN22 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN23 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN26 );

	HW_PINCTRL_MUXSEL12_CLR(
		BM_PINCTRL_MUXSEL12_BANK6_PIN00 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN01 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN02 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN03 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN04 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN05 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN06 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN07 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN08 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN09 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN10 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN11 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN12 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN13 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN14 );

	HW_PINCTRL_MUXSEL13_CLR(
		BM_PINCTRL_MUXSEL13_BANK6_PIN16 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN17 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN18 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN19 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN20 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN21 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN22 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN23 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN24 );
#else
	/* Configure Bank-3 Pins 0-15 as EMI pins*/
	HW_PINCTRL_MUXSEL10_CLR(
		BM_PINCTRL_MUXSEL10_BANK5_PIN00 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN01 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN02 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN03 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN04 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN05 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN06 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN07 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN08 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN09 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN10 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN11 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN12 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN13 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN14 |
		BM_PINCTRL_MUXSEL10_BANK5_PIN15);

	HW_PINCTRL_MUXSEL11_CLR(
		BM_PINCTRL_MUXSEL11_BANK5_PIN16 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN17 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN18 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN19 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN20 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN21 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN22 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN23 |
		BM_PINCTRL_MUXSEL11_BANK5_PIN26 );

	HW_PINCTRL_MUXSEL12_CLR(
		BM_PINCTRL_MUXSEL12_BANK6_PIN00 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN01 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN02 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN03 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN04 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN05 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN06 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN07 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN08 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN09 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN10 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN11 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN12 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN13 |
		BM_PINCTRL_MUXSEL12_BANK6_PIN14 );

	HW_PINCTRL_MUXSEL13_CLR(
		BM_PINCTRL_MUXSEL13_BANK6_PIN16 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN17 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN18 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN19 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN20 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN21 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN22 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN23 |
		BM_PINCTRL_MUXSEL13_BANK6_PIN24 );
#endif
}
//------------------------------------------------------------------------------------------//
void exit_selfrefresh(void){
	unsigned int start;
	unsigned int value;
	value = HW_DRAM_CTL16_RD();
	value &= ~(1<<17);
	HW_DRAM_CTL16_WR(value);

	start = HW_DIGCTL_MICROSECONDS_RD();

	while ((HW_EMI_STAT_RD()&BM_EMI_STAT_DRAM_HALTED)) {
		if (HW_DIGCTL_MICROSECONDS_RD() > (start + 1000000)) {
			DPrintf("Exit self refresh timeout\n");
			return;
		}
	}
}
//------------------------------------------------------------------------------------------//
void set_port_priority(void){
	HW_EMI_CTRL_CLR(BM_EMI_CTRL_PORT_PRIORITY_ORDER);
	HW_EMI_CTRL_SET(BF_EMI_CTRL_PORT_PRIORITY_ORDER( BV_EMI_CTRL_PORT_PRIORITY_ORDER__PORT1230));

	HW_EMI_CTRL_CLR(BM_EMI_CTRL_PORT_PRIORITY_ORDER);
	HW_EMI_CTRL_SET(BF_EMI_CTRL_PORT_PRIORITY_ORDER(0x2));
}
//------------------------------------------------------------------------------------------//
void entry_auto_clock_gate(void){
	unsigned int value;
	value =  HW_DRAM_CTL16_RD();
	value |= 1<<19;
	HW_DRAM_CTL16_WR(value);

	value =  HW_DRAM_CTL16_RD();
	value |= 1<<11;
	HW_DRAM_CTL16_WR(value);
}
//------------------------------------------------------------------------------------------//
void change_cpu_freq(void){
	int value = HW_POWER_VDDDCTRL_RD();

	DPrintf("power 0x%x\r\n" , value);
    value &= ~BM_POWER_VDDDCTRL_TRG;
	value |= BF_POWER_VDDDCTRL_TRG(28);
    value &= ~BM_POWER_VDDDCTRL_BO_OFFSET;
	value |= BF_POWER_VDDDCTRL_BO_OFFSET(7);
    value &= ~BM_POWER_VDDDCTRL_LINREG_OFFSET;
	value |= BF_POWER_VDDDCTRL_LINREG_OFFSET(2);
	//value |= BM_POWER_VDDDCTRL_ENABLE_LINREG;
	//value |= BM_POWER_VDDDCTRL_DISABLE_STEPPING;

	HW_POWER_VDDDCTRL_WR(value); /*change to 1.50v*/

	DDelay(10000);

	DPrintf("Frac 0x%x\n", HW_CLKCTRL_FRAC0_RD());

	value = HW_CLKCTRL_FRAC0_RD();
	value &= ~BM_CLKCTRL_FRAC0_CPUFRAC;
	value |= BF_CLKCTRL_FRAC0_CPUFRAC(19);
	value &= ~BM_CLKCTRL_FRAC0_CLKGATECPU;

	HW_CLKCTRL_FRAC0_WR(value); /*Change cpu to 360Mhz*/

	HW_CLKCTRL_CLKSEQ_SET(BM_CLKCTRL_CLKSEQ_BYPASS_CPU);

	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_DIV);
	HW_CLKCTRL_HBUS_CLR(((~3)&BM_CLKCTRL_HBUS_DIV));

	DDelay(10000);
	DPrintf("Change cpu freq\n");

	value = HW_CLKCTRL_CPU_RD();
	value &= ~BM_CLKCTRL_CPU_DIV_CPU;
	value |=1;
	HW_CLKCTRL_CPU_WR(value);

	HW_CLKCTRL_CLKSEQ_CLR(BM_CLKCTRL_CLKSEQ_BYPASS_CPU);

	DPrintf("hbus 0x%x\n" , HW_CLKCTRL_HBUS_RD());
	DPrintf("cpu 0x%x\n" , HW_CLKCTRL_CPU_RD());
}
//------------------------------------------------------------------------------------------//
void poweron_vdda(void){
    int value = HW_POWER_VDDACTRL_RD();
    value &= ~BM_POWER_VDDACTRL_TRG;
    value |= BF_POWER_VDDACTRL_TRG(0xC);
    value &= ~BM_POWER_VDDACTRL_BO_OFFSET;
    value |= BF_POWER_VDDACTRL_BO_OFFSET(6);
    value &= ~BM_POWER_VDDACTRL_LINREG_OFFSET;
    value |= BF_POWER_VDDACTRL_LINREG_OFFSET(2);

    HW_POWER_VDDACTRL_WR(value);
}
//------------------------------------------------------------------------------------------//
void DDR2_Init(void){
	unsigned int value;

	value = HW_DRAM_CTL16_RD();
	value &= ~BM_DRAM_CTL16_START;
	HW_DRAM_CTL16_WR(value);

	DPrintf("Call DDR2EmiController_IS43DR16320D_3DBLI_200MHz(),8M*16*4 DDR2\n");
	DDR2EmiController_IS43DR16320D_3DBLI_200MHz();

	value = HW_DRAM_CTL17_RD();
	value &= ~BM_DRAM_CTL17_SREFRESH;
	HW_DRAM_CTL17_WR(value);

	value = HW_DRAM_CTL16_RD();
	value |= BM_DRAM_CTL16_START;
	HW_DRAM_CTL16_WR(value);

	DPrintf("Wait for ddr ready\n");
	while(!(HW_DRAM_CTL58_RD()&0x100000));
}
//------------------------------------------------------------------------------------------//
void MemTest(void){
	int blret,i;
	volatile int *pTest = 0x40000000;
	DPrintf("Test memory accress\n");
	DPrintf("ddr2 0x%x\r\n", pTest);
	for (i = 0; i < 1000; i++)
		*pTest++ = i;

	pTest = (volatile int *)0x40000000;
	blret = 1;
	for (i = 0; i < 1000; i++) {
		if (*pTest != (i)) {
			DPrintf("0x%x error value 0x%x\r\n", i, *pTest);
			blret = 0;
		}
		pTest++;
	}
	DPrintf("Finish simple test,");
	if (blret == 0){
		DPrintf("fail\n");
	}
	else{
		DPrintf("success\n");
	}
}
//------------------------------------------------------------------------------------------//
int _start(int arg){
	DDuartEnable();
	DPrintf("----------------------------------------------------------------------\n");
	DPrintf("BootPrep start\n");

#ifdef MEM_MDDR
	/* set to mddr mode*/
	HW_PINCTRL_EMI_DS_CTRL_CLR(BW_PINCTRL_EMI_DS_CTRL_DDR_MODE(0x3));
#else
	/* set to ddr2 mode*/
	HW_PINCTRL_EMI_DS_CTRL_SET(BW_PINCTRL_EMI_DS_CTRL_DDR_MODE(0x3));
#endif
	poweron_pll();
	DDelay(11000);

	init_emi_pin(0,PIN_DRIVE_12mA);

	init_clock();

	DDelay(10000);

	poweron_vdda();

	DDR2_Init();
#if 0
	exit_selfrefresh();
	set_port_priority();
	entry_auto_clock_gate();
#endif
	change_cpu_freq();
#if 0
	for (i = 0; i <= 40; i++) {
		DPrintf("mem %x - 0x%x\r\n",
			i, *(volatile int*)(0x800E0000 + i * 4));
	}
#endif
	MemTest();
	DPrintf("BootPrep finish\n");
	DPrintf("----------------------------------------------------------------------\n");
	DBeep();
	return 0;
}
//------------------------------------------------------------------------------------------//
/* kiss gcc's ass to make it happy */
void __aeabi_unwind_cpp_pr0() {}
void __aeabi_unwind_cpp_pr1() {}
