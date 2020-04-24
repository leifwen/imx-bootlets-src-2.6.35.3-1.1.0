/*
 * Boot Prep common file
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

#include <stdarg.h>

#include "regspinctrl.h"
#include "regsdigctl.h"
#include "regsuartdbg.h"
//------------------------------------------------------------------------------------------//
void DDuartEnable(void){
	HW_PINCTRL_MUXSEL7_CLR(0xF000F);
	HW_PINCTRL_MUXSEL7_SET(0xF000F);

	HW_PINCTRL_MUXSEL6_CLR(0xF0);
	HW_PINCTRL_MUXSEL6_SET(0xA0);
}
//------------------------------------------------------------------------------------------//
void DDelay(unsigned int microSec){
	unsigned int currentTime = HW_DIGCTL_MICROSECONDS_RD();
	while ((HW_DIGCTL_MICROSECONDS_RD() - currentTime) <  microSec);
}
//------------------------------------------------------------------------------------------//
void DBeep(void){
	int i;
	HW_PINCTRL_MUXSEL3_SET(3 << 10);
  	HW_PINCTRL_DOE1_SET(1 << 21);

	for (i = 0; i < 100; i++) {
		HW_PINCTRL_DOUT1_SET(1 << 21);
		DDelay(500);
		HW_PINCTRL_DOUT1_CLR(1 << 21);
		DDelay(50);
	}

	HW_PINCTRL_MUXSEL3_SET(3 << 12);
	HW_PINCTRL_DOE2_SET(1 << 6);
	for (i = 0; i < 100; i++) {
		HW_PINCTRL_DOUT2_SET(1 << 6);
		DDelay(500);
		HW_PINCTRL_DOUT2_CLR(1 << 6);
		DDelay(50);
	}
}
//------------------------------------------------------------------------------------------//
void DPutc(char ch){
	int loop = 0;
	while (HW_UARTDBGFR_RD()&BM_UARTDBGFR_TXFF) {
		loop++;
		if (loop > 10000)
			break;
	};

	/* if(!(HW_UARTDBGFR_RD() &BM_UARTDBGFR_TXFF)) */
	HW_UARTDBGDR_WR(ch);
}
//------------------------------------------------------------------------------------------//
void DPrinthex(int data){
	int i = 0;
	char c;
	for (i = sizeof(int)*2-1; i >= 0; i--) {
		c = data>>(i*4);
		c &= 0xf;
		if (c > 9)
			DPutc(c-10+'A');
		else
			DPutc(c+'0');
	}
}
//------------------------------------------------------------------------------------------//
void DPrintf(char *fmt, ...){
	va_list args;
	va_start(args, fmt);
	while (*fmt) {

		if (*fmt == '%') {
			fmt++;
			switch (*fmt) {

			case 'x':
			case 'X':
				DPrinthex(va_arg(args, int));
				break;
			case '%':
				DPutc('%');
				break;
			default:
				break;
			}

		} else {
			DPutc(*fmt);
		}
		fmt++;
	}
	va_end(args);
}
//------------------------------------------------------------------------------------------//