/*
 * Copyright (c) 2009-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#ifndef _REGS_H
#define _REGS_H  1


//
// define base address of the register block only if it is not already
// defined, which allows the compiler to override at build time for
// users who've mapped their registers to locations other than the
// physical location
//

#ifndef REGS_BASE
#define REGS_BASE 0x80000000
#endif


//
// common register types
//

#ifndef __LANGUAGE_ASM__
typedef unsigned char  reg8_t;
typedef unsigned short reg16_t;
typedef unsigned int   reg32_t;
#endif


//
// macros for single instance registers
//

#define BF_SET(reg, field)       HW_##reg##_SET(BM_##reg##_##field)
#define BF_CLR(reg, field)       HW_##reg##_CLR(BM_##reg##_##field)
#define BF_TOG(reg, field)       HW_##reg##_TOG(BM_##reg##_##field)

#define BF_SETV(reg, field, v)   HW_##reg##_SET(BF_##reg##_##field(v))
#define BF_CLRV(reg, field, v)   HW_##reg##_CLR(BF_##reg##_##field(v))
#define BF_TOGV(reg, field, v)   HW_##reg##_TOG(BF_##reg##_##field(v))

#define BV_FLD(reg, field, sym)  BF_##reg##_##field(BV_##reg##_##field##__##sym)
#define BV_VAL(reg, field, sym)  BV_##reg##_##field##__##sym

#define BF_RD(reg, field)        HW_##reg.B.field
#define BF_WR(reg, field, v)     BW_##reg##_##field(v)

#define BF_CS1(reg, f1, v1)  \
        (HW_##reg##_CLR(BM_##reg##_##f1),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1)))

#define BF_CS2(reg, f1, v1, f2, v2)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2)))

#define BF_CS3(reg, f1, v1, f2, v2, f3, v3)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3)))

#define BF_CS4(reg, f1, v1, f2, v2, f3, v3, f4, v4)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3 |      \
                        BM_##reg##_##f4),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3) |  \
                        BF_##reg##_##f4(v4)))

#define BF_CS5(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3 |      \
                        BM_##reg##_##f4 |      \
                        BM_##reg##_##f5),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3) |  \
                        BF_##reg##_##f4(v4) |  \
                        BF_##reg##_##f5(v5)))

#define BF_CS6(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3 |      \
                        BM_##reg##_##f4 |      \
                        BM_##reg##_##f5 |      \
                        BM_##reg##_##f6),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3) |  \
                        BF_##reg##_##f4(v4) |  \
                        BF_##reg##_##f5(v5) |  \
                        BF_##reg##_##f6(v6)))

#define BF_CS7(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3 |      \
                        BM_##reg##_##f4 |      \
                        BM_##reg##_##f5 |      \
                        BM_##reg##_##f6 |      \
                        BM_##reg##_##f7),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3) |  \
                        BF_##reg##_##f4(v4) |  \
                        BF_##reg##_##f5(v5) |  \
                        BF_##reg##_##f6(v6) |  \
                        BF_##reg##_##f7(v7)))

#define BF_CS8(reg,f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8)  \
        (HW_##reg##_CLR(BM_##reg##_##f1 |      \
                        BM_##reg##_##f2 |      \
                        BM_##reg##_##f3 |      \
                        BM_##reg##_##f4 |      \
                        BM_##reg##_##f5 |      \
                        BM_##reg##_##f6 |      \
                        BM_##reg##_##f7 |      \
                        BM_##reg##_##f8),      \
         HW_##reg##_SET(BF_##reg##_##f1(v1) |  \
                        BF_##reg##_##f2(v2) |  \
                        BF_##reg##_##f3(v3) |  \
                        BF_##reg##_##f4(v4) |  \
                        BF_##reg##_##f5(v5) |  \
                        BF_##reg##_##f6(v6) |  \
                        BF_##reg##_##f7(v7) |  \
                        BF_##reg##_##f8(v8)))


//
// macros for multiple instance registers
//

#define BF_SETn(reg, n, field)       HW_##reg##_SET(n, BM_##reg##_##field)
#define BF_CLRn(reg, n, field)       HW_##reg##_CLR(n, BM_##reg##_##field)
#define BF_TOGn(reg, n, field)       HW_##reg##_TOG(n, BM_##reg##_##field)

#define BF_SETVn(reg, n, field, v)   HW_##reg##_SET(n, BF_##reg##_##field(v))
#define BF_CLRVn(reg, n, field, v)   HW_##reg##_CLR(n, BF_##reg##_##field(v))
#define BF_TOGVn(reg, n, field, v)   HW_##reg##_TOG(n, BF_##reg##_##field(v))

#define BV_FLDn(reg,n,field,sym)  BF_##reg##_##field(\
	BV_##reg##_##field##__##sym)
#define BV_VALn(reg, n, field, sym)  BV_##reg##_##field##__##sym

#define BF_RDn(reg, n, field)        HW_##reg(n).B.field
#define BF_WRn(reg, n, field, v)     BW_##reg##_##field(n, v)

#define BF_CS1n(reg, n, f1, v1)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1))))

#define BF_CS2n(reg, n, f1, v1, f2, v2)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2))))

#define BF_CS3n(reg, n, f1, v1, f2, v2, f3, v3)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3))))

#define BF_CS4n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3 |       \
                            BM_##reg##_##f4)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3) |   \
                            BF_##reg##_##f4(v4))))

#define BF_CS5n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3 |       \
                            BM_##reg##_##f4 |       \
                            BM_##reg##_##f5)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3) |   \
                            BF_##reg##_##f4(v4) |   \
                            BF_##reg##_##f5(v5))))

#define BF_CS6n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3 |       \
                            BM_##reg##_##f4 |       \
                            BM_##reg##_##f5 |       \
                            BM_##reg##_##f6)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3) |   \
                            BF_##reg##_##f4(v4) |   \
                            BF_##reg##_##f5(v5) |   \
                            BF_##reg##_##f6(v6))))

#define BF_CS7n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6,v6,f7,v7)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3 |       \
                            BM_##reg##_##f4 |       \
                            BM_##reg##_##f5 |       \
                            BM_##reg##_##f6 |       \
                            BM_##reg##_##f7)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3) |   \
                            BF_##reg##_##f4(v4) |   \
                            BF_##reg##_##f5(v5) |   \
                            BF_##reg##_##f6(v6) |   \
                            BF_##reg##_##f7(v7))))

#define BF_CS8n(reg, n, f1, v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8)  \
        (HW_##reg##_CLR(n, (BM_##reg##_##f1 |       \
                            BM_##reg##_##f2 |       \
                            BM_##reg##_##f3 |       \
                            BM_##reg##_##f4 |       \
                            BM_##reg##_##f5 |       \
                            BM_##reg##_##f6 |       \
                            BM_##reg##_##f7 |       \
                            BM_##reg##_##f8)),      \
         HW_##reg##_SET(n, (BF_##reg##_##f1(v1) |   \
                            BF_##reg##_##f2(v2) |   \
                            BF_##reg##_##f3(v3) |   \
                            BF_##reg##_##f4(v4) |   \
                            BF_##reg##_##f5(v5) |   \
                            BF_##reg##_##f6(v6) |   \
                            BF_##reg##_##f7(v7) |   \
                            BF_##reg##_##f8(v8))))



#endif // _REGS_H

////////////////////////////////////////////////////////////////////////////////
