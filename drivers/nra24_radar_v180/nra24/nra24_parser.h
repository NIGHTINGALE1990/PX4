/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Declarations of parser for the nra24 radar rangefinder
 */

#pragma once

/**
 *   AA   AA        0C    07     01   76   01   4C   2D  02   9B         8E     55 55
 *   AA   AA        0C    07     01   76   01   79   2D  02   BA         DA     55 55
 *                |  mesg_id     i    rcs   h    l   rs  ro   ro
 *     AA AA      |  0C   07  |  01   77   01   8F   2D  02   BC   |     F3   | 55   55
 *  0xAA 0xAA     | 0x0C 0x07 | 0x01 0xC8 0x07 0xD0 0x00 0x02 0xEE |    0x90  | 0x55 0x55
 * Start Sequence |Message ID |          Data Payload              |Check Sum | End Sequence
 * Start Sequence = 0xAAAA
 * Message ID = 0x0C + 0x07*0x100 = 0x70C
 * Data Payload = 0x01 0xC8 0x07 0xD0 0x00 0x02 0xEE
 * Check Sum = 0x90
 * End Sequence = 0x5555
 * Data Payload  各字段解析如下 ：
 * Index = 1
 * Rcs = 0xC8*0.5 – 50 = 50
 * Range = (0x07*0x100 +0xD0)*0.01 = 20 //目标距离 20m
 * Rsvd1 = 0
 * RollCount = (0x0 & 0xE0) >> 5 = 0
 * Check Sum = 0x90    //校验和为数据负载之和的低八位：0x90 = 0xFF&0x290
 *                      //0x290=0x01+0xC8+0x07+0xD0+0x00+0x02+0xEE
*/

enum NRA24_PARSE_STATE {
	NRA24_DECODE_INT = 0,
	NRA24_DECODE_ENDSEQ1,
	NRA24_DECODE_ENDSEQ2,
	NRA24_DECODE_MESG_ID_L,
	NRA24_DECODE_MESG_ID_H,
	NRA24_DECODE_INDEX,
	NRA24_DECODE_RCS,
	NRA24_DECODE_RANGE_H,
	NRA24_DECODE_RANGE_L,
	NRA24_DECODE_RSVD1,
	NRA24_DECODE_ROLLCOUNT_A,
	NRA24_DECODE_ROLLCOUNT_B,
	NRA24_DECODE_CHECKSUM
};

int nra24_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum NRA24_PARSE_STATE *state, float *dist, int *roll_count);

