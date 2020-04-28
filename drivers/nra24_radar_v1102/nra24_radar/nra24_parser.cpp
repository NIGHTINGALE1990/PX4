/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the NRA24 laser rangefinder series
 */

#include "nra24_parser.h"
#include <string.h>
#include <stdlib.h>

// #define NRA24_DEBUG

#ifdef NRA24_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"NRA24_DECODE_INT",
	// "NRA24_DECODE_ENDSEQ1",
	// "NRA24_DECODE_ENDSEQ2",
	"NRA24_DECODE_STARTSEQ1"，
	"NRA24_DECODE_STARTSEQ2",
	"NRA24_DECODE_MESG_ID_L",
	"NRA24_DECODE_MESG_ID_H",
	"NRA24_DECODE_INDEX",
	"NRA24_DECODE_RCS",
	"NRA24_DECODE_RANGE_H",
	"NRA24_DECODE_RANGE_L",
	"NRA24_DECODE_RSVD1",
	"NRA24_DECODE_ROLLCOUNT_A",
	"NRA24_DECODE_ROLLCOUNT_B",
	"NRA24_DECODE_CHECKSUM"
};
#endif

int nra24_parse(char c, char *parserbuf, unsigned *parserbuf_index, NRA24_PARSE_STATE *state, float *dist, int *roll_count)
{
	int ret = -1;
	//char *end;

	switch (*state) {
	case NRA24_PARSE_STATE::NRA24_DECODE_CHECKSUM:
		if (c == 0xAA) {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
		}

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_INT:
		if (c == 0xAA) {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ1:
		if (c == 0xAA) {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ2;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
			*parserbuf_index = 0;
		}

		break;

	// case NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ2:
	// 	if (c == 0xAA) {
	// 		*state = NRA24_PARSE_STATE::NRA24_DECODE_STARTSEQ1;
	// 		parserbuf[*parserbuf_index] = c;
	// 		(*parserbuf_index)++;

	// 	} else {
	// 		*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
	// 		*parserbuf_index = 0;
	// 	}

	// 	break;

	// case NRA24_PARSE_STATE::NRA24_DECODE_STARTSEQ1:
	// 	if (c == 0xAA) {
	// 		*state = NRA24_PARSE_STATE::NRA24_DECODE_STARTSEQ2;
	// 		parserbuf[*parserbuf_index] = c;
	// 		(*parserbuf_index)++;

	// 	} else {
	// 		*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
	// 		*parserbuf_index = 0;
	// 	}

	// 	break;

	case NRA24_PARSE_STATE::NRA24_DECODE_ENDSEQ2:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_MESG_ID_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_MESG_ID_L:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_MESG_ID_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_MESG_ID_H:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_INDEX;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_INDEX:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_RCS;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_RCS:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_RANGE_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_RANGE_H:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_RANGE_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_RANGE_L:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_RSVD1;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_RSVD1:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_ROLLCOUNT_A;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;


	case NRA24_PARSE_STATE::NRA24_DECODE_ROLLCOUNT_A:
		*state = NRA24_PARSE_STATE::NRA24_DECODE_ROLLCOUNT_B;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case NRA24_PARSE_STATE::NRA24_DECODE_ROLLCOUNT_B:
		// Find the checksum
		uint8_t checksum_value = 0;

		for (int i = 4; i < 11; i++) {
			checksum_value += parserbuf[i];
		}

		if (c == (checksum_value & 0xFF)) {
		parserbuf[*parserbuf_index] = '\0';

			if (parserbuf[2] == 0x0C) {

			unsigned int t1 = parserbuf[6];
			unsigned int t2 = parserbuf[7];
			unsigned int t0 = t1*256;
			*dist = ((float)t0 + (float)t2) / 100;
			*state = NRA24_PARSE_STATE::NRA24_DECODE_CHECKSUM;
			*parserbuf_index = 0;
			ret = 0;

			} else if (parserbuf[2] == 0x0B) {

			*roll_count = parserbuf[5];

			*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
			*parserbuf_index = 0;

			} else {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
			*parserbuf_index = 0;

			}

		} else {
			*state = NRA24_PARSE_STATE::NRA24_DECODE_INT;
			*parserbuf_index = 0;
		}

		break;
	}

#ifdef NRA24_DEBUG
	printf("state: NRA24_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
