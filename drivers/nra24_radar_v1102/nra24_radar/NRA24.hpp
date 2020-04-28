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
 * @file nra24.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 * Driver for the Benewake nra24 laser rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/uORB.h>

#include "nra24_parser.h"

#define NRA24_DEFAULT_PORT	"/dev/ttyS2"
#define NRA24_MAX_DISTANCE	        50.0f
#define NRA24_MIN_DISTANCE	        0.315f

using namespace time_literals;

class NRA24 : public px4::ScheduledWorkItem
{
public:
	NRA24(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~NRA24();

	int init();

	void print_info();

private:

	int collect();

	void Run() override;

	void start();
	void stop();

	PX4Rangefinder	_px4_rangefinder;

	NRA24_PARSE_STATE _parse_state {NRA24_PARSE_STATE::NRA24_DECODE_INT};

	char _linebuf[29] {};
	char _port[20] {};

	static constexpr uint32_t NRA24_MEASURE_INTERVAL{10_ms};	// 25ms default sensor conversion time.

	int _fd{-1};

	unsigned int _linebuf_index{0};

	hrt_abstime _last_read{0};
	float distance_track;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
