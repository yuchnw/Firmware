/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "WorkItem.hpp"

#include "WorkQueue.hpp"

#include <px4_log.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkItem::WorkItem()
{
	_perf_cycle_time = perf_alloc(PC_ELAPSED, "wq_cycle_run_time");
	_perf_latency = perf_alloc(PC_ELAPSED, "wq_run_latency");
	_perf_interval = perf_alloc(PC_INTERVAL, "wq_run_interval");

	if (!Init()) {
		PX4_DEBUG("init fail");
	}
}

WorkItem::~WorkItem()
{
	perf_free(_perf_cycle_time);
	perf_free(_perf_latency);
	perf_free(_perf_interval);
}

bool WorkItem::Init()
{
	px4::WorkQueue *wq = work_queue_create("SPIx", SCHED_PRIORITY_MAX, 3000);

	if (wq != nullptr) {
		_wq = wq;

		return true;
	}

	return false;
}

void WorkItem::ScheduleNow()
{
	_qtime = hrt_absolute_time();
	_wq->add(this);
};

void WorkItem::print_status() const
{
	perf_print_counter(_perf_cycle_time);
	perf_print_counter(_perf_interval);
	perf_print_counter(_perf_latency);
}

} // namespace px4