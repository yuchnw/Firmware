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

#pragma once

#include <containers/List.hpp>
#include <containers/Queue.hpp>

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>


extern "C" __EXPORT int wq_manager_main(int argc, char *argv[]);

namespace px4
{

// work queues

// SPI1 : PRIORITY MAX : STACK 750


enum PX4_WQS {
	SPI_1 = 0,
	SPI_2,

	I2C_1,
	I2C_2,

	rate_loop,
};

struct wq_config_t {
	const char *name;
	int16_t priority;
	uint16_t stacksize;
};

static constexpr wq_config_t wq_configurations[] = {

	[SPI_1] = { "SPI_1", 250, 500 },
	[SPI_2] = { "SPI_2", 250, 500 },

	[I2C_1] = { "I2C_1", 240, 500 },
	[I2C_2] = { "I2C_2", 240, 500 },

	[rate_loop] = { "rate_loop", 249, 1000 },

};

class WorkQueue;

// list of all px4 work queues
extern pthread_mutex_t px4_work_queues_list_mutex;
extern List<WorkQueue *> px4_work_queues_list;

class WorkQueueManager : public ModuleBase<WorkQueueManager>
{

public:

	WorkQueueManager() = default;
	~WorkQueueManager() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static WorkQueueManager *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	static void task_main_trampoline(int argc, char *argv[]);

	int print_status() override;


	void add_workqueue(WorkQueue *wq);
	void remove_workqueue(WorkQueue *wq);

private:
	void create_work_queue_thread(pthread_t *thread);

	static void *work_queue_runner(void *context);

};

WorkQueue *work_queue_create(const char *name, uint8_t priority, int stacksize);


} // namespace px4
