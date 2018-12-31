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

#include "WorkQueueManager.hpp"

#include "WorkQueue.hpp"

#include <string.h>

#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <drivers/drv_hrt.h>

namespace px4
{

List<WorkQueue *> px4_work_queues_list;
pthread_mutex_t px4_work_queues_list_mutex = PTHREAD_MUTEX_INITIALIZER;

static WorkQueue *
find_work_queue(const char *name)
{
	// search list
	for (WorkQueue *wq = px4_work_queues_list.getHead(); wq != nullptr; wq = wq->getSibling()) {
		if (strcmp(wq->name(), name) == 0) {
			return wq;
		}
	}

	return nullptr;
}

WorkQueue *
work_queue_create(const char *name, uint8_t priority, int stacksize)
{
	// find existing work queue or create new
	PX4_DEBUG("work_queue_create: %s %d %d", name, priority, stacksize);

	// search list for existing work queue
	WorkQueue *wq = find_work_queue(name);

	// create work queue if it doesn't exist
	if (wq == nullptr) {

		// add to list
		// main thread wakes up, creates the thread

		// we wait until new wq is created, then return



		// wait for task to start
		int i = 0;
		wq = nullptr;

		do {
			wq = find_work_queue(name);

			// Wait up to 1s
			px4_usleep(2500);
		} while (!wq && ++i < 400);

		if (i == 400) {
			PX4_ERR("Timed out while waiting for task to start");
		}

		if (wq != nullptr) {
			// work queue found, set PID
			//wq->set_task_id(pid);
		}
	}

	return wq;
}

void
WorkQueueManager::add_workqueue(WorkQueue *wq)
{
	// add to work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);

	px4_work_queues_list.add(wq);

	pthread_mutex_unlock(&px4_work_queues_list_mutex);
}

void
WorkQueueManager::remove_workqueue(WorkQueue *wq)
{
	// remove from work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);

	px4_work_queues_list.remove(wq);

	pthread_mutex_unlock(&px4_work_queues_list_mutex);
}

void
WorkQueueManager::run()
{
	while (true) {
		px4_sleep(1);

		// create new work queues as needed

		// queue of work queues requested
		// px4 queue with semaphore built in?

		// q.pop() "px4 work queue struct"

		/* start the MAVLink receiver last to avoid a race */
		pthread_t wq_thread;
		WorkQueueManager::create_work_queue_thread(&wq_thread);
	}

	// TODO: iterate and join all threads before shutdown
	// pthread_join(_receive_thread, nullptr);
}

void
WorkQueueManager::create_work_queue_thread(pthread_t *thread)
{
	// TODO: name, priority, stack

	pthread_attr_t attr;
	pthread_attr_init(&attr);

	// priority
	struct sched_param param;
	pthread_attr_getschedparam(&attr, &param);
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&attr, &param);

	// FIFO scheduling
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

	// stacksize
	pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(1000));

	int ret_create = pthread_create(thread, &attr, WorkQueueManager::work_queue_runner, (void *)this);

	if (ret_create != 0) {
		PX4_ERR("failed to create thread");
	}

	pthread_attr_destroy(&attr);
}

void *WorkQueueManager::work_queue_runner(void *context)
{
	//MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);
	WorkQueueManager *manager = static_cast<WorkQueueManager *>(context);
	WorkQueue wq("name");

	manager->add_workqueue(&wq);

	// /* set thread name */
	// {
	// 	char thread_name[24];
	// 	sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
	// 	px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	// }

	// Loop forever processing
	for (;;) {
		// TODO: shutdown mechanism
		wq.process();
	}

	manager->remove_workqueue(&wq);

	return nullptr;
}

WorkQueueManager *
WorkQueueManager::instantiate(int argc, char *argv[])
{
	WorkQueueManager *instance = new WorkQueueManager();

	return instance;
}

int
WorkQueueManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
WorkQueueManager::print_status()
{
	// status of each work queue

	return 0;
}

int
WorkQueueManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wq_manager", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int
WorkQueueManager::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("wq_manager",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ESTIMATOR,
				      6600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

} // namespace px4

int wq_manager_main(int argc, char *argv[])
{
	return px4::WorkQueueManager::main(argc, argv);
}
