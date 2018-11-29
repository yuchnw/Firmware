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

#include "WorkQueue.hpp"
#include "WorkItem.hpp"

#include <string.h>

#include <px4_tasks.h>
#include <px4_time.h>
#include <drivers/drv_hrt.h>

namespace px4
{

List<WorkQueue *> px4_work_queues_list;
pthread_mutex_t px4_work_queues_list_mutex = PTHREAD_MUTEX_INITIALIZER;

WorkQueue::WorkQueue(const char *name) :
	_name(name)
{
#ifndef __PX4_NUTTX
	px4_sem_init(&_qlock, 0, 1);
#endif /* __PX4_NUTTX */

	px4_sem_init(&_process_lock, 0, 0);
	px4_sem_setprotocol(&_process_lock, SEM_PRIO_NONE);
}

WorkQueue::~WorkQueue()
{
#ifndef __PX4_NUTTX
	px4_sem_destroy(&_qlock);
#endif /* __PX4_NUTTX */

	px4_sem_destroy(&_process_lock);
}

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

static int work_queue_thread(int argc, char *argv[]);

WorkQueue *work_queue_create(const char *name, uint8_t priority, int stacksize)
{
	// find existing work queue or create new
	PX4_DEBUG("work_queue_create: %s %d %d", name, priority, stacksize);

	// search list for existing work queue
	WorkQueue *wq = find_work_queue(name);

	if (wq == nullptr) {

		int pid = px4_task_spawn_cmd(name,
					     SCHED_DEFAULT,
					     priority,
					     stacksize,
					     &work_queue_thread,
					     (char *const *)nullptr);

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
			wq->set_task_id(pid);
		}
	}

	return wq;
}

int work_queue_thread(int argc, char *argv[])
{
	PX4_DEBUG("work_queue_thread: %s", px4_get_taskname());

	// create work queue and add to list
	WorkQueue wq(px4_get_taskname());

	// add to work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);
	px4_work_queues_list.add(&wq);
	pthread_mutex_unlock(&px4_work_queues_list_mutex);

	// Loop forever processing
	for (;;) {
		// TODO: shutdown mechanism
		wq.process();
	}

	// remove from work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);
	px4_work_queues_list.remove(&wq);
	pthread_mutex_unlock(&px4_work_queues_list_mutex);
}

void WorkQueue::add(WorkItem *item)
{
	work_lock();
	_q.push(item);
	work_unlock();

	// Wake up the worker thread
	px4_sem_post(&_process_lock);
}

void WorkQueue::process()
{
	px4_sem_wait(&_process_lock);

	// process queued work
	work_lock();

	while (!_q.empty()) {

		WorkItem *work = _q.pop();

		work_unlock(); // unlock to run

		work->pre_run();
		work->Run();
		work->post_run();

		work_lock();
	}

	work_unlock();
}

} // namespace px4
