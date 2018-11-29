
#pragma once

#include <px4_app.h>
#include <lib/work_queue/ScheduledWorkItem.hpp>
#include <string.h>

using namespace px4;

class WQueueScheduledTest : public px4::ScheduledWorkItem
{
public:
	WQueueScheduledTest() = default;
	~WQueueScheduledTest() = default;

	int main();

	void Run() override;

	static px4::AppState appState; /* track requests to terminate app */

private:
	int _iter{0};
};
