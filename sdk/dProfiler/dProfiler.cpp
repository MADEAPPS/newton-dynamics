/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dProfiler.h"

//#if 0
#if !defined (WIN32) || (_MSC_VER >= 1900)


	#include "Tracy.hpp"
	#include "common\TracySystem.hpp"
	#include "client\TracyProfiler.hpp"

	using namespace tracy;
	
	long long dProfilerStartTrace__(const dProfilerSourceLocation* const srcloc)
	{
		const auto thread = GetThreadHandle();
		Magic magic;
		auto token = GetToken();
		auto& tail = token->get_tail_index();
		auto item = token->enqueue_begin<tracy::moodycamel::CanAlloc>(magic);
		MemWrite(&item->hdr.type, QueueType::ZoneBegin);
#ifdef TRACY_RDTSCP_OPT
		MemWrite(&item->zoneBegin.time, Profiler::GetTime(item->zoneBegin.cpu));
#else
		uint32_t cpu;
		MemWrite(&item->zoneBegin.time, Profiler::GetTime(cpu));
		MemWrite(&item->zoneBegin.cpu, cpu);
#endif
		MemWrite(&item->zoneBegin.thread, thread);
		MemWrite(&item->zoneBegin.srcloc, (uint64_t)srcloc);
		tail.store(magic + 1, std::memory_order_release);
		return long long(thread);
	}

	void dProfilerEndTrace__(long long threadId)
	{
		Magic magic;
		std::thread::native_handle_type thread = (std::thread::native_handle_type) threadId;
		auto token = GetToken();
		auto& tail = token->get_tail_index();
		auto item = token->enqueue_begin<tracy::moodycamel::CanAlloc>(magic);
		MemWrite(&item->hdr.type, QueueType::ZoneEnd);
#ifdef TRACY_RDTSCP_OPT
		MemWrite(&item->zoneEnd.time, Profiler::GetTime(item->zoneEnd.cpu));
#else
		uint32_t cpu;
		MemWrite(&item->zoneEnd.time, Profiler::GetTime(cpu));
		MemWrite(&item->zoneEnd.cpu, cpu);
#endif
		MemWrite(&item->zoneEnd.thread, thread);
		tail.store(magic + 1, std::memory_order_release);
	}

	void dProfilerSetTrackName__(const char* const trackName)
	{
		std::thread::native_handle_type handle = (std::thread::native_handle_type) GetThreadHandle();
		SetThreadName(handle, trackName);
	}

#else

	long long dProfilerStartTrace__(const dProfilerSourceLocation* const)
	{
		return 0;
	}

	void dProfilerEndTrace__(int long long)
	{
	}

	void dProfilerSetTrackName__(const char* const)
	{
	}

#endif
