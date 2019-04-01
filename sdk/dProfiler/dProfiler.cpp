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

#if !defined (WIN32) || (_MSC_VER >= 1900)

	#include "Tracy.hpp"
	#include "common\TracySystem.hpp"
	#include "client\TracyProfiler.hpp"
	
	long long dProfilerStartTrace(const dProfilerSourceLocation* const sourceLocation)
	{
		std::thread::native_handle_type thread = (std::thread::native_handle_type)tracy::GetThreadHandle();
//		m_thread = thread;
		tracy::Magic magic;
		auto& token = tracy::s_token.ptr;
		auto& tail = token->get_tail_index();
		auto item = token->enqueue_begin<tracy::moodycamel::CanAlloc>(magic);
		tracy::MemWrite(&item->hdr.type, tracy::QueueType::ZoneBegin);
#ifdef TRACY_RDTSCP_OPT
		tracy::MemWrite(&item->zoneBegin.time, tracy::Profiler::GetTime(item->zoneBegin.cpu));
#else
		uint32_t cpu;
		tracy::MemWrite(&item->zoneBegin.time, tracy::Profiler::GetTime(cpu));
		tracy::MemWrite(&item->zoneBegin.cpu, cpu);
#endif
		tracy::MemWrite(&item->zoneBegin.thread, thread);
		tracy::MemWrite(&item->zoneBegin.srcloc, (uint64_t)sourceLocation);
		tail.store(magic + 1, std::memory_order_release);

		return long long (thread);
	}

	void dProfilerEndTrace(long long threadId)
	{
		std::thread::native_handle_type thread = (std::thread::native_handle_type) threadId;
		tracy::Magic magic;
		auto& token = tracy::s_token.ptr;
		auto& tail = token->get_tail_index();
		auto item = token->enqueue_begin<tracy::moodycamel::CanAlloc>(magic);
		tracy::MemWrite(&item->hdr.type, tracy::QueueType::ZoneEnd);
#ifdef TRACY_RDTSCP_OPT
		tracy::MemWrite(&item->zoneEnd.time, tracy::Profiler::GetTime(item->zoneEnd.cpu));
#else
		uint32_t cpu;
		tracy::MemWrite(&item->zoneEnd.time, tracy::Profiler::GetTime(cpu));
		tracy::MemWrite(&item->zoneEnd.cpu, cpu);
#endif
		tracy::MemWrite(&item->zoneEnd.thread, thread);
		tail.store(magic + 1, std::memory_order_release);
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
		std::thread::native_handle_type handle = (std::thread::native_handle_type) tracy::GetThreadHandle();
		tracy::SetThreadName(handle, trackName);

		//const char* xxx0 = tracy::GetThreadName(long long (handle));
		//const char* xxx1 = tracy::GetThreadName(long long (handle));
	}

	void dProfilerDeleteTrack()
	{
	}

#else

	long long dProfilerStartTrace(const char* const fileName)
	{
		return 0;
	}

	void dProfilerEndTrace(int long long)
	{
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
	}

	void dProfilerDeleteTrack()
	{
	}

#endif
