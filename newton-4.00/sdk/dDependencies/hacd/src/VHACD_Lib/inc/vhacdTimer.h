/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#ifndef ND_VHACD_TIMER_H
#define ND_VHACD_TIMER_H

#include "vhacdDefines.h"

namespace nd
{
	namespace VHACD 
	{
		#ifdef _WIN32
		class Timer {
			public:
			Timer(void)
			{
				m_start.QuadPart = 0;
				m_stop.QuadPart = 0;
				QueryPerformanceFrequency(&m_freq);
			}
			~Timer(void) {}
			void Tic()
			{
				QueryPerformanceCounter(&m_start);
			}
			void Toc()
			{
				QueryPerformanceCounter(&m_stop);
			}
			double GetElapsedTime() // in ms
			{
				LARGE_INTEGER delta;
				delta.QuadPart = m_stop.QuadPart - m_start.QuadPart;
				return (1000.0 * (double)delta.QuadPart) / (double)m_freq.QuadPart;
			}

		private:
			LARGE_INTEGER m_start;
			LARGE_INTEGER m_stop;
			LARGE_INTEGER m_freq;
		};
	}
}
#endif // VHACD_TIMER_H
