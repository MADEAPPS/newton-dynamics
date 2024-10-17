/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_PROFILER_H__
#define __ND_PROFILER_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"

// to make a profile build use Use CMAKE to create a profile configuration
// or make a configuration that define macro D_PROFILER

#ifdef D_PROFILER
	#include <dTracyProfiler.h>
	#define D_TRACKTIME() dProfilerZoneScoped(__FUNCTION__)
	#define D_TRACKTIME_NAMED(name) dProfilerZoneScoped(#name)
	#define D_SET_TRACK_NAME(trackName) dProfilerSetTrackName(trackName)
#else
	#define D_TRACKTIME() 
	#define D_TRACKTIME_NAMED(name)
	#define D_SET_TRACK_NAME(trackName)
#endif

#endif
