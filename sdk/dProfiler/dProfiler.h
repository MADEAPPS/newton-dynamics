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

#ifndef __D_PROFILER_H__
#define __D_PROFILER_H__


#ifdef D_PROFILER_EXPORTS
#define D_PROFILER_API __declspec(dllexport)
#else
#define D_PROFILER_API __declspec(dllimport)
#endif


D_PROFILER_API int dProfilerStartTrace(const char* const fileName);
D_PROFILER_API void dProfilerEndTrace(int id);

D_PROFILER_API void dProfilerSetTrackName(const char* const trackName);
D_PROFILER_API void dProfilerDeleteTrack();

//D_PROFILER_API void ttStartRecording(const char* const fileName);
//D_PROFILER_API void ttStopRecording();

//D_PROFILER_API int ttOpenRecord(const char* const name);
//D_PROFILER_API void ttCloseRecord(int record);

//D_PROFILER_API void ttDeleteTrack();
//D_PROFILER_API void ttSetTrackName(const char* const threadName);

#endif