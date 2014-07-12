/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _D_RUNTINE_PROFILER_
#define _D_RUNTINE_PROFILER_

#define MAX_FRAMES 350
#define MAX_FRAMES_STEP 1
#define MAX_TRACKS (NEWTON_POST_LISTERNER_CALLBACK_UPDATE + 1)

#define CHART_HIEGHT 120

class DemoEntityManager;

class dRuntimeProfiler
{
	public:
	dRuntimeProfiler();
	dRuntimeProfiler(int origin_x, int origin_y);
	~dRuntimeProfiler(void);

	void Init (DemoEntityManager* const scene);
	int Render (int mask, int lineNumber);
	int RenderConcurrentPerformance (int lineNumber);

	void DrawLabel (dFloat x, dFloat y, const char* const label) const;
	void DrawRectangle (dFloat x, dFloat y, dFloat dx, dFloat dy, const dVector& color) const;
	void DrawTrack (dFloat x, dFloat y, const dVector& color, int start, const unsigned* const track) const;
	void RenderThreadPerformance ();



	void DrawConcurrentChart(int count, const dFloat* const times, const dVector* colors);

	static unsigned GetTimeInMicrosenconds();

	int m_width;
	int m_height;
	int m_oringin_x;
	int m_oringin_y;
	int m_frameIndex;
	int m_nextLine;
	unsigned m_perfomanceTracks[MAX_TRACKS][MAX_FRAMES];
	DemoEntityManager* m_scene;
};


#endif
