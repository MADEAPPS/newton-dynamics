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

#include <toolbox_stdafx.h>
#include "OpenGlUtil.h"
#include "dRuntimeProfiler.h"
#include "dHighResolutionTimer.h"
#include "DemoEntityManager.h"
#include "NewtonDemos.h"

dRuntimeProfiler::dRuntimeProfiler()
{
}

dRuntimeProfiler::dRuntimeProfiler(int origin_x, int origin_y)
{
	m_oringin_x = origin_x;
	m_oringin_y = origin_y;
	m_frameIndex = 0;
	memset (m_perfomanceTracks, 0, sizeof (m_perfomanceTracks));
}

dRuntimeProfiler::~dRuntimeProfiler(void)
{
}

unsigned dRuntimeProfiler::GetTimeInMicrosenconds()
{
	return unsigned (dGetTimeInMicrosenconds());
}


void dRuntimeProfiler::Init (DemoEntityManager* const scene)
{
	m_scene = scene;
}

void dRuntimeProfiler::DrawLabel (dFloat x, dFloat y, const char* const label) const
{
	dVector color (1.0f, 1.0f, 1.0f, 0.0f);
	m_scene->Print (color, x, m_height - y, label);
}


void dRuntimeProfiler::DrawRectangle (dFloat x, dFloat y, dFloat dx, dFloat dy, const dVector& color) const
{
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor4f(color.m_x, color.m_y, color.m_z, color.m_w);

	// speedometer

	float x1 = x + dx;
	float y1 = y + dy;
	glBegin(GL_QUADS);
	glVertex3f( x,  m_height - y, 0);
	glVertex3f( x,  m_height - y1, 0);
	glVertex3f( x1, m_height - y1, 0);
	glVertex3f( x1, m_height - y, 0);
	glEnd();

}

void dRuntimeProfiler::DrawTrack (dFloat x0, dFloat y0, const dVector& color, int start, const unsigned* const track) const
{
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(color.m_x, color.m_y, color.m_z);

	unsigned buffer[2048];

	int index = 0;
	for (int i = start + 1; i < MAX_FRAMES; i ++) {
		buffer[index] = track[i];
		index ++;
	}

	for (int i = 0; i < start; i ++) {
		buffer[index] = track[i];
		index ++;
	}
	buffer[index] = track[start];

	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < MAX_FRAMES; i ++) {
		dFloat y1 = dFloat (buffer[i]) * 1.0e-3f * (CHART_HIEGHT / 16.666f);
		glVertex3f (x0 + i * MAX_FRAMES_STEP, y0 + y1, 0.0f);
	}

	glEnd();
}


int dRuntimeProfiler::Render (int mask, int lineNumber)
{
	struct GLViewPort
	{
		int x;
		int y;
		int width;
		int height;
	} viewport;

	//Retrieves the viewport and stores it in the variable
	glGetIntegerv(GL_VIEWPORT, (GLint*) &viewport.x); 

	m_width = viewport.width;
	m_height = viewport.height;

	NewtonWorld* const world = m_scene->GetNewton(); 
	for (int i = 0; i < MAX_TRACKS; i ++) {
		m_perfomanceTracks[i][m_frameIndex] = NewtonReadPerformanceTicks (world, i);
	}

	glColor3f(1.0, 1.0, 1.0);
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, viewport.width, 0, viewport.height );

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);


	dFloat x0 = dFloat (m_oringin_x);
	dFloat y0 = dFloat (m_oringin_y);
	dFloat x1 = x0 + MAX_FRAMES * MAX_FRAMES_STEP;
	dFloat y1 = y0 + CHART_HIEGHT;


	glBegin(GL_LINES);

	glVertex3f (x0, y0, 0.0f);
	glVertex3f (x0, y1, 0.0f);

	glVertex3f (x0, y0, 0.0f);
	glVertex3f (x1, y0, 0.0f);


	for (int i = 1; i < 4; i ++) {
		dFloat y = y0 + (y1 - y0) * i / 4;
		glVertex3f (x0 - 5, y, 0.0f);
		glVertex3f (x0 + 5, y, 0.0f);
	}

	for (int i = 1; i < MAX_FRAMES; i += 16) {
		dFloat x = x0 + (x1 - x0) * i / MAX_FRAMES;
		glVertex3f (x , y0 - 5, 0.0f);
		glVertex3f (x , y0 + 5, 0.0f);
	}
	glEnd();

	DrawLabel (10, m_height  - m_nextLine, "Profiler legend");
	m_nextLine = lineNumber;

	// total engine time
	if (mask & 1) {
		DrawLabel (10, m_height - m_nextLine, "white chart: world global update");
		DrawTrack (x0, y0, dVector (1.0f, 1.0f, 1.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_WORLD_UPDATE][0]);
		m_nextLine += 20;
	}


	// draw collision performance
	if (mask & 2) { 
		DrawLabel (10, m_height - m_nextLine, "red chart: collision global update");
		DrawTrack (x0, y0, dVector (1.0f, 0.0f, 0.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_COLLISION_UPDATE][0]);
		m_nextLine += 20;
	}

	if (mask & 4) { 
		DrawLabel (10, m_height - m_nextLine, "green chart: collision broad phase update");
		DrawTrack (x0, y0, dVector (0.0f, 1.0f, 0.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_COLLISION_UPDATE_BROAD_PHASE][0]);
		m_nextLine +=  20;
	}

	if (mask & 8) { 
		DrawLabel (10, m_height - m_nextLine, "blue chart: collision narrow phase update");
		DrawTrack (x0, y0, dVector (0.0f, 0.0f, 1.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_COLLISION_UPDATE_NARROW_PHASE][0]);
		m_nextLine += 20;
	}

	// draw dynamics performance
	if (mask & 16) { 
		DrawLabel (10, m_height - m_nextLine, "cyan chart: dynamics global update");
		DrawTrack (x0, y0, dVector (0.0f, 1.0f, 1.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_DYNAMICS_UPDATE][0]);
		m_nextLine += 20;
	}

	if (mask & 32) { 
		DrawLabel (10, m_height - m_nextLine, "black chart: dynamics solver update");
		DrawTrack (x0, y0, dVector (0.0f, 0.0f, 0.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_DYNAMICS_CONSTRAINT_GRAPH][0]);
		m_nextLine += 20;
	}

	if (mask & 64) { 
		DrawLabel (10, m_height - m_nextLine, "yellow chart: dynamics solver update");
		DrawTrack (x0, y0, dVector (1.0f, 1.0f, 0.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_DYNAMICS_SOLVE_CONSTRAINT_GRAPH][0]);
		m_nextLine += 20;
	}

	// draw force Update performance
	if (mask & 128) { 
		DrawLabel (10, m_height - m_nextLine, "magenta chart: force and torque callback update");
		DrawTrack (x0, y0, dVector (1.0f, 0.0f, 1.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_FORCE_CALLBACK_UPDATE][0]);
		m_nextLine += 20;
	}

	if (mask & 256) { 
		DrawLabel (10, m_height - m_nextLine, "magenta chart: pre simulation listener");
		DrawTrack (x0, y0, dVector (1.0f, 0.0f, 1.0f), m_frameIndex, &m_perfomanceTracks[NEWTON_PROFILER_FORCE_CALLBACK_UPDATE][0]);
		m_nextLine += 20;
	}

	if (mask & 256) { 
		DrawLabel (10, m_height - m_nextLine, "purple chart: pre simulation listener");
		DrawTrack (x0, y0, dVector (0.64f, 0.29f, 0.64f), m_frameIndex, &m_perfomanceTracks[NEWTON_PRE_LISTERNER_CALLBACK_UPDATE][0]);
		m_nextLine += 20;
	}

	if (mask & 512) { 
		DrawLabel (10, m_height - m_nextLine, "pink chart: post simulation listener");
		DrawTrack (x0, y0, dVector (1.0f, 0.68f, 0.79f), m_frameIndex, &m_perfomanceTracks[NEWTON_POST_LISTERNER_CALLBACK_UPDATE][0]);
		m_nextLine += 20;
	}

	{
		int base_y = 0;
		glColor3f(1.0, 1.0, 1.0);
		DrawLabel (x0 - 30, y0 + (y1 - y0) * 0 / 4 + base_y, "0");

		for (int i = 1; i < 5; i ++) {
			char label[32];
			sprintf (label, "%4.2f", (1000.0f / 60.0f) * (float)i / 4.0f );
			DrawLabel (x0 - 55, y0 + (y1 - y0) * i / 4 + base_y, label);
		}
	}

	glMatrixMode(GL_TEXTURE);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glEnable( GL_DEPTH_TEST );

	m_frameIndex = (m_frameIndex + 1) % MAX_FRAMES;

	glColor3f(1.0, 1.0, 1.0);

	return m_nextLine + 20;
}


void dRuntimeProfiler::RenderThreadPerformance ()
{
	NewtonWorld* const world = m_scene->GetNewton(); 
	int threadCount = NewtonGetThreadsCount(world);

	if (threadCount > 0) {
		struct GLViewPort
		{
			int x;
			int y;
			int width;
			int height;
		} viewport;

		//Retrieves the viewport and stores it in the variable
		glGetIntegerv(GL_VIEWPORT, (GLint*) &viewport.x); 


		glColor3f(1.0, 1.0, 1.0);
		glDisable (GL_LIGHTING);
		//glDisable(GL_TEXTURE_2D);

		glMatrixMode(GL_TEXTURE);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, viewport.width, 0, viewport.height );

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);

		m_width = viewport.width;
		m_height = viewport.height;

		float x0 = m_width - 600.0f; 
		float x1 = x0 + 256.0f;
		float y0 = 50.0f;
		for (int i = 0; i < threadCount; i ++) {
			char label[32];
			sprintf (label, "thread %d", i);
			DrawLabel (x0 - 90, y0 + i * 22 - 10, label);
		}

		DrawLabel (x0, y0 - 30, "0.0 ms");
		DrawLabel ((x1 + x0) * 0.5f, y0 - 30, "8.33 ms");
		DrawLabel (x1, y0 - 30, "16.66 ms");

		y0 -= 15.0f;
		dVector color (1.0f, 0.0f, 0.0f, 1.0f/8.0f);
		DrawRectangle (x0, m_height - (y0 + 20.0f * threadCount), x1 - x0, 20 * threadCount, color);

		color = dVector (1.0f, 1.0f, 0.0f, 1.0f/2.0f);
		if (threadCount > 1)  {
			for (int i = 0; i < threadCount; i ++) {
				int tick = NewtonReadThreadPerformanceTicks (world, i);
				dFloat time = dFloat (tick) * (1.0e-3f * 256.0f / 16.666f);
				DrawRectangle(x0, m_height - (y0 + 15), time, 10, color);
				y0 += 20.0f;
			}
		} else {
			int tick =  NewtonReadPerformanceTicks (world, NEWTON_PROFILER_WORLD_UPDATE);
			dFloat time = dFloat (tick) * (1.0e-3f * 256.0f / 16.666f);
			DrawRectangle(x0, m_height - (y0 + 15), time, 10, color);
		}

		glMatrixMode(GL_TEXTURE);
		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		glDisable(GL_BLEND);
		glEnable (GL_DEPTH_TEST);
		glColor3f(1.0, 1.0, 1.0);
	}
}

void dRuntimeProfiler::DrawConcurrentChart(int count, const dFloat* const times, const dVector* colors)
{
	int leftBorder = 10;
	int rightBorder = 70;

	glBegin(GL_LINES);
	glVertex3f (leftBorder, m_height - m_nextLine, 0.0f);
	glVertex3f (m_width - rightBorder, m_height - m_nextLine, 0.0f);

	int samples = 12;

	dFloat with = m_width - leftBorder - rightBorder;
	for (int i = 0 ; i <= samples; i ++) {
		dFloat x = i * with / samples;	
		glVertex3f (leftBorder + x, m_height - m_nextLine - 4, 0.0f);
		glVertex3f (leftBorder + x, m_height - m_nextLine + 4, 0.0f);
	}
	glEnd();

	dFloat timeStep = ((1.0f / 60.0f) * 1000.0f) / 4.0f;
	dFloat x0 = 0;
	for (int i = 0; i < count; i ++) {
		dFloat step = times[i] * with / (timeStep * samples); 
		DrawRectangle(leftBorder + x0, m_nextLine - 5.0f, step, 10, colors[i]);
		x0 += step;
	}

	dFloat acc = 0.0f;
	for (int i = 0 ; i <= samples; i ++) {
		char text[256];
		sprintf (text, "%4.2f ms", acc);
		dFloat x = i * with / samples;	
		DrawLabel (leftBorder + x, m_height - m_nextLine, text);		
		acc += timeStep;
	}
}


int dRuntimeProfiler::RenderConcurrentPerformance (int lineNumber)
{
	struct GLViewPort
	{
		int x;
		int y;
		int width;
		int height;
	} viewport;
	//Retrieves the viewport and stores it in the variable
	glGetIntegerv(GL_VIEWPORT, (GLint*) &viewport.x); 

	m_width = viewport.width;
	m_height = viewport.height;

	glColor3f(1.0, 1.0, 1.0);
	glDisable (GL_LIGHTING);
	//glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, viewport.width, 0, viewport.height );

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	DrawLabel (10, m_height - m_nextLine, "Concurrent profiler,    red = physcis;      blue = graphics      green = idle");
	m_nextLine = lineNumber;

	dFloat times[10];
	dVector colors[10];

	colors[0] = dVector (0.0f, 1.0f, 0.0f, 0.5f);
	colors[1] = dVector (0.0f, 0.0f, 1.0f, 0.5f);
	times[0] = m_scene->m_mainThreadPhysicsTime;
	times[1] = m_scene->m_mainThreadGraphicsTime;
	DrawLabel (10, m_height - m_nextLine, "graphics thread:");
	m_nextLine += 30;
	DrawConcurrentChart(2, times, colors);
	m_nextLine += 30;

	colors[0] = dVector (1.0f, 0.0f, 0.0f, 0.5f);
	times[0] = m_scene->m_physThreadTime;
	DrawLabel (10, m_height - m_nextLine, "physics thread:");
	m_nextLine += 30;
	DrawConcurrentChart(1, times, colors);
	m_nextLine += 40;

	glMatrixMode(GL_TEXTURE);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glDisable(GL_BLEND);
	glEnable (GL_DEPTH_TEST);
	glColor3f(1.0, 1.0, 1.0);

	return m_nextLine + 30;
}



