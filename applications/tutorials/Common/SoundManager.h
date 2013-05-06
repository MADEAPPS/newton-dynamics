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

#ifndef __SOUND_MANAGER__H
#define __SOUND_MANAGER__H

#define MAX_SOUNDS			64
#define MAX_SOUND_DISTANCE	30.0f
#define MIN_SOUND_DISTANCE	 5.0f

class SoundManager
{
	public:
	SoundManager(void);
	virtual ~SoundManager(void);

	void* LoadSound (const char* name);
	void Play (void* clip, dFloat volume, int loopTimes);

	int m_audioOn;
	int m_soundCount;
	Mix_Chunk *m_soudnList[MAX_SOUNDS];
};

#endif

