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

#include "StdAfx.h"
#include "OpenGlUtil.h"
#include "SoundManager.h"


SoundManager::SoundManager(void)
{
	// We're going to be requesting certain things from our audio device, so we set them up beforehand 
	int audio_rate = 22050;
	Uint16 audio_format = AUDIO_S16; // 16-bit stereo 
	int audio_channels = 2;
	int audio_buffers = 512;

	m_audioOn = 0;
	m_soundCount = 0;
	// This is where we open up our audio device.  Mix_OpenAudio takes as its parameters the audio format we'd /like/ to have.
	if (!Mix_OpenAudio(audio_rate, audio_format, audio_channels, audio_buffers)) {
		// If we actually care about what we got, we can ask here.  In this
		//program we don't, but I'm showing the function call here anyway
		//in case we'd want to know later. 
	    Mix_QuerySpec(&audio_rate, &audio_format, &audio_channels);

		m_audioOn = 1;
	}

}

SoundManager::~SoundManager(void)
{
	// This is the cleaning up part 
	if (m_audioOn) {
		for (int i = 0; i < m_soundCount; i ++) {
			Mix_FreeChunk (m_soudnList[i]);
		}
		Mix_CloseAudio();
	}

}

void* SoundManager::LoadSound (const char* name)
{
	Mix_Chunk *sound;
	char pathName[2048];

	GetWorkingFileName (name, pathName);
	sound = Mix_LoadWAV(pathName);

	m_soudnList[m_soundCount] = sound;
	m_soundCount ++;

	return sound;
}


void SoundManager::Play (void* clip, dFloat volume, int loopTimes)
{
	Mix_VolumeChunk ((Mix_Chunk *) clip, int (volume * MIX_MAX_VOLUME));
	Mix_PlayChannel(-1, (Mix_Chunk *) clip, loopTimes);
}
