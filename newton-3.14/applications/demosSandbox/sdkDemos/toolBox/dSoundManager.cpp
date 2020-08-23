/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "toolbox_stdafx.h"
#include "dSoundManager.h"
#include "DemoEntityManager.h"

#if 0
dSoundManager::dSoundChannel::dSoundChannel()
	:m_source(0)
	,m_myAssetNode(NULL)
{
	alGenSources(1, (ALuint*)&m_source);
	dAssert (m_source);
	dAssert (alGetError() == AL_NO_ERROR);
}

dSoundManager::dSoundChannel::~dSoundChannel()
{
	alDeleteSources(1, (ALuint*)&m_source);
	dAssert (alGetError() == AL_NO_ERROR);
}

void dSoundManager::dSoundChannel::LinkAsset( dTree<dSoundAsset, dCRCTYPE>::dTreeNode* const assetNode)
{
	m_myAssetNode = assetNode;
	dSoundAsset& asset = m_myAssetNode->GetInfo();
	alSourcei (m_source, AL_BUFFER, asset.m_buffer);
	dAssert (alGetError() == AL_NO_ERROR);
}

void dSoundManager::dSoundChannel::Play ()
{
	// set some default values
	alSourcef(m_source, AL_GAIN, 1);
	alSource3f(m_source, AL_POSITION, 0, 0, 0);
	alSource3f(m_source, AL_VELOCITY, 0, 0, 0);
	alSourcei(m_source, AL_LOOPING, AL_FALSE);

	alSourcePlay(m_source);
	dAssert (alGetError() == AL_NO_ERROR);
}

void dSoundManager::dSoundChannel::Stop ()
{
	alSourceStop (m_source);
	dAssert (alGetError() == AL_NO_ERROR);
}

bool dSoundManager::dSoundChannel::IsPlaying() const
{
	ALint state;
	alGetSourcei(m_source, AL_SOURCE_STATE, &state);
	return (state == AL_PLAYING) ? true : false;
}

void dSoundManager::dSoundChannel::SetLoop(bool mode)
{
	alSourcei(m_source, AL_LOOPING, mode ? 1 : 0);
}

void dSoundManager::dSoundChannel::SetPitch(dFloat pitch)
{
	alSourcef(m_source, AL_PITCH, pitch);
}

void dSoundManager::dSoundChannel::SetVolume(dFloat volume)
{
	alSourcef(m_source, AL_GAIN, volume);
}


dFloat dSoundManager::dSoundChannel::GetSecPosition() const
{
	ALfloat position;
	alGetSourcef(m_source, AL_SEC_OFFSET, &position);
	return position;
}

dFloat dSoundManager::dSoundChannel::GetVolume() const
{
	ALfloat volume;
	alGetSourcef(m_source, AL_GAIN, &volume);
	return volume;
}

dSoundManager::dSoundAsset::dSoundAsset()
	:dRefCounter()
	,m_buffer(0)
	,m_lenght(0)
	,m_frequecy(0)
{
	alGenBuffers(1, (ALuint*)&m_buffer);
	dAssert (m_buffer);
	dAssert (alGetError() == AL_NO_ERROR);
}

dSoundManager::dSoundAsset::~dSoundAsset()
{
	RemoveAll();
	alDeleteBuffers(1, (ALuint *)&m_buffer);
	dAssert (alGetError() == AL_NO_ERROR);
}




dSoundManager::dSoundManager()
	:m_device(alcOpenDevice(NULL))
	,m_context(NULL)
	,m_coordinateSystem (dGetIdentityMatrix())
{
	if (m_device) {
		m_context = alcCreateContext(m_device, NULL);
		alcMakeContextCurrent(m_context);

		// clear error code
		alGetError();
		dAssert (alGetError() == AL_NO_ERROR);

		ALfloat listenerPos[]={0.0,0.0,0.0};
		alListenerfv(AL_POSITION, listenerPos);
		dAssert (alGetError() == AL_NO_ERROR);

		ALfloat listenerVel[]={0.0,0.0,0.0};
		alListenerfv(AL_VELOCITY, listenerVel);
		dAssert (alGetError() == AL_NO_ERROR);

		ALfloat listenerOri[]={0.0,0.0,-1.0, 0.0,1.0,0.0};
		alListenerfv(AL_ORIENTATION, listenerOri);
		dAssert (alGetError() == AL_NO_ERROR);


	/*
		// fmod coordinate system uses (0,0,1) as from vector
		m_coordinateSystem[0] = dVector (0.0f, 0.0f, 1.0f);
		m_coordinateSystem[1] = dVector (0.0f, 1.0f, 0.0f);
		m_coordinateSystem[2] = dVector (1.0f, 0.0f, 0.0f);
	*/
	}
}

dSoundManager::~dSoundManager()
{
	if (m_device) {
		alcDestroyContext(m_context);
		alcCloseDevice(m_device);
	}
}



void dSoundManager::UpdateListener(const dVector& position, const dVector& velocity, const dVector& heading, const dVector& upDir)
{
//	dAssert (0);
/*
	dVector fposition (m_coordinateSystem.RotateVector(position));
	dVector fvelocity (m_coordinateSystem.RotateVector(velocity));
	dVector fheading (m_coordinateSystem.RotateVector(heading));
	dVector fupDir (m_coordinateSystem.RotateVector(upDir));

	FMOD_VECTOR up = { fupDir.m_x, fupDir.m_y, fupDir.m_z };
	FMOD_VECTOR pos = { fposition.m_x, fposition.m_y, fposition.m_z };
	FMOD_VECTOR vel = { fvelocity.m_x, fvelocity.m_y, fvelocity.m_z };
	FMOD_VECTOR forward = { fheading.m_x, fheading.m_y, fheading.m_z };
	
	FMOD_RESULT result = FMOD_System_Set3DListenerAttributes(m_system, 0, &pos, &vel, &forward, &up);
	ERRCHECK(result);		
*/
}


void dSoundManager::LoadWaveFile(dSoundAsset* const asset, const char* const fileName)
{
	FILE* const wave = fopen(fileName, "rb");
	if (wave) {
		char xbuffer[5];
		memset (xbuffer, 0, sizeof (xbuffer));
		fread(xbuffer, sizeof(char), 4, wave);
		if (!strcmp(xbuffer, "RIFF")) {

			//file_read_int32_le(xbuffer, wave);
			int chunkSize;
			fread(&chunkSize, sizeof(int), 1, wave);
			fread(xbuffer, sizeof(char), 4, wave);
			if (!strcmp(xbuffer, "WAVE")) {
				fread(xbuffer, sizeof(char), 4, wave);
				if (!strcmp(xbuffer, "fmt ")) {
					fread(&chunkSize, sizeof(int), 1, wave);

					int sampleRate;
					int byteRate;
					short channels;
					short audioFormat;
					short blockAlign;
					short bitsPerSample;

					fread(&audioFormat, sizeof(short), 1, wave);
					fread(&channels, sizeof(short), 1, wave);
					fread(&sampleRate, sizeof(int), 1, wave);
					fread(&byteRate, sizeof(int), 1, wave);
					fread(&blockAlign, sizeof(short), 1, wave);
					fread(&bitsPerSample, sizeof(short), 1, wave);
					for (int i = 0; i < (chunkSize - 16); i ++) {
						fread(xbuffer, sizeof(char), 1, wave);
					}
					
					#define WAVE_FORMAT_PCM 0x0001
					//0x0003 WAVE_FORMAT_IEEE_FLOAT IEEE float 
					//0x0006 WAVE_FORMAT_ALAW 8-bit ITU-T G.711 A-law 
					//0x0007 WAVE_FORMAT_MULAW 8-bit ITU-T G.711 µ-law 
					//0xFFFE WAVE_FORMAT_EXTENSIBLE Determined by SubFormat 

					// I only parse WAVE_FORMAT_PCM format
					dAssert (audioFormat == WAVE_FORMAT_PCM);

					fread(xbuffer, sizeof(char), 4, wave);
					if (!strcmp(xbuffer, "fact")) {
						int size;
						int samplesPerChannels;
						fread(&size, sizeof(int), 1, wave);
						fread(&samplesPerChannels, sizeof(int), 1, wave);
						fread(xbuffer, sizeof(char), 4, wave);
					}

					if (!strcmp(xbuffer, "data")) {
						int size;
						fread(&size, sizeof(int), 1, wave);

						char* const data = new char[size];
						fread(data, sizeof(char), size, wave);


						int waveFormat = AL_FORMAT_MONO8;
						if (channels == 1) {
							if (bitsPerSample == 8) {
								waveFormat = AL_FORMAT_MONO8;
							} else {
								dAssert (bitsPerSample == 16);
								waveFormat = AL_FORMAT_MONO16;
							}
						} else {
							dAssert (channels == 2);
							if (bitsPerSample == 8) {
								waveFormat = AL_FORMAT_STEREO8;
							} else {
								dAssert (bitsPerSample == 16);
								waveFormat = AL_FORMAT_STEREO16;
							}

						}

						asset->m_lenght = dFloat(size) / byteRate;
						asset->m_frequecy = dFloat (sampleRate);
						alBufferData(asset->m_buffer, waveFormat, data, size, sampleRate);

						delete[] data;
					}
				}
			}
		}
		fclose (wave);
	}
}


void* dSoundManager::CreateSound (const char* const fileName)
{
	if (m_device) {
		char path[2048];
		dGetWorkingFileName (fileName, path);

		dCRCTYPE code = dCRC64 (path);
		dSoundAssetList::dTreeNode* assetNode = m_assets.Find(code);
		if (!assetNode) {
			assetNode = m_assets.Insert (code);
			LoadWaveFile(&assetNode->GetInfo(), path);
		}
		assetNode->GetInfo().AddRef();
		return assetNode;
	}
	return NULL;
	
}

void dSoundManager::DestroySound(void* const soundAssetHandle)
{
	if (m_device) {
		dSoundAssetList::dTreeNode* const node = (dSoundAssetList::dTreeNode*)soundAssetHandle;
		dAssert (node);

		dSoundAsset& asset = node->GetInfo();
		dAssert (asset.GetRef() > 1);
		asset.Release();

		m_assets.Remove(node);
	}
}


void dSoundManager::DestroyAllSound()
{
	if (m_device) {
		while (m_assets.GetRoot()) {
			DestroySound (m_assets.GetRoot());
		}
	}
}


dFloat dSoundManager::GetSoundlength (void* const soundAssetHandle)
{
	if (m_device) {
		dSoundAssetList::dTreeNode* const node = (dSoundAssetList::dTreeNode*)soundAssetHandle;
		dSoundAsset& asset = node->GetInfo();
		return 	asset.m_lenght;
	}
	return 0;
}


void* dSoundManager::CreatePlayChannel (void* const soundAssetHandle) 
{
	if (m_device) {
		dSoundAssetList::dTreeNode* const node = (dSoundAssetList::dTreeNode*)soundAssetHandle;
		dAssert (node);

		dSoundAsset& asset = node->GetInfo();
		dSoundChannelList::dListNode* const channelNode = asset.Append();
		dSoundChannel& channel = channelNode->GetInfo();
		channel.LinkAsset (node);
		return channelNode;
	}
	return NULL;
}

void dSoundManager::DestroyChannel(void* const channelHandle)
{
	if (m_device) {
		dAssert (0);
	}
}


void* dSoundManager::GetAsset(void* const channelHandle) const
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.m_myAssetNode;
	}
	return NULL;
}


void dSoundManager::PlayChannel (void* const channelHandle)
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		if (!channel.IsPlaying()) {
			channel.Play();
			m_channelPlaying.Append(node);
		}
	}
}

void dSoundManager::StopChannel (void* const channelHandle)
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.Stop();
	}
}

dFloat dSoundManager::GetChannelVolume(void* const channelHandle) const
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.GetVolume();
	}
	return 0;
}

void dSoundManager::SetChannelVolume(void* const channelHandle, dFloat volume)
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetVolume(volume);
	}
}


void dSoundManager::SetChannelLoopMode (void* const channelHandle, bool mode)
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetLoop(mode);
	}
}

void dSoundManager::SetChannelPitch(void* const channelHandle, dFloat pitch)
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetPitch(pitch);
	}
}

dFloat dSoundManager::GetChannelGetPosition(void* const channelHandle) const
{
	if (m_device) {
		dSoundChannelList::dListNode* const node = (dSoundChannelList::dListNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.GetSecPosition();
	}
	return 0;
}

void dSoundManager::Update()
{
	if (m_device) {
		dSoundChannelPlaying::dListNode* next;
		for (dSoundChannelPlaying::dListNode* node = m_channelPlaying.GetFirst(); node; node = next) {
			dSoundChannelList::dListNode* const channelNode = node->GetInfo();
			next = node->GetNext();

			dSoundChannel& channel = channelNode->GetInfo();
			if (!channel.IsPlaying()) {
				m_channelPlaying.Remove(node);
			} else {
			}
		}
	}
}

#endif