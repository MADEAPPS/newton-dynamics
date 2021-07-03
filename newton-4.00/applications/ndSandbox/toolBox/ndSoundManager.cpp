/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSoundManager.h"
#include "ndDemoEntityManager.h"

#ifdef USE_SOUND
dSoundManager::dSoundManager()
	:ndModel()
	,m_device(alcOpenDevice(nullptr))
	,m_context(nullptr)
	,m_coordinateSystem(dGetIdentityMatrix())
{
	if (m_device)
	{
		m_context = alcCreateContext(m_device, nullptr);
		alcMakeContextCurrent(m_context);

		// clear error code
		alGetError();
		dAssert(alGetError() == AL_NO_ERROR);

		ALfloat listenerPos[] = { 0.0,0.0,0.0 };
		alListenerfv(AL_POSITION, listenerPos);
		dAssert(alGetError() == AL_NO_ERROR);

		ALfloat listenerVel[] = { 0.0,0.0,0.0 };
		alListenerfv(AL_VELOCITY, listenerVel);
		dAssert(alGetError() == AL_NO_ERROR);

		ALfloat listenerOri[] = { 0.0,0.0,-1.0, 0.0,1.0,0.0 };
		alListenerfv(AL_ORIENTATION, listenerOri);
		dAssert(alGetError() == AL_NO_ERROR);

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
	if (m_device)
	{
		alcDestroyContext(m_context);
		alcCloseDevice(m_device);
	}
}

void dSoundManager::UpdateListener(const dVector& position, const dVector& velocity, const dVector& heading, const dVector& upDir)
{
	dAssert(0);
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
	if (wave) 
	{
		char xbuffer[5];
		memset(xbuffer, 0, sizeof(xbuffer));
		fread(xbuffer, sizeof(char), 4, wave);
		if (!strcmp(xbuffer, "RIFF")) 
		{
			dInt32 chunkSize;
			fread(&chunkSize, sizeof(dInt32), 1, wave);
			fread(xbuffer, sizeof(char), 4, wave);
			if (!strcmp(xbuffer, "WAVE")) 
			{
				fread(xbuffer, sizeof(char), 4, wave);
				if (!strcmp(xbuffer, "fmt ")) 
				{
					fread(&chunkSize, sizeof(dInt32), 1, wave);

					dInt32 sampleRate;
					dInt32 byteRate;
					short channels;
					short audioFormat;
					short blockAlign;
					short bitsPerSample;

					fread(&audioFormat, sizeof(short), 1, wave);
					fread(&channels, sizeof(short), 1, wave);
					fread(&sampleRate, sizeof(dInt32), 1, wave);
					fread(&byteRate, sizeof(dInt32), 1, wave);
					fread(&blockAlign, sizeof(short), 1, wave);
					fread(&bitsPerSample, sizeof(short), 1, wave);
					for (dInt32 i = 0; i < (chunkSize - 16); i++) 
					{
						fread(xbuffer, sizeof(char), 1, wave);
					}

					#define WAVE_FORMAT_PCM 0x0001
					//0x0003 WAVE_FORMAT_IEEE_FLOAT IEEE float 
					//0x0006 WAVE_FORMAT_ALAW 8-bit ITU-T G.711 A-law 
					//0x0007 WAVE_FORMAT_MULAW 8-bit ITU-T G.711 µ-law 
					//0xFFFE WAVE_FORMAT_EXTENSIBLE Determined by SubFormat 

					// I only parse WAVE_FORMAT_PCM format
					dAssert(audioFormat == WAVE_FORMAT_PCM);

					fread(xbuffer, sizeof(char), 4, wave);
					if (!strcmp(xbuffer, "fact")) 
					{
						dInt32 size;
						dInt32 samplesPerChannels;
						fread(&size, sizeof(dInt32), 1, wave);
						fread(&samplesPerChannels, sizeof(dInt32), 1, wave);
						fread(xbuffer, sizeof(char), 4, wave);
					}

					if (!strcmp(xbuffer, "data")) 
					{
						dInt32 size;
						fread(&size, sizeof(dInt32), 1, wave);

						char* const data = new char[size];
						fread(data, sizeof(char), size, wave);


						dInt32 waveFormat = AL_FORMAT_MONO8;
						if (channels == 1) 
						{
							if (bitsPerSample == 8) 
							{
								waveFormat = AL_FORMAT_MONO8;
							}
							else 
							{
								dAssert(bitsPerSample == 16);
								waveFormat = AL_FORMAT_MONO16;
							}
						}
						else 
						{
							dAssert(channels == 2);
							if (bitsPerSample == 8) 
							{
								waveFormat = AL_FORMAT_STEREO8;
							}
							else 
							{
								dAssert(bitsPerSample == 16);
								waveFormat = AL_FORMAT_STEREO16;
							}
						}

						asset->m_lenght = dFloat32(size) / byteRate;
						asset->m_frequecy = dFloat32(sampleRate);
						alBufferData(asset->m_buffer, waveFormat, data, size, sampleRate);

						delete[] data;
					}
				}
			}
		}
		fclose(wave);
	}
}

void* dSoundManager::CreateSound(const char* const fileName)
{
	if (m_device)
	{
		char path[2048];
		dGetWorkingFileName(fileName, path);

		dUnsigned64 code = dCRC64(path);
		dSoundAssetList::dNode* assetNode = m_assets.Find(code);
		if (!assetNode)
		{
			assetNode = m_assets.Insert(code);
			LoadWaveFile(&assetNode->GetInfo(), path);
		}
		assetNode->GetInfo().AddRef();
		return assetNode;
	}
	return nullptr;
}

void dSoundManager::DestroySound(void* const soundAssetHandle)
{
	dAssert(0);
	if (m_device)
	{
		dSoundAssetList::dNode* const node = (dSoundAssetList::dNode*)soundAssetHandle;
		dAssert(node);

		dSoundAsset& asset = node->GetInfo();
		dAssert(asset.GetRef() > 1);
		asset.Release();

		m_assets.Remove(node);
	}
}

void dSoundManager::DestroyAllSound()
{
	dAssert(0);
	if (m_device)
	{
		while (m_assets.GetRoot())
		{
			DestroySound(m_assets.GetRoot());
		}
	}
}

dFloat32 dSoundManager::GetSoundlength(void* const soundAssetHandle)
{
	dAssert(0);
	if (m_device)
	{
		dSoundAssetList::dNode* const node = (dSoundAssetList::dNode*)soundAssetHandle;
		dSoundAsset& asset = node->GetInfo();
		return 	asset.m_lenght;
	}
	return 0;
}

void* dSoundManager::CreatePlayChannel(void* const soundAssetHandle)
{
	if (m_device)
	{
		dSoundAssetList::dNode* const node = (dSoundAssetList::dNode*)soundAssetHandle;
		dAssert(node);

		dSoundAsset& asset = node->GetInfo();
		dSoundChannelList::dNode* const channelNode = asset.Append();
		dSoundChannel& channel = channelNode->GetInfo();
		channel.LinkAsset(node);
		return channelNode;
	}
	return nullptr;
}

void dSoundManager::DestroyChannel(void* const channelHandle)
{
	dAssert(0);
	if (m_device)
	{
		dAssert(0);
	}
}

void* dSoundManager::GetAsset(void* const channelHandle) const
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.m_myAssetNode;
	}
	return nullptr;
}

void dSoundManager::PlayChannel(void* const channelHandle)
{
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		if (!channel.IsPlaying())
		{
			channel.Play();
			m_channelPlaying.Append(node);
		}
	}
}

void dSoundManager::StopChannel(void* const channelHandle)
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.Stop();
	}
}

dFloat32 dSoundManager::GetChannelVolume(void* const channelHandle) const
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.GetVolume();
	}
	return 0;
}

void dSoundManager::SetChannelVolume(void* const channelHandle, dFloat32 volume)
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetVolume(volume);
	}
}

void dSoundManager::SetChannelLoopMode(void* const channelHandle, bool mode)
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetLoop(mode);
	}
}

void dSoundManager::SetChannelPitch(void* const channelHandle, dFloat32 pitch)
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		channel.SetPitch(pitch);
	}
}

dFloat32 dSoundManager::GetChannelGetPosition(void* const channelHandle) const
{
	dAssert(0);
	if (m_device)
	{
		dSoundChannelList::dNode* const node = (dSoundChannelList::dNode*)channelHandle;
		dSoundChannel& channel = node->GetInfo();
		return channel.GetSecPosition();
	}
	return 0;
}

void dSoundManager::PostUpdate(ndWorld* const world, dFloat32)
{
	if (m_device)
	{
		dSoundChannelPlaying::dNode* next;
		for (dSoundChannelPlaying::dNode* node = m_channelPlaying.GetFirst(); node; node = next)
		{
			dSoundChannelList::dNode* const channelNode = node->GetInfo();
			next = node->GetNext();

			dSoundChannel& channel = channelNode->GetInfo();
			if (!channel.IsPlaying())
			{
				m_channelPlaying.Remove(node);
			}
			else
			{
			}
		}
	}
}

dSoundManager::dSoundChannel::dSoundChannel()
	:m_source(0)
	,m_myAssetNode(nullptr)
{
	alGenSources(1, (ALuint*)&m_source);
	dAssert (m_source);
	int xxxx = alGetError();
	xxxx = alGetError();
	dAssert (alGetError() == AL_NO_ERROR);
}

dSoundManager::dSoundChannel::~dSoundChannel()
{
	alDeleteSources(1, (ALuint*)&m_source);
	dAssert (alGetError() == AL_NO_ERROR);
}

void dSoundManager::dSoundChannel::LinkAsset( dTree<dSoundAsset, dUnsigned64>::dNode* const assetNode)
{
	m_myAssetNode = assetNode;
	dSoundAsset& asset = m_myAssetNode->GetInfo();
	alSourcei (m_source, AL_BUFFER, asset.m_buffer);
	int xxxx = alGetError();
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

void dSoundManager::dSoundChannel::SetPitch(dFloat32 pitch)
{
	alSourcef(m_source, AL_PITCH, pitch);
}

void dSoundManager::dSoundChannel::SetVolume(dFloat32 volume)
{
	alSourcef(m_source, AL_GAIN, volume);
}

dFloat32 dSoundManager::dSoundChannel::GetSecPosition() const
{
	ALfloat position;
	alGetSourcef(m_source, AL_SEC_OFFSET, &position);
	return position;
}

dFloat32 dSoundManager::dSoundChannel::GetVolume() const
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

#endif
