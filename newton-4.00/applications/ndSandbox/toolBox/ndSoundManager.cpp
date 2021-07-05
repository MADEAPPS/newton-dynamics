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
#include "ndDemoCamera.h"
#include "ndSoundManager.h"
#include "ndDemoEntityManager.h"

#if 0
void ndSoundManager::DestroySound(void* const soundAssetHandle)
{
	dAssert(0);
	if (m_device)
	{
		ndSoundAssetList::dNode* const node = (ndSoundAssetList::dNode*)soundAssetHandle;
		dAssert(node);

		ndSoundAsset& asset = node->GetInfo();
		dAssert(asset.GetRef() > 1);
		asset.Release();

		m_assets.Remove(node);
	}
}

void ndSoundManager::DestroyAllSound()
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

dFloat32 ndSoundManager::GetSoundlength(void* const soundAssetHandle)
{
	dAssert(0);
	if (m_device)
	{
		ndSoundAssetList::dNode* const node = (ndSoundAssetList::dNode*)soundAssetHandle;
		ndSoundAsset& asset = node->GetInfo();
		return 	asset.m_lenght;
	}
	return 0;
}


void ndSoundManager::DestroyChannel(void* const channelHandle)
{
	dAssert(0);
	if (m_device)
	{
		dAssert(0);
	}
}

void* ndSoundManager::GetAsset(void* const channelHandle) const
{
	dAssert(0);
	if (m_device)
	{
		ndSoundChannelList::dNode* const node = (ndSoundChannelList::dNode*)channelHandle;
		ndSoundChannel& channel = node->GetInfo();
		return channel.m_myAssetNode;
	}
	return nullptr;
}

#endif

ndSoundChannel::ndSoundChannel()
	:m_source(0)
	,m_asset(nullptr)
	,m_manager(nullptr)
	,m_playingNode(nullptr)
	,m_gain(0.0f)
	,m_maxDistance(60.0f)
	,m_referenceDistance(40.0f)
{
	alGenSources(1, (ALuint*)&m_source);
	dAssert(m_source);
	dAssert(alGetError() == AL_NO_ERROR);

	//alSourcef(m_source, AL_ROLLOFF_FACTOR, ALfloat(0.0f));
	//alSourcef(m_source, AL_REFERENCE_DISTANCE, ALfloat(1000.0f));
	//dAssert(alGetError() == AL_NO_ERROR);
}

ndSoundChannel::~ndSoundChannel()
{
	Stop();
	alDeleteSources(1, (ALuint*)&m_source);
	dAssert(alGetError() == AL_NO_ERROR);
}

bool ndSoundChannel::GetLoop() const
{
	ALint state;
	alGetSourcei(m_source, AL_LOOPING, &state);
	return state ? true : false;
}

void ndSoundChannel::SetLoop(bool mode)
{
	alSourcei(m_source, AL_LOOPING, mode ? 1 : 0);
}

bool ndSoundChannel::IsPlaying() const
{
	ALint state;
	alGetSourcei(m_source, AL_SOURCE_STATE, &state);
	return (state == AL_PLAYING) ? true : false;
}

void ndSoundChannel::Play()
{
	// set some default values
	if (!IsPlaying())
	{
		dAssert(!m_playingNode);
		alSourcePlay(m_source);
		dAssert(alGetError() == AL_NO_ERROR);
		m_playingNode = m_manager->m_channelPlaying.Append(this);
	}
}

void ndSoundChannel::Stop()
{
	alSourceStop(m_source);
	dAssert(alGetError() == AL_NO_ERROR);
	if (m_playingNode)
	{
		m_manager->m_channelPlaying.Remove(m_playingNode);
		m_playingNode = nullptr;
	}
}

void ndSoundChannel::SetVolume(dFloat32 volume)
{
	ALfloat vol = ALfloat(volume);
	m_gain = volume;
	alSourcef(m_source, AL_GAIN, vol);
	dAssert(alGetError() == AL_NO_ERROR);
}

dFloat32 ndSoundChannel::GetVolume() const
{
	ALfloat volume;
	alGetSourcef(m_source, AL_GAIN, &volume);
	dAssert(alGetError() == AL_NO_ERROR);
	return volume;
}

void ndSoundChannel::ApplyAttenuation(const dVector& listenerPosit)
{
	//dFloat32 ROLLOFF_FACTOR = 1.0f;
	// for some reason the attenuation model does not works in open-al
	// so I am applying manually, according to the formula in the docs
	ALfloat sourcePosit[3];
	alGetSourcefv(m_source, AL_POSITION, sourcePosit);
	dAssert(alGetError() == AL_NO_ERROR);
	
	const dVector posit (dFloat32(sourcePosit[0]), dFloat32(sourcePosit[1]), dFloat32(sourcePosit[2]), dFloat32(1.0f));
	const dVector dist(posit - listenerPosit);
	dFloat32 distance = dSqrt(dist.DotProduct(dist).GetScalar());
	//distance = dMin(distance, m_maxDistance);
	//distance = dMax(distance, m_referenceDistance);
	//
	//dFloat32 attenuation = ROLLOFF_FACTOR * (dFloat32 (1.0f) - (distance - m_referenceDistance) / (m_maxDistance - m_referenceDistance));
	//dTrace(("%f %f\n", attenuation, m_gain));
	//alSourcef(m_source, AL_GAIN, ALfloat(m_gain * attenuation));
	//dAssert(alGetError() == AL_NO_ERROR);
}

dFloat32 ndSoundChannel::GetPitch() const
{
	ALfloat pitch;
	alGetSourcef(m_source, AL_PITCH, &pitch);
	return pitch;
}

void ndSoundChannel::SetPitch(dFloat32 pitch)
{
	ALfloat pit = ALfloat(pitch);
	alSourcef(m_source, AL_PITCH, pit);
	dAssert(alGetError() == AL_NO_ERROR);
}

dFloat32 ndSoundChannel::GetLengthInSeconds() const
{
	return m_asset->m_durationInSeconds;
}

dFloat32 ndSoundChannel::GetPositionInSeconds() const
{
	ALfloat position;
	alGetSourcef(m_source, AL_SEC_OFFSET, &position);
	dAssert(alGetError() == AL_NO_ERROR);
	return position;
}

const dVector ndSoundChannel::GetPosition() const
{
	ALfloat sourcePosition[3];
	alGetSourcefv(m_source, AL_POSITION, sourcePosition);
	dAssert(alGetError() == AL_NO_ERROR);
	const dVector posit(dFloat32 (sourcePosition[0]), dFloat32(sourcePosition[1]), dFloat32(sourcePosition[2]), dFloat32 (1.0f));
	return m_manager->m_coordinateSystem.UntransformVector(posit);
}

void ndSoundChannel::SetPosition(const dVector& posit) const
{
	const dVector alPosit(m_manager->m_coordinateSystem.TransformVector(posit));
	ALfloat sourcePosition[3];
	sourcePosition[0] = ALfloat(alPosit.m_x);
	sourcePosition[1] = ALfloat(alPosit.m_y);
	sourcePosition[2] = ALfloat(alPosit.m_z);
	alSourcefv(m_source, AL_POSITION, sourcePosition);
	dAssert(alGetError() == AL_NO_ERROR);
}

const dVector ndSoundChannel::GetVelocity() const
{
	ALfloat sourceVeloc[3];
	alGetSourcefv(m_source, AL_VELOCITY, sourceVeloc);
	dAssert(alGetError() == AL_NO_ERROR);
	const dVector veloc(dFloat32(sourceVeloc[0]), dFloat32(sourceVeloc[1]), dFloat32(sourceVeloc[2]), dFloat32(1.0f));
	return m_manager->m_coordinateSystem.UnrotateVector(veloc);
}

void ndSoundChannel::SetVelocity(const dVector& velocity) const
{
	const dVector alVeloc(m_manager->m_coordinateSystem.RotateVector(velocity));
	ALfloat sourceVeloc[3];
	sourceVeloc[0] = ALfloat(alVeloc.m_x);
	sourceVeloc[1] = ALfloat(alVeloc.m_y);
	sourceVeloc[2] = ALfloat(alVeloc.m_z);
	alSourcefv(m_source, AL_VELOCITY, sourceVeloc);
	dAssert(alGetError() == AL_NO_ERROR);
}

ndSoundAsset::ndSoundAsset()
	:ndSoundChannelList()
	,m_buffer(0)
	,m_frequecy(0)
	,m_durationInSeconds(0)
	,m_node(nullptr)
{
	alGenBuffers(1, (ALuint*)&m_buffer);
	dAssert(m_buffer);
	dAssert(alGetError() == AL_NO_ERROR);
}

ndSoundAsset::ndSoundAsset(const ndSoundAsset& copy)
	:ndSoundChannelList()
	,m_buffer(copy.m_buffer)
	,m_frequecy(copy.m_frequecy)
	,m_durationInSeconds(copy.m_durationInSeconds)
	,m_node(nullptr)
{
	alGenBuffers(1, (ALuint*)&m_buffer);
	dAssert(m_buffer);
	dAssert(alGetError() == AL_NO_ERROR);
	dAssert(copy.GetCount() == 0);
}

ndSoundAsset::~ndSoundAsset()
{
	RemoveAll();
	alDeleteBuffers(1, (ALuint *)&m_buffer);
	dAssert(alGetError() == AL_NO_ERROR);
}

ndSoundManager::ndSoundManager(ndDemoEntityManager* const scene)
	:ndModel()
	,m_device(alcOpenDevice(nullptr))
	,m_context(nullptr)
	,m_scene(scene)
	,m_assets()
	,m_channelPlaying()
	,m_coordinateSystem(dYawMatrix (90.0f * dDegreeToRad))
	,m_cameraPreviousPosit(dVector::m_zero)
{
	dAssert(m_device);
	if (m_device)
	{
		m_context = alcCreateContext(m_device, nullptr);
		alcMakeContextCurrent(m_context);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerPosit[] = { 0.0,0.0,0.0 };
		alListenerfv(AL_POSITION, listenerPosit);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerVeloc[] = { 0.0,0.0,0.0 };
		alListenerfv(AL_VELOCITY, listenerVeloc);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerOri[] = { 0.0,0.0,-1.0, 0.0,1.0,0.0 };
		alListenerfv(AL_ORIENTATION, listenerOri);
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndSoundManager::~ndSoundManager()
{
	if (m_device)
	{
		m_assets.RemoveAll();
		alcDestroyContext(m_context);
		alcCloseDevice(m_device);
	}
}

void ndSoundManager::LoadWaveFile(ndSoundAsset* const asset, const char* const fileName)
{
	char path[2048];
	dGetWorkingFileName(fileName, path);

	FILE* const wave = fopen(path, "rb");
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

						dArray<char> data;
						data.SetCount(size);
						fread(&data[0], sizeof(char), size, wave);

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

						asset->m_frequecy = dFloat32(sampleRate);
						asset->m_durationInSeconds = dFloat32(size) / byteRate;
						alBufferData(asset->m_buffer, waveFormat, &data[0], size, sampleRate);
					}
				}
			}
		}
		fclose(wave);
	}
}

ndSoundAsset* ndSoundManager::CreateSoundAsset(const char* const fileName)
{
	ndSoundAssetList::dNode* assetNode = nullptr;
	if (m_device)
	{
		dUnsigned64 code = dCRC64(fileName);
		assetNode = m_assets.Find(code);
		if (!assetNode)
		{
			assetNode = m_assets.Insert(code);
			LoadWaveFile(&assetNode->GetInfo(), fileName);
			assetNode->GetInfo().m_node = assetNode;
		}
	}
	return &assetNode->GetInfo();
}

ndSoundChannel* ndSoundManager::CreateSoundChannel(const char* const fileName)
{
	ndSoundChannel* channel = nullptr;
	if (m_device)
	{
		dUnsigned64 code = dCRC64(fileName);
		ndSoundAssetList::dNode* const assetNode = m_assets.Find(code);
		dAssert(assetNode);
	
		ndSoundAsset& asset = assetNode->GetInfo();
		ndSoundChannelList::dNode* const channelNode = asset.Append();
		channel = &channelNode->GetInfo();

		channel->m_asset = &asset;
		channel->m_manager = this;
		alSourcei(channel->m_source, AL_BUFFER, asset.m_buffer);
		dAssert(alGetError() == AL_NO_ERROR);
	}
	return channel;
}

void ndSoundManager::PostUpdate(ndWorld* const, dFloat32 timestep)
{
	if (m_device)
	{
		// get camera matrix in open-al space
		ndDemoCamera* const camera = m_scene->GetCamera();
		const dMatrix matrix(camera->GetCurrentMatrix() * m_coordinateSystem);

		// set Listener position
		ALfloat listenerPosit[3];
		listenerPosit[0] = matrix.m_posit.m_x;
		listenerPosit[1] = matrix.m_posit.m_y;
		listenerPosit[2] = matrix.m_posit.m_z;
		alListenerfv(AL_POSITION, listenerPosit);
		dAssert(alGetError() == AL_NO_ERROR);

		// set Listener orientation
		//{ 0.0, 0.0, -1.0, 0.0, 1.0, 0.0 }
		ALfloat listenerOrientation[6];
		listenerOrientation[0] = (ALfloat)matrix.m_front.m_x;
		listenerOrientation[1] = (ALfloat)matrix.m_front.m_y;
		listenerOrientation[2] = (ALfloat)matrix.m_front.m_z;
		listenerOrientation[3] = (ALfloat)matrix.m_up.m_x;
		listenerOrientation[4] = (ALfloat)matrix.m_up.m_y;
		listenerOrientation[5] = (ALfloat)matrix.m_up.m_z;
		alListenerfv(AL_ORIENTATION, listenerOrientation);
		dAssert(alGetError() == AL_NO_ERROR);

		// estimate listener velocity, by using camera previous location
		const dVector camVelocity((matrix.m_posit - m_cameraPreviousPosit).Scale(1.0f / timestep));
		ALfloat listenerVeloc[3];
		listenerVeloc[0] = camVelocity.m_x;
		listenerVeloc[1] = camVelocity.m_y;
		listenerVeloc[2] = camVelocity.m_z;
		alListenerfv(AL_VELOCITY, listenerVeloc);
		dAssert(alGetError() == AL_NO_ERROR);

		m_cameraPreviousPosit = matrix.m_posit;

		ndSoundChannelPlaying::dNode* next;
		for (ndSoundChannelPlaying::dNode* node = m_channelPlaying.GetFirst(); node; node = next)
		{
			ndSoundChannel* const channel = node->GetInfo();
			next = node->GetNext();

			if (!channel->IsPlaying())
			{
				channel->Stop();
			}
			else
			{
				channel->ApplyAttenuation(matrix.m_posit);
				#if 0
				// test positional sound
				static dFloat32 xxx;
				xxx += timestep;
				dFloat32 axxx = 10.0f * dSin(xxx / 1.1f);
				dVector xxx1(channel->GetPosition());
				//dVector xxx1(channel->GetVelocity());
				xxx1.m_z = axxx;
				dTrace(("%f\n", axxx));
				channel->SetPosition(xxx1);
				//channel->SetVelocity(xxx1);
				#endif
			}
		}

		//alDopplerFactor(1.0f);
	}
}
