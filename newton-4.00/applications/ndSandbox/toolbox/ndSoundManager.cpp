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

#define DEFAULT_DISTANCE_MODEL AL_INVERSE_DISTANCE_CLAMPED

ndSoundChannel::ndSoundChannel()
	:m_source(0)
	,m_asset(nullptr)
	,m_manager(nullptr)
	,m_assetNode(nullptr)
	,m_playingNode(nullptr)
	,m_gain(ndFloat32 (1.0f))
	,m_pitch(ndFloat32(1.0f))
	,m_volume(ndFloat32(1.0f))
	,m_minDropOffDist(ndFloat32(30.0f))
	,m_maxDropOffDist(ndFloat32(50.0f))
{
	alGenSources(1, (ALuint*)&m_source);
	dAssert(m_source);
	dAssert(alGetError() == AL_NO_ERROR);

	ALfloat distanceModel = DEFAULT_DISTANCE_MODEL;
	alSourcefv(m_source, AL_DISTANCE_MODEL, &distanceModel);
	dAssert(alGetError() == AL_NO_ERROR);

	ALfloat posit[3];
	alGetSourcefv(m_source, AL_POSITION, posit);
	dAssert(alGetError() == AL_NO_ERROR);
	m_posit = ndVector (ndFloat32(posit[0]), ndFloat32(posit[1]), ndFloat32(posit[2]), ndFloat32(1.0f));

	ALfloat veloc[3];
	alGetSourcefv(m_source, AL_VELOCITY, veloc);
	dAssert(alGetError() == AL_NO_ERROR);
	m_veloc = ndVector(ndFloat32(veloc[0]), ndFloat32(veloc[1]), ndFloat32(veloc[2]), ndFloat32(0.0f));
}

ndSoundChannel::~ndSoundChannel()
{
	Stop();
	alDeleteSources(1, (ALuint*)&m_source);
	dAssert(alGetError() == AL_NO_ERROR);

	dAssert(m_asset);
	m_asset->Remove(m_assetNode);
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

void ndSoundChannel::SetVolume(ndFloat32 volume)
{
	if (dAbs(volume - m_volume) > ndFloat32(1.0e-3f))
	{
		m_volume = volume;
		alSourcef(m_source, AL_GAIN, ALfloat(m_volume));
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndFloat32 ndSoundChannel::GetVolume() const
{
	return m_volume;
}

void ndSoundChannel::SetAttenuationRefDistance(ndFloat32 dist, ndFloat32 minDropOffDist, ndFloat32 maxDropOffDist)
{
	alSourcef(m_source, AL_REFERENCE_DISTANCE, ALfloat(dist));
	dAssert(alGetError() == AL_NO_ERROR);

	dAssert(dist < minDropOffDist);
	dAssert(dist < minDropOffDist);
	m_minDropOffDist = minDropOffDist;
	m_maxDropOffDist = maxDropOffDist;
}

void ndSoundChannel::ApplyAttenuation(const ndVector& listenerPosit)
{
	ndFloat32 gain = ndFloat32(1.0f);
	const ndVector dist(m_posit - listenerPosit);
	ndFloat32 distance = ndSqrt(dist.DotProduct(dist).GetScalar());
	if (distance > m_minDropOffDist)
	{
		gain = ndFloat32(0.0f);
		if (distance < m_maxDropOffDist)
		{
			gain = ndFloat32(1.0f) - ((distance - m_minDropOffDist) / (m_maxDropOffDist - m_minDropOffDist));
		}
	}

	gain *= m_volume;
	if (dAbs(gain - m_gain) > ndFloat32(1.0e-3f))
	{
		m_gain = gain;
		alSourcef(m_source, AL_GAIN, ALfloat(m_gain));
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndFloat32 ndSoundChannel::GetPitch() const
{
	return m_pitch;
}

void ndSoundChannel::SetPitch(ndFloat32 pitch)
{
	if (dAbs(pitch - m_pitch) > ndFloat32(1.0e-3f))
	{
		m_pitch = pitch;
		alSourcef(m_source, AL_PITCH, ALfloat(m_pitch));
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndFloat32 ndSoundChannel::GetLengthInSeconds() const
{
	return m_asset->m_durationInSeconds;
}

ndFloat32 ndSoundChannel::GetPositionInSeconds() const
{
	ALfloat position;
	alGetSourcef(m_source, AL_SEC_OFFSET, &position);
	dAssert(alGetError() == AL_NO_ERROR);
	return position;
}

const ndVector ndSoundChannel::GetPosition() const
{
	return m_manager->m_coordinateSystem.UntransformVector(m_posit);
}

void ndSoundChannel::SetPosition(const ndVector& position)
{
	const ndVector posit(m_manager->m_coordinateSystem.TransformVector(position));
	const ndVector err(posit - m_posit);
	if (err.DotProduct(err).GetScalar() > ndFloat32(1.0f))
	{
		ALfloat sourcePosition[3];
		m_posit = posit;
		sourcePosition[0] = ALfloat(posit.m_x);
		sourcePosition[1] = ALfloat(posit.m_y);
		sourcePosition[2] = ALfloat(posit.m_z);
		alSourcefv(m_source, AL_POSITION, sourcePosition);
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

const ndVector ndSoundChannel::GetVelocity() const
{
	return m_manager->m_coordinateSystem.UnrotateVector(m_veloc);
}

void ndSoundChannel::SetVelocity(const ndVector& velocity)
{
	const ndVector veloc(m_manager->m_coordinateSystem.RotateVector(velocity));
	const ndVector err(veloc - m_veloc);
	if (err.DotProduct(err).GetScalar() > ndFloat32(0.25f))
	{
		m_veloc = veloc & ndVector::m_triplexMask;
		ALfloat sourceVeloc[3];
		sourceVeloc[0] = ALfloat(veloc.m_x);
		sourceVeloc[1] = ALfloat(veloc.m_y);
		sourceVeloc[2] = ALfloat(veloc.m_z);
		alSourcefv(m_source, AL_VELOCITY, sourceVeloc);
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndSoundAsset::ndSoundAsset()
	:ndSoundChannelList()
	,m_buffer(0)
	,m_frequecy(0)
	,m_durationInSeconds(0)
	,m_node(nullptr)
{
}

ndSoundAsset::ndSoundAsset(const ndSoundAsset&)
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

ndSoundAsset::~ndSoundAsset()
{
	if (m_buffer)
	{
		while (GetCount())
		{
			ndSoundChannel* const channel = GetFirst()->GetInfo();
			delete channel;
		}

		alDeleteBuffers(1, (ALuint *)&m_buffer);
		dAssert(alGetError() == AL_NO_ERROR);
	}
}

ndSoundManager::ndSoundManager(ndDemoEntityManager* const scene)
	:ndClassAlloc()
	,m_device(alcOpenDevice(nullptr))
	,m_context(nullptr)
	,m_scene(scene)
	,m_assets()
	,m_channelPlaying()
	,m_coordinateSystem(dYawMatrix (90.0f * ndDegreeToRad))
	,m_posit(ndVector::m_zero)
	,m_veloc(ndVector::m_zero)
	,m_posit0(ndVector::m_zero)
	,m_upDir(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f))
	,m_frontDir(ndFloat32 (0.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f))
{
	dAssert(m_device);
	if (m_device)
	{
		m_context = alcCreateContext(m_device, nullptr);
		alcMakeContextCurrent(m_context);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerPosit[] = { 0.0f, 0.0f, 0.0f };
		alListenerfv(AL_POSITION, listenerPosit);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerVeloc[] = { 0.0f, 0.0f, 0.0f };
		alListenerfv(AL_VELOCITY, listenerVeloc);
		dAssert(alGetError() == AL_NO_ERROR);
	
		ALfloat listenerOri[] = { 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f };
		alListenerfv(AL_ORIENTATION, listenerOri);
		dAssert(alGetError() == AL_NO_ERROR);

		alDistanceModel(DEFAULT_DISTANCE_MODEL);
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
		size_t bytesRead = fread(xbuffer, sizeof(char), 4, wave);
		if (!strcmp(xbuffer, "RIFF"))
		{
			ndInt32 chunkSize;
			bytesRead = fread(&chunkSize, sizeof(ndInt32), 1, wave);
			bytesRead = fread(xbuffer, sizeof(char), 4, wave);
			if (!strcmp(xbuffer, "WAVE"))
			{
				bytesRead = fread(xbuffer, sizeof(char), 4, wave);
				if (!strcmp(xbuffer, "fmt "))
				{
					bytesRead = fread(&chunkSize, sizeof(ndInt32), 1, wave);

					ndInt32 sampleRate;
					ndInt32 byteRate;
					short channels;
					short audioFormat;
					short blockAlign;
					short bitsPerSample;

					bytesRead = fread(&audioFormat, sizeof(short), 1, wave);
					bytesRead = fread(&channels, sizeof(short), 1, wave);
					bytesRead = fread(&sampleRate, sizeof(ndInt32), 1, wave);
					bytesRead = fread(&byteRate, sizeof(ndInt32), 1, wave);
					bytesRead = fread(&blockAlign, sizeof(short), 1, wave);
					bytesRead = fread(&bitsPerSample, sizeof(short), 1, wave);
					for (ndInt32 i = 0; i < (chunkSize - 16); i++)
					{
						bytesRead = fread(xbuffer, sizeof(char), 1, wave);
					}

					#define WAVE_FORMAT_PCM 0x0001
					//0x0003 WAVE_FORMAT_IEEE_FLOAT IEEE float 
					//0x0006 WAVE_FORMAT_ALAW 8-bit ITU-T G.711 A-law 
					//0x0007 WAVE_FORMAT_MULAW 8-bit ITU-T G.711 µ-law 
					//0xFFFE WAVE_FORMAT_EXTENSIBLE Determined by SubFormat 

					// I only parse WAVE_FORMAT_PCM format
					dAssert(audioFormat == WAVE_FORMAT_PCM);

					bytesRead = fread(xbuffer, sizeof(char), 4, wave);
					if (!strcmp(xbuffer, "fact"))
					{
						ndInt32 size;
						ndInt32 samplesPerChannels;
						bytesRead = fread(&size, sizeof(ndInt32), 1, wave);
						bytesRead = fread(&samplesPerChannels, sizeof(ndInt32), 1, wave);
						bytesRead = fread(xbuffer, sizeof(char), 4, wave);
					}

					if (!strcmp(xbuffer, "data"))
					{
						ndInt32 size;
						bytesRead = fread(&size, sizeof(ndInt32), 1, wave);

						ndArray<char> data;
						data.SetCount(size);
						bytesRead = fread(&data[0], sizeof(char), size, wave);

						ndInt32 waveFormat = AL_FORMAT_MONO8;
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

						asset->m_frequecy = ndFloat32(sampleRate);
						asset->m_durationInSeconds = ndFloat32(size) / byteRate;
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
	ndSoundAssetList::ndNode* assetNode = nullptr;
	if (m_device)
	{
		ndUnsigned64 code = dCRC64(fileName);
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
		ndUnsigned64 code = dCRC64(fileName);
		ndSoundAssetList::ndNode* const assetNode = m_assets.Find(code);
		dAssert(assetNode);
	
		ndSoundAsset& asset = assetNode->GetInfo();

		channel = new ndSoundChannel;
		channel->m_assetNode = asset.Append(channel);
		channel->m_asset = &asset;
		channel->m_manager = this;
		alSourcei(channel->m_source, AL_BUFFER, asset.m_buffer);
		dAssert(alGetError() == AL_NO_ERROR);
	}
	return channel;
}

void ndSoundManager::Update(ndWorld* const, ndFloat32 timestep)
{
	if (m_device)
	{
		// get camera matrix in open-al space
		ndDemoCamera* const camera = m_scene->GetCamera();
		const ndMatrix matrix(camera->GetCurrentMatrix() * m_coordinateSystem);

		ndVector err(matrix.m_posit - m_posit);
		if (err.DotProduct(err).GetScalar() > ndFloat32 (0.25f))
		{
			// set Listener position
			ALfloat listenerPosit[3];
			m_posit = matrix.m_posit;
			listenerPosit[0] = ALfloat(matrix.m_posit.m_x);
			listenerPosit[1] = ALfloat(matrix.m_posit.m_y);
			listenerPosit[2] = ALfloat(matrix.m_posit.m_z);
			alListenerfv(AL_POSITION, listenerPosit);
			dAssert(alGetError() == AL_NO_ERROR);
		}

		ndVector veloc((matrix.m_posit - m_posit0).Scale(1.0f / timestep));
		err = veloc - m_veloc;
		if (err.DotProduct(err).GetScalar() > ndFloat32(1.0f))
		{
			// estimate listener velocity, by using camera previous location
			m_veloc = veloc;
			ALfloat listenerVeloc[3];
			listenerVeloc[0] = ALfloat(veloc.m_x);
			listenerVeloc[1] = ALfloat(veloc.m_y);
			listenerVeloc[2] = ALfloat(veloc.m_z);
			alListenerfv(AL_VELOCITY, listenerVeloc);
			dAssert(alGetError() == AL_NO_ERROR);
		}
		m_posit0 = matrix.m_posit;

		ndFloat32 angle0 = matrix.m_up.DotProduct(m_upDir).GetScalar();
		ndFloat32 angle1 = matrix.m_front.DotProduct(m_frontDir).GetScalar();
		if ((angle0 < ndFloat32(0.999f)) || (angle1 < ndFloat32(0.999f)))
		{
			// update camera if is changes more than 2.5 degrees for previous orientation
			m_upDir = matrix.m_up;
			m_frontDir = matrix.m_front;

			// set Listener orientation
			//{ 0.0, 0.0, -1.0, 0.0, 1.0, 0.0 }
			ALfloat listenerOrientation[6];
			listenerOrientation[0] = (ALfloat)m_frontDir.m_x;
			listenerOrientation[1] = (ALfloat)m_frontDir.m_y;
			listenerOrientation[2] = (ALfloat)m_frontDir.m_z;
			listenerOrientation[3] = (ALfloat)m_upDir.m_x;
			listenerOrientation[4] = (ALfloat)m_upDir.m_y;
			listenerOrientation[5] = (ALfloat)m_upDir.m_z;
			alListenerfv(AL_ORIENTATION, listenerOrientation);
			dAssert(alGetError() == AL_NO_ERROR);
		}

		//dTrace(("p(%f %f %f)", m_posit[0], m_posit[1], m_posit[2]));
		//dTrace(("v(%f %f %f)", m_veloc[0], m_veloc[1], m_veloc[2]));
		//dTrace(("\n"));

		ndSoundChannelPlaying::ndNode* next;
		for (ndSoundChannelPlaying::ndNode* node = m_channelPlaying.GetFirst(); node; node = next)
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
