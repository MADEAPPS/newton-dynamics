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

#ifndef __DSOUND_MANAGER_H__
#define __DSOUND_MANAGER_H__

#include "ndSandboxStdafx.h"

class ndSoundAsset;
class ndSoundManager;
class ndDemoEntityManager;

class ndSoundChannel
{
	public:
	ndSoundChannel();
	virtual ~ndSoundChannel();

	bool IsPlaying() const;

	void Play();
	void Stop();
	
	bool GetLoop() const;
	void SetLoop(bool mode);
	
	dFloat32 GetPitch() const;
	void SetPitch(dFloat32 pitch);

	dFloat32 GetVolume() const;
	void SetVolume(dFloat32 volumne);

	dFloat32 GetLengthInSeconds() const;
	dFloat32 GetPositionInSeconds() const;

	const dVector GetPosition() const;
	void SetPosition(const dVector& posit);
	
	const dVector GetVelocity() const;
	void SetVelocity(const dVector& velocity) const;

	void SetAttenuationRefDistance(dFloat32 refDist, dFloat32 minDropOffDist, dFloat32 maxDropOffDist);

	private:
	void ApplyAttenuation(const dVector& listenerPosit);

	dInt32 m_source;
	ndSoundAsset* m_asset;
	ndSoundManager* m_manager;
	dList<ndSoundChannel*>::dNode* m_playingNode;

	// since open-al does not check for parameter changes, we have to cache
	// them to prevent stuttering 
	dVector m_posit;

	dFloat32 m_gain;
	dFloat32 m_pitch;
	dFloat32 m_volume;
	dFloat32 m_minDropOffDist;
	dFloat32 m_maxDropOffDist;


	friend class ndSoundManager;
};

class ndSoundChannelList: public dList<ndSoundChannel>
{
};

class ndSoundAsset: public ndSoundChannelList
{
	public:
	ndSoundAsset();
	ndSoundAsset(const ndSoundAsset& copy);
	virtual ~ndSoundAsset();

	dInt32 m_buffer;
	dFloat32 m_frequecy;
	dFloat32 m_durationInSeconds;
	dTree<ndSoundAsset, dUnsigned64>::dNode* m_node;
	friend class ndSoundManager;
};

class ndSoundAssetList: public dTree<ndSoundAsset, dUnsigned64>
{
};

class ndSoundManager: public dClassAlloc
{
	class ndSoundChannelPlaying: public dList<ndSoundChannel*>
	{
	};
	
	public:
	ndSoundManager(ndDemoEntityManager* const scene);
	~ndSoundManager();

	// sound clip asset manager
	ndSoundAsset* CreateSoundAsset (const char* const fileName);

	// sound play tracks or channels 
	ndSoundChannel* CreateSoundChannel(const char* const fileName);

	//void DestroyAllSound();
	//void DestroySound(void* const soundAssetHandle);
	//dFloat32 GetSoundlength (void* const soundAssetHandle);
	//void DestroyChannel(void* const channelHandle);
	//void* GetAsset(void* const channelHandle) const;

	void Update(ndWorld* const world, dFloat32 timestep);

	private:
	void LoadWaveFile(ndSoundAsset* const asset, const char* const fileName);

	ALCdevice* m_device;
	ALCcontext* m_context;
	ndDemoEntityManager* m_scene;
	ndSoundAssetList m_assets;
	ndSoundChannelPlaying m_channelPlaying;
	dMatrix m_coordinateSystem;

	dVector m_posit;
	dVector m_veloc;
	dVector m_posit0;
	dVector m_upDir;
	dVector m_frontDir;

	friend ndSoundChannel;
};

#endif