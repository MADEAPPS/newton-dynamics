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

class ndSoundChannel: public ndClassAlloc
{
	public:
	ndSoundChannel();
	~ndSoundChannel();

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

	const ndVector GetPosition() const;
	void SetPosition(const ndVector& posit);
	
	const ndVector GetVelocity() const;
	void SetVelocity(const ndVector& velocity);

	void SetAttenuationRefDistance(dFloat32 refDist, dFloat32 minDropOffDist, dFloat32 maxDropOffDist);

	private:
	void ApplyAttenuation(const ndVector& listenerPosit);

	dInt32 m_source;
	ndSoundAsset* m_asset;
	ndSoundManager* m_manager;
	ndList<ndSoundChannel*>::ndNode* m_assetNode;
	ndList<ndSoundChannel*>::ndNode* m_playingNode;

	// since open-al does not check for parameter changes, we have to cache
	// them to prevent stuttering 
	ndVector m_posit;
	ndVector m_veloc;

	dFloat32 m_gain;
	dFloat32 m_pitch;
	dFloat32 m_volume;
	dFloat32 m_minDropOffDist;
	dFloat32 m_maxDropOffDist;

	friend class ndSoundAsset;
	friend class ndSoundManager;
};

class ndSoundChannelList: public ndList<ndSoundChannel*>
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
	ndTree<ndSoundAsset, dUnsigned64>::ndNode* m_node;
	friend class ndSoundManager;
};

class ndSoundAssetList: public ndTree<ndSoundAsset, dUnsigned64>
{
};

class ndSoundManager: public ndClassAlloc
{
	class ndSoundChannelPlaying: public ndList<ndSoundChannel*>
	{
	};
	
	public:
	ndSoundManager(ndDemoEntityManager* const scene);
	~ndSoundManager();

	// sound clip asset manager
	ndSoundAsset* CreateSoundAsset (const char* const fileName);

	// sound play tracks or channels 
	ndSoundChannel* CreateSoundChannel(const char* const fileName);

	void Update(ndWorld* const world, dFloat32 timestep);

	private:
	void LoadWaveFile(ndSoundAsset* const asset, const char* const fileName);

	ALCdevice* m_device;
	ALCcontext* m_context;
	ndDemoEntityManager* m_scene;
	ndSoundAssetList m_assets;
	ndSoundChannelPlaying m_channelPlaying;
	ndMatrix m_coordinateSystem;

	ndVector m_posit;
	ndVector m_veloc;
	ndVector m_posit0;
	ndVector m_upDir;
	ndVector m_frontDir;
	friend ndSoundChannel;
};

#endif