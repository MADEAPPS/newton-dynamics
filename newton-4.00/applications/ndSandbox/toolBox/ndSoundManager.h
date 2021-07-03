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
#include "ndDemoEntity.h"

class dSoundChannel: public dRefCounter<dSoundChannel>
{
	public:
	dSoundChannel();
	virtual ~dSoundChannel();

	//void Play();
	//void Stop();
	//
	//void SetLoop(bool mode);
	//void SetPitch(dFloat32 pitch);
	//void SetVolume(dFloat32 volumne);
	//
	//bool IsPlaying() const;
	//dFloat32 GetVolume() const;
	//dFloat32 GetSecPosition() const;
	//
	//private:
	//void LinkAsset (dTree<dSoundAsset, dUnsigned64>::dNode* const assetNode);
	//
	//dInt32 m_source;
	//dTree<dSoundAsset, dUnsigned64>::dNode* m_myAssetNode;
	//friend class ndSoundManager;
};

class dSoundChannelList: public dList<dSoundChannel>
{
};

class dSoundAsset: public dSoundChannelList
{
	public:
	dSoundAsset();
	dSoundAsset(const dSoundAsset& copy);
	virtual ~dSoundAsset();

	//using dRefCounter::operator new;
	//using dRefCounter::operator delete;

	dInt32 m_buffer;
	dFloat32 m_lenght;
	dFloat32 m_frequecy;
};

class dSoundAssetList: public dTree<dSoundAsset, dUnsigned64>
{
};

class ndSoundManager: public ndModel
{
	//class dSoundAsset;
	//class dSoundChannel;
	//typedef void (*dEvenCallback) (void* const channelHandle, void* const userData, void* const evenHandle);
	
	//class dSoundChannelPlaying: public dList<dSoundChannelList::dNode*>
	//{
	//};
	
	public:
	ndSoundManager();
	~ndSoundManager();

	//void DestroyAllSound();
	//
	//// sound clip asset manager
	dSoundAssetList::dNode* CreateSoundAsset (const char* const fileName);
	//void DestroySound(void* const soundAssetHandle);
	//
	//dFloat32 GetSoundlength (void* const soundAssetHandle);
	//
	//// sound play tracks or channels 
	//void* CreatePlayChannel (void* const soundAssetHandle);
	//void DestroyChannel(void* const channelHandle);
	//
	//void* GetAsset(void* const channelHandle) const;
	//dFloat32 GetChannelVolume(void* const channelHandle) const;
	//dFloat32 GetChannelGetPosition(void* const channelHandle) const;
	//
	//void PlayChannel (void* const channelHandle);
	//void StopChannel (void* const channelHandle);
	//
	//void SetChannelVolume(void* const channelHandle, dFloat32 volume);
	//void SetChannelPitch(void* const channelHandle, dFloat32 pitch);
	//void SetChannelLoopMode (void* const channelHandle, bool mode);
	//
	//void Update(ndWorld* const, dFloat32) {}
	//void PostUpdate(ndWorld* const world, dFloat32);
	//
	//private:
	//void UpdateListener(const dVector& position, const dVector& velocity, const dVector& heading, const dVector& upDir);
	//void LoadWaveFile(dSoundAsset* const asset, const char* const fileName);
	//
	//ALCdevice* m_device;
	//ALCcontext* m_context;
	//
	//dSoundAssetList m_assets;
	//dSoundChannelPlaying m_channelPlaying;
	//dMatrix m_coordinateSystem;

	private:
	void Update(ndWorld* const, dFloat32) {}
	void PostUpdate(ndWorld* const world, dFloat32);

	void LoadWaveFile(dSoundAsset* const asset, const char* const fileName);

	ALCdevice* m_device;
	ALCcontext* m_context;
	dSoundAssetList m_assets;
	//dMatrix m_coordinateSystem;
};


#endif