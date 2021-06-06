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

#if 0
class dSoundManager  
{
	class dSoundAsset;
	class dSoundChannel;
	typedef void (*dEvenCallback) (void* const channelHandle, void* const userData, void* const evenHandle);

	class dSoundChannel: public dRefCounter
	{
		public:
		dSoundChannel();
		virtual ~dSoundChannel();

		void Play();
		void Stop();
	
		void SetLoop(bool mode);
		void SetPitch(dFloat pitch);
		void SetVolume(dFloat volumne);

		bool IsPlaying() const;
		dFloat GetVolume() const;
		dFloat GetSecPosition() const;


		private:
		void LinkAsset (dTree<dSoundAsset, dUnsigned64>::dTreeNode* const assetNode);

		dInt32 m_source;
		dTree<dSoundAsset, dUnsigned64>::dTreeNode* m_myAssetNode;
		friend class dSoundManager;
	};

	class dSoundChannelList: public dList<dSoundChannel>
	{
	};

	class dSoundAsset: public dSoundChannelList, virtual public dRefCounter
	{
		public:
		dSoundAsset();
		virtual ~dSoundAsset();

		using dRefCounter::operator new;
		using dRefCounter::operator delete;


		dInt32 m_buffer;
		dFloat m_lenght;
		dFloat m_frequecy;
	};

	class dSoundAssetList: public dTree<dSoundAsset, dUnsigned64>
	{
	};

	class dSoundChannelPlaying: public dList<dSoundChannelList::dListNode*>
	{
	};
	
	public:
	dSoundManager();
	~dSoundManager();

	void Update();
	void UpdateListener(const dVector& position, const dVector& velocity, const dVector& heading, const dVector& upDir);
	void DestroyAllSound();

	// sound clip asset manager
	void* CreateSound (const char* const fileName);
	void DestroySound(void* const soundAssetHandle);
	dFloat GetSoundlength (void* const soundAssetHandle);

	// sound play tracks or channels 
	void* CreatePlayChannel (void* const soundAssetHandle);
	void DestroyChannel(void* const channelHandle);

	void* GetAsset(void* const channelHandle) const;
	dFloat GetChannelVolume(void* const channelHandle) const;
	dFloat GetChannelGetPosition(void* const channelHandle) const;

	void PlayChannel (void* const channelHandle);
	void StopChannel (void* const channelHandle);
	
	void SetChannelVolume(void* const channelHandle, dFloat volume);
	void SetChannelPitch(void* const channelHandle, dFloat pitch);
	void SetChannelLoopMode (void* const channelHandle, bool mode);


	private:
	void LoadWaveFile(dSoundAsset* const asset, const char* const fileName);

	ALCdevice* m_device;
	ALCcontext* m_context;

	dSoundAssetList m_assets;
	dSoundChannelPlaying m_channelPlaying;
	dMatrix m_coordinateSystem;
};
#endif
#endif