/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_ANIMATION_KEYFRAMES_TRACK_h__
#define __ND_ANIMATION_KEYFRAMES_TRACK_h__

class ndAnimationKeyFramesTrack
{
	public:
	template<class OBJECT>
	class ndKeyFramesArray: public ndArray<OBJECT>
	{
		public:
		ndKeyFramesArray()
			:ndArray<OBJECT>()
		{
		}
		ndInt32 GetIndex(ndFloat32 time) const;

		public:
		ndArray<ndFloat32> m_time;
	};

	ndAnimationKeyFramesTrack()
		:m_position()
		,m_rotation()
	{
	}

	~ndAnimationKeyFramesTrack()
	{
	}

	const ndString& GetName () const
	{
		return m_name;
	}

	void SetName (const ndString& name)
	{
		m_name = name; 
	}

	void InterpolatePosition(ndFloat32 param, ndVector &positOut) const;
	void InterpolateRotation(ndFloat32 param, ndQuaternion& rotationOut) const;

	ndString m_name;
	ndKeyFramesArray<ndVector> m_position;
	ndKeyFramesArray<ndQuaternion> m_rotation;
};

#endif