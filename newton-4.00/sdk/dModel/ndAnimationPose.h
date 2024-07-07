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

#ifndef __ND_ANIMATION_POSE_h__
#define __ND_ANIMATION_POSE_h__

class ndAnimKeyframe
{
	public:
	ndAnimKeyframe()
		:m_posit(ndVector::m_wOne)
		,m_rotation()
		,m_userData(nullptr)
	{
		m_userParamInt = 0;
	}

	ndAnimKeyframe(const ndVector& posit, const ndQuaternion& rotation)
		:m_posit(posit)
		,m_rotation(rotation)
		,m_userData(nullptr)
	{
		m_userParamInt = 0;
	}

	ndVector m_posit;
	ndQuaternion m_rotation;
	void* m_userData;

	union
	{
		ndInt64 m_userParamInt;
		ndFloat32 m_userParamFloat;
	};
};

class ndAnimationPose: public ndArray<ndAnimKeyframe>
{
	public:
	ndAnimationPose();
	ndAnimationPose(const ndAnimationPose& source);

	void Clear();
	void CopySource(const ndAnimationPose& source);
};

class ndAnimationLocalPose: public ndAnimationPose
{
	public:
	ndAnimationLocalPose(ndAnimKeyframe* const buffer)
		:ndAnimationPose()
	{
		m_array = buffer;
		m_capacity = 0x7fffffff;
	}

	~ndAnimationLocalPose()
	{
		m_capacity = 0;
		m_array = nullptr;
	}
};

#endif