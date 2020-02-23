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

#ifndef __D_ANIM_POSE_h__
#define __D_ANIM_POSE_h__


class dAnimKeyframe
{
	public:
	dVector m_posit;
	dQuaternion m_rotation;
	void* m_userData;
};

class dAnimationPose: public dArray<dAnimKeyframe>
{
	public:
	dAnimationPose();
	dAnimationPose(const dAnimationPose& source);

	void Clear();
	void CopySource(const dAnimationPose& source);
};

class dAnimationLocalPose: public dAnimationPose
{
	public:
	dAnimationLocalPose(dAnimKeyframe* const buffer)
		:dAnimationPose()
	{
		m_capacity = 0x7fffffff;
		m_data = buffer;
	}

	~dAnimationLocalPose()
	{
		m_capacity = 0;
		m_data = NULL;
	}
};

#endif