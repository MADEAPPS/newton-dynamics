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

#ifndef __D_ANIMATION_KEYFRAMES_TRACK_h__
#define __D_ANIMATION_KEYFRAMES_TRACK_h__

class ndAnimationKeyFramesTrack
{
	public:
	template<class OBJECT>
	class dKeyFramesArray: public dArray<OBJECT>
	{
		public:
		dKeyFramesArray()
			:dArray<OBJECT>()
		{
		}
		dInt32 GetIndex(dFloat32 time) const;

		dArray<dFloat32> m_time;
	};

	ndAnimationKeyFramesTrack()
		:m_position()
		,m_rotation()
	{
	}

	~ndAnimationKeyFramesTrack()
	{
	}

	const dString& GetName () const
	{
		return m_name;
	}

	void SetName (const dString& name)
	{
		m_name = name; 
	}

	const void InterpolatePosition(dFloat32 time, dFloat32 length, dVector &positOut) const;
	const void InterpolateRotation(dFloat32 time, dFloat32 length, dQuaternion& rotationOut) const;

	dString m_name;
	dKeyFramesArray<dVector> m_position;
	dKeyFramesArray<dQuaternion> m_rotation;
};

#endif