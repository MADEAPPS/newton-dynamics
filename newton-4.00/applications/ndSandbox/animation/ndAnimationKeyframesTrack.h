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
	class dKeyFramesArray: public ndArray<OBJECT>
	{
		public:
		dKeyFramesArray()
			:ndArray<OBJECT>()
		{
		}
		dInt32 GetIndex(dFloat32 time) const;

		private:
		dInt32 GetIndexDebug(dFloat32 time) const;

		public:
		ndArray<dFloat32> m_time;
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

	void InterpolatePosition(dFloat32 time, dFloat32 length, ndVector &positOut) const;
	void InterpolateRotation(dFloat32 time, dFloat32 length, ndQuaternion& rotationOut) const;

	ndString m_name;
	dKeyFramesArray<ndVector> m_position;
	dKeyFramesArray<ndQuaternion> m_rotation;
};

#endif