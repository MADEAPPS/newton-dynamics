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
	class dAnimimationKeyFramesArray: public dArray<OBJECT>
	{
		public:
		dAnimimationKeyFramesArray()
			:dArray<OBJECT>()
		{
		}

		int GetIndex(dFloat32 t) const
		{
			dAssert(t >= 0.0f);
			const int size = GetCount();
			//const dAnimimationKeyFramesArray& me = *this;
			if (t > m_time[size - 1])
			{
				t = m_time[size - 1];
			}

			int i0 = 0;
			int i1 = size - 1;

			while ((i1 - i0) > 8) 
			{
				const int mid = (i1 + i0) / 2;
				if (t < m_time[mid])
				{
					i1 = mid;
				} 
				else 
				{
					i0 = mid;
				}
			}
			dAssert(m_time[i0] <= t);
			for (int i = i0 + 1; i < size; i++) 
			{
				if (m_time[i] >= t)
				{
					return i - 1;
				}
			}
			return i0;
		}

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

	const void InterpolatePosition(dFloat32 t, dVector &positOut) const;
	const void InterpolateRotation(dFloat32 t, dQuaternion& rotationOut) const;

	dString m_name;
	dAnimimationKeyFramesArray<dVector> m_position;
	dAnimimationKeyFramesArray<dQuaternion> m_rotation;
};

#endif