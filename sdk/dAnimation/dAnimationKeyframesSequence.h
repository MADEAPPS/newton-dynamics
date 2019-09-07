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

#ifndef __D_ANIMATION_KEYFRAMES_SEQUENCE_h__
#define __D_ANIMATION_KEYFRAMES_SEQUENCE_h__
//#include "dAnimPose.h"
//#include "dAnimIKBlendNode.h"

class dAnimPose;

class dAnimimationKeyFramesTrack
{
	public:
	class dPositionKey
	{
		public:
		dVector m_posit;
		dFloat m_time;
	};

	class dRotationKey
	{
		public:
		dQuaternion m_rotation;
		dFloat m_time;
	};

	template<class OBJECT>
	class dAnimTakeArray: public dArray<OBJECT>
	{
		public:
		dAnimTakeArray()
			:dArray<OBJECT>()
		{
		}

		int GetIndex(dFloat t) const
		{
			dAssert(t >= 0.0f);
			const int size = this->GetSize();
			if (t > this->m_data[size - 1].m_time) {
				t = this->m_data[size - 1].m_time;
			}

			int i0 = 0;
			int i1 = size - 1;

			while ((i1 - i0) > 8) {
				const int mid = (i1 + i0) / 2;
				if (t < this->m_data[mid].m_time) {
					i1 = mid;
				} else {
					i0 = mid;
				}
			}
			dAssert(this->m_data[i0].m_time <= t);
			for (int i = i0 + 1; i < size; i++) {
				if (this->m_data[i].m_time >= t) {
					return i - 1;
				}
			}
			return i0;
		}
	};

	dAnimimationKeyFramesTrack()
		:m_position()
		,m_rotation()
	{
	}

	~dAnimimationKeyFramesTrack()
	{
	}

	void SetName (const dString& name)
	{
		m_name = name; 
	}

	const void InterpolatePosition(dFloat t, dVector &positOut) const;
	const void InterpolateRotation(dFloat t, dQuaternion& rotationOut) const;

	dString m_name;
	dAnimTakeArray<dPositionKey> m_position;
	dAnimTakeArray<dRotationKey> m_rotation;
};

class dAnimationKeyframesSequence: public dRefCounter
{
	public:
	//dAnimationKeyframesSequence(int tracksCount);
	dAnimationKeyframesSequence();
	~dAnimationKeyframesSequence();

	dFloat GetPeriod() const { return m_period; }
	void SetPeriod(dFloat period) { m_period = period;}

	dList<dAnimimationKeyFramesTrack>& GetTracks() { return m_tracks; }
	
	void CalculatePose(dAnimPose& output, dFloat t) const;
	
	dList<dAnimimationKeyFramesTrack> m_tracks;
	dFloat m_period;
};

/*
class dAnimIKBlendNodeTake: public dAnimIKBlendNode
{
	public:
	dAnimIKBlendNodeTake(dAnimIKController* const character, dAnimationKeyframesSequence* const takeData);
	virtual ~dAnimIKBlendNodeTake();

	virtual void Evaluate(dAnimPose& output, dFloat timestep);

	void SetFrame(dFloat t);

	dFloat m_time;
	dAnimationKeyframesSequence* m_takeData;
};
*/

#endif