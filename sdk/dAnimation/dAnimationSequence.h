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

#ifndef __D_ANIMIMATION_TAKE_DATA_h__
#define __D_ANIMIMATION_TAKE_DATA_h__

class dAnimationPose;

class dAnimationSequence: public dRefCounter
{
	public:
	class dAnimationSequenceTrack
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
		class dAnimSequecenceTrackArray : public dArray<OBJECT>
		{
			public:
			dAnimSequecenceTrackArray()
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

		dAnimationSequenceTrack();
		~dAnimationSequenceTrack();

		const void InterpolatePosition(dFloat t, dVector &positOut) const;
		const void InterpolateRotation(dFloat t, dQuaternion& rotationOut) const;

		dAnimSequecenceTrackArray<dPositionKey> m_position;
		dAnimSequecenceTrackArray<dRotationKey> m_rotation;
	};

	dAnimationSequence(int tracksCount);
	~dAnimationSequence();

//	static dAnimationSequence* LoadAnimation(const dScene& scene, const char* const animName);

	dFloat GetPeriod() const { return m_period; }
	void SetPeriod(dFloat period) { m_period = period;}

	dList<dAnimationSequenceTrack>& GetTracks() { return m_tracks; }
	
	void CalculatePose(dAnimationPose& output, dFloat t) const;
	
	dList<dAnimationSequenceTrack> m_tracks;
	dFloat m_period;
};


#endif