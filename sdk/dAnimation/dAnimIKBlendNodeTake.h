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

#ifndef __D_ANIM_IK_BLEND_NODE_TAKE_h__
#define __D_ANIM_IK_BLEND_NODE_TAKE_h__
#include "dAnimPose.h"
#include "dAnimIKBlendNode.h"

class dAnimTakeData: public dRefCounter
{
	public:
	template<class OBJECT>
	class dAnimTakeArray
	{
		public:
		dAnimTakeArray()
			:m_count(0)
			,m_data(NULL)
		{
		}

		~dAnimTakeArray()
		{
			if (m_data) {
				delete[] m_data;
			}
		}

		int m_count;
		OBJECT* m_data;
	};

	class dAnimTakeTrack
	{
		public:
		dAnimTakeTrack();
		~dAnimTakeTrack();

		dAnimTakeArray<dVector> m_position;
		dAnimTakeArray<dVector> m_rotation;
	};

	dAnimTakeData(int tracksCount);
	~dAnimTakeData();

	dList<dAnimTakeTrack>& GetTracks() { return m_tracks; }

	dList<dAnimTakeTrack> m_tracks;
};


class dAnimIKBlendNodeTake: public dAnimIKBlendNode
{
	public:
	dAnimIKBlendNodeTake(dAnimIKController* const character, dAnimTakeData* const takeData);
	virtual ~dAnimIKBlendNodeTake();

	virtual void Evaluate(dAnimPose& output, dFloat timestep);

	dAnimTakeData* m_takeData;
};


#endif