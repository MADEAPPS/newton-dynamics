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

#include "dAnimationStdAfx.h"
#include "dAnimIKBlendNodeTake.h"

int dAnimTakeData::dAnimTakeTrack::GetIndex(dFloat t) const
{
	dAssert(t >= 0.0f);
	if (t > m_time[m_time.GetSize() - 1]) {
		t = m_time[m_time.GetSize() - 1];
	}

	int i0 = 0; 
	int i1 = m_time.GetSize() - 1;

	const dFloat* const data = &m_time[0];
	while ((i1 - i0) > 8) {
		const int mid = (i1 + i0) / 2;
		if (t < data[mid]) {
			i1 = mid;
		} else {
			i0 = mid;
		}
	}
	dAssert(data[i0] <= t);
	for (int i = i0 + 1; i < m_time.GetSize(); i++) {
		if (data[i] >= t) {
			return i - 1;
		}
	}
	return i0;
}

dVector dAnimTakeData::dAnimTakeTrack::InterpolatePosition(int base, dFloat t) const
{
	const dFloat t0 = m_time[base];
	const dFloat t1 = m_time[base + 1];
	const dFloat param = (t - t0) / (t1 - t0 + dFloat(1.0e-6f));
	const dVector& p0 = m_position[base];
	const dVector& p1 = m_position[base + 1];
	return p0 + (p1 - p0).Scale(param);
}

dQuaternion dAnimTakeData::dAnimTakeTrack::InterpolateRotation(int base, dFloat t) const
{
	const dFloat t0 = m_time[base];
	const dFloat t1 = m_time[base + 1];
	const dFloat param = (t - t0) / (t1 - t0 + dFloat (1.0e-6f));
	const dQuaternion& rot0 = m_rotation[base];
	const dQuaternion& rot1 = m_rotation[base + 1];
	return dQuaternion(rot0.Slerp(rot1, param));
}

dAnimTakeData::dAnimTakeData(int tracksCount)
	:dRefCounter()
	,m_tracks()
	,m_period(1.0f)
{
	for (int i = 0; i < tracksCount; i ++) {
		m_tracks.Append();
	}
}

dAnimTakeData::~dAnimTakeData()
{
}

dAnimTakeData::dAnimTakeTrack::dAnimTakeTrack()
	:m_position()
	,m_rotation()
{
}

dAnimTakeData::dAnimTakeTrack::~dAnimTakeTrack()
{
}

void dAnimTakeData::CalculatePose(dAnimPose& output, dFloat t) const
{
	dAnimPose::dListNode* destNode = output.GetFirst();
	for (dList<dAnimTakeTrack>::dListNode* srcNode = m_tracks.GetFirst(); srcNode; srcNode = srcNode->GetNext()) {
		const dAnimTakeTrack& track = srcNode->GetInfo();
		if (track.m_rotation.GetSize() && track.m_position.GetSize()) {
			dAssert(track.m_time.GetSize() > 1);
			dAnimKeyframe& keyFrameOut = destNode->GetInfo();
			int index = track.GetIndex(t);
			keyFrameOut.m_posit = track.InterpolatePosition(index, t);
			keyFrameOut.m_rotation = track.InterpolateRotation(index, t);
		} else if (track.m_rotation.GetSize()) {
			dAnimKeyframe& keyFrameOut = destNode->GetInfo();
			dAssert(track.m_time.GetSize() > 1);
			int index = track.GetIndex(t);
			keyFrameOut.m_rotation = track.InterpolateRotation(index, t);
		} else if (track.m_position.GetSize()) {
			dAssert(0);
		}

		destNode = destNode->GetNext();
	}
}

dAnimIKBlendNodeTake::dAnimIKBlendNodeTake(dAnimIKController* const character, dAnimTakeData* const takeData)
	:dAnimIKBlendNode(character, NULL)
	,m_takeData(takeData)
	,m_time(0.0f)
{
	m_takeData->AddRef();
}

dAnimIKBlendNodeTake::~dAnimIKBlendNodeTake()
{
	m_takeData->Release();
}

void dAnimIKBlendNodeTake::SetFrame(dFloat t)
{
	m_time = dMod(m_time + t, m_takeData->m_period);
m_time = m_takeData->m_period * 0.0f;
}

void dAnimIKBlendNodeTake::Evaluate(dAnimPose& output, dFloat timestep)
{
	SetFrame(timestep);
	m_takeData->CalculatePose(output, m_time);
}


