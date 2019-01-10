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


dVector dAnimTakeData::dAnimTakeTrack::InterpolatePosition(dFloat t) const
{
	int base = m_position.GetIndex(t);
	const dFloat t0 = m_position[base].m_time;
	const dFloat t1 = m_position[base + 1].m_time;
	const dFloat param = (t - t0) / (t1 - t0 + dFloat(1.0e-6f));
	const dVector& p0 = m_position[base].m_posit;
	const dVector& p1 = m_position[base + 1].m_posit;
	return p0 + (p1 - p0).Scale(param);
}

dQuaternion dAnimTakeData::dAnimTakeTrack::InterpolateRotation(dFloat t) const
{
	int base = m_rotation.GetIndex(t);
	const dFloat t0 = m_rotation[base].m_time;
	const dFloat t1 = m_rotation[base + 1].m_time;
	const dFloat param = (t - t0) / (t1 - t0 + dFloat (1.0e-6f));
	const dQuaternion& rot0 = m_rotation[base].m_rotation;
	const dQuaternion& rot1 = m_rotation[base + 1].m_rotation;
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
	dAnimPose::dListNode* destNode = output.GetFirst()->GetNext();
	for (dList<dAnimTakeTrack>::dListNode* srcNode = m_tracks.GetFirst()->GetNext(); srcNode; srcNode = srcNode->GetNext()) {
		const dAnimTakeTrack& track = srcNode->GetInfo();
		dAnimKeyframe& keyPositionOut = destNode->GetInfo();
		dAnimKeyframe& keyRotationOut = destNode->GetInfo();
		keyPositionOut.m_posit = track.InterpolatePosition(t);
		keyRotationOut.m_rotation = track.InterpolateRotation(t);
		dAssert(keyRotationOut.m_rotation.DotProduct(keyRotationOut.m_rotation) > 0.999f);
		dAssert(keyRotationOut.m_rotation.DotProduct(keyRotationOut.m_rotation) < 1.001f);
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
//m_time = m_takeData->m_period * 0.0f;
}

void dAnimIKBlendNodeTake::Evaluate(dAnimPose& output, dFloat timestep)
{
	SetFrame(timestep);
	m_takeData->CalculatePose(output, m_time);
}


