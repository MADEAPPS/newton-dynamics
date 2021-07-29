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

#include "ndSandboxStdafx.h"
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"

ndAnimationSequence::ndAnimationSequence()
	:m_tracks()
	,m_period(1.0f)
{
}

ndAnimationSequence::~ndAnimationSequence()
{
}

ndAnimationKeyFramesTrack* ndAnimationSequence::AddTrack()
{
	dList<ndAnimationKeyFramesTrack>::dNode* const node = m_tracks.Append();
	return &node->GetInfo();
}

void ndAnimationSequence::CalculatePose(ndAnimationPose& output, dFloat32 t) const
{
	int index = 0;
	ndAnimKeyframe* const keyFrames = &output[0];
	for (dList<ndAnimationKeyFramesTrack>::dNode* srcNode = m_tracks.GetFirst(); srcNode; srcNode = srcNode->GetNext()) 
	{
		const ndAnimationKeyFramesTrack& track = srcNode->GetInfo();
		ndAnimKeyframe& keyFrame = keyFrames[index];
		track.InterpolatePosition(t, keyFrame.m_posit);
		track.InterpolateRotation(t, keyFrame.m_rotation);
		dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() > 0.999f);
		dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() < 1.001f);

		index ++;
	}
}

//void ndAnimationSequence::Load(const char* const fileName)
void ndAnimationSequence::Load(const char* const)
{
	dAssert(0);
	//char* const oldloc = setlocale(LC_ALL, 0);
	//setlocale(LC_ALL, "C");
	//
	//m_tracks.RemoveAll();
	//m_period = 0;
	//FILE* const file = fopen(fileName, "rb");
	//if (file) {
	//	int tracksCount;
	//	fscanf(file, "period %f\n", &m_period);
	//	fscanf(file, "tracksCount %d\n", &tracksCount);
	//	for (int i = 0; i < tracksCount; i++) {
	//		ndAnimationKeyFramesTrack* const track = AddTrack();
	//		track->Load(file);
	//	}
	//	fclose(file);
	//}
	//
	//// restore locale settings
	//setlocale(LC_ALL, oldloc);
}

//void ndAnimationSequence::Save(const char* const fileName)
void ndAnimationSequence::Save(const char* const)
{
	dAssert(0);
	//char* const oldloc = setlocale(LC_ALL, 0);
	//setlocale(LC_ALL, "C");
	//
	//FILE* const file = fopen(fileName, "wb");
	//if (file) {
	//	fprintf(file, "period %f\n", m_period);
	//	fprintf(file, "tracksCount %d\n", m_tracks.GetCount());
	//	for (dList<ndAnimationKeyFramesTrack>::dNode* trackNode = m_tracks.GetFirst(); trackNode; trackNode = trackNode->GetNext()) {
	//		ndAnimationKeyFramesTrack& track = trackNode->GetInfo();
	//		track.Save(file);
	//	}
	//	fclose(file);
	//}
	//
	//// restore locale settings
	//setlocale(LC_ALL, oldloc);
}
