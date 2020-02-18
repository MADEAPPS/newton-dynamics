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
#include "dAnimationPose.h"
#include "dAnimationSequence.h"

dAnimationSequence::dAnimationSequence()
	:m_tracks()
	,m_period(1.0f)
{
}

dAnimationSequence::~dAnimationSequence()
{
}

dAnimimationKeyFramesTrack* dAnimationSequence::AddTrack()
{
	dList<dAnimimationKeyFramesTrack>::dListNode* const node = m_tracks.Append();
	return &node->GetInfo();
}

void dAnimationSequence::CalculatePose(dAnimationPose& output, dFloat t) const
{
	dAnimationPose::dListNode* destNode = output.GetFirst()->GetNext();
	for (dList<dAnimimationKeyFramesTrack>::dListNode* srcNode = m_tracks.GetFirst()->GetNext(); srcNode; srcNode = srcNode->GetNext()) {
		const dAnimimationKeyFramesTrack& track = srcNode->GetInfo();
		dAnimKeyframe& keyFrame = destNode->GetInfo();
		track.InterpolatePosition(t, keyFrame.m_posit);
		track.InterpolateRotation(t, keyFrame.m_rotation);
		dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation) > 0.999f);
		dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation) < 1.001f);
		destNode = destNode->GetNext();
	}
}

#if 0
//dAnimationSequence* LoadAnimation(dAnimIKController* const controller, const char* const animName)
dAnimTakeData* dAnimTakeData::LoadAnimation(const dScene& scene, const char* const animName)
{
//	dTree<dAnimTakeData*, dString>::dTreeNode* cachedAnimNode = m_animCache.Find(animName);
//	if (!cachedAnimNode) {

//		dScene scene(world);
//		char pathName[2048];
//		dGetWorkingFileName(animName, pathName);
//		scene.Deserialize(pathName);
/*
		dScene::dTreeNode* const animTakeNode = scene.FindChildByType(scene.GetRootNode(), dAnimationTake::GetRttiType());
		if (animTakeNode) {
			dTree<dAnimTakeData::dAnimTakeTrack*, dString> map;
			const dAnimPose& basePose = controller->GetBasePose();

			dAnimTakeData* const animdata = new dAnimTakeData(basePose.GetCount());
			dAnimationTake* const animTake = (dAnimationTake*)scene.GetInfoFromNode(animTakeNode);
			animdata->SetPeriod(animTake->GetPeriod());

			cachedAnimNode = m_animCache.Insert(animdata, animName);

			dList<dAnimTakeData::dAnimTakeTrack>& tracks = animdata->GetTracks();
			dList<dAnimTakeData::dAnimTakeTrack>::dListNode* ptr = tracks.GetFirst();
			for (dAnimPose::dListNode* ptrNode = basePose.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
				DemoEntity* const entity = (DemoEntity*)ptrNode->GetInfo().m_userData;
				map.Insert(&ptr->GetInfo(), entity->GetName());
				ptr = ptr->GetNext();
			}

			for (void* link = scene.GetFirstChildLink(animTakeNode); link; link = scene.GetNextChildLink(animTakeNode, link)) {
				dScene::dTreeNode* const node = scene.GetNodeFromLink(link);
				dAnimationTrack* const srcTrack = (dAnimationTrack*)scene.GetInfoFromNode(node);
				if (srcTrack->IsType(dAnimationTrack::GetRttiType())) {

					dTree<dAnimTakeData::dAnimTakeTrack*, dString>::dTreeNode* const ptrNode = map.Find(srcTrack->GetName());
					dAssert(ptrNode);
					dAnimTakeData::dAnimTakeTrack* const dstTrack = ptrNode->GetInfo();

					const dList<dAnimationTrack::dCurveValue>& rotations = srcTrack->GetRotations();
					dstTrack->m_rotation.Resize(rotations.GetCount());
					int index = 0;
					for (dList<dAnimationTrack::dCurveValue>::dListNode* node = rotations.GetFirst(); node; node = node->GetNext()) {
						dAnimationTrack::dCurveValue keyFrame(node->GetInfo());

						dMatrix matrix(dPitchMatrix(keyFrame.m_x) * dYawMatrix(keyFrame.m_y) * dRollMatrix(keyFrame.m_z));
						dQuaternion rot(matrix);
						dstTrack->m_rotation[index].m_rotation = rot;
						dstTrack->m_rotation[index].m_time = keyFrame.m_time;
						index++;
					}

					for (int i = 0; i < rotations.GetCount() - 1; i++) {
						dFloat dot = dstTrack->m_rotation[i].m_rotation.DotProduct(dstTrack->m_rotation[i + 1].m_rotation);
						if (dot < 0.0f) {
							dstTrack->m_rotation[i + 1].m_rotation.m_x *= -1.0f;
							dstTrack->m_rotation[i + 1].m_rotation.m_y *= -1.0f;
							dstTrack->m_rotation[i + 1].m_rotation.m_z *= -1.0f;
							dstTrack->m_rotation[i + 1].m_rotation.m_w *= -1.0f;
						}
					}

					const dList<dAnimationTrack::dCurveValue>& positions = srcTrack->GetPositions();
					dstTrack->m_position.Resize(positions.GetCount());
					index = 0;
					for (dList<dAnimationTrack::dCurveValue>::dListNode* node = positions.GetFirst(); node; node = node->GetNext()) {
						dAnimationTrack::dCurveValue keyFrame(node->GetInfo());
						dstTrack->m_position[index].m_posit = dVector(keyFrame.m_x, keyFrame.m_y, keyFrame.m_z, dFloat(1.0f));
						dstTrack->m_position[index].m_time = keyFrame.m_time;
						index++;
					}
				}
			}
		}

	}
*/

//	dAssert(cachedAnimNode);
//	return cachedAnimNode->GetInfo();
	return NULL;
}
#endif

void dAnimationSequence::Save(const char* const fileName)
{

}