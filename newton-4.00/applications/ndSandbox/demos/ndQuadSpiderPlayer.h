/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndQuadSpiderPlayer
{
	#define D_BODY_MASS	ndFloat32(20.0f)
	#define D_LIMB_MASS ndFloat32(0.25f)

	class ndController;

	class ndEffectorInfo
	{
		public:
		ndEffectorInfo()
			:m_calf(nullptr)
			,m_effector(nullptr)
		{
		}

		ndJointHinge* m_calf;
		ndIkSwivelPositionEffector* m_effector;
	};

	class ndProdeduralGaitGenerator : public ndAnimationSequence
	{
		public:
		enum State
		{
			onAir,
			onGround, 
			groundToAir,
			airToGround,
		};

		class ndPose
		{
			public:
			ndVector m_base;
			ndVector m_end;
			ndVector m_start;
			ndVector m_posit;
			ndFloat32 m_time;
			ndFloat32 m_maxTime;
		};

		ndProdeduralGaitGenerator(ndController* const controller);
		void CalculatePose(ndAnimationPose& output, ndFloat32 param) override;

		State GetState() const;
		void IntegrateLeg(ndAnimationPose& output, ndInt32 legIndex);
		ndFloat32 CalculateTime() const;

		ndPose m_pose[4];
		ndFloat32 m_timeLine[3];
		ndController* m_owner;
		ndFloat32 m_omega;
		ndFloat32 m_stride;
		ndFloat32 m_timeAcc;
	};

	class ndGeneratorWalkGait : public ndProdeduralGaitGenerator
	{
		public:
		ndGeneratorWalkGait(ndController* const controller);
	};

	class ndController : public ndModelNotify
	{
		public:

		ndController();

		void Update(ndFloat32 timestep);

		void ResetModel();
		bool IsTerminal() const;

		void CreateArticulatedModel(
			ndDemoEntityManager* const scene,
			ndModelArticulation* const model,
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh);
		void CreateAnimationBlendTree();

		static ndSharedPtr<ndModel> CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader);

		ndAnimationPose m_pose;
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndFloat32 m_timestep;
	};
};

