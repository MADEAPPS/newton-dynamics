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
#include "ndSkyBox.h"
#include "ndUIEntity.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndController_0
{
	#define D_PUSH_ACCEL			ndFloat32 (1.0f)
	#define D_REWARD_MIN_ANGLE		(ndFloat32 (20.0f) * ndDegreeToRad)

	enum ndActionSpace
	{
		m_statePut,
		m_pushLeft,
		m_pushRight,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_poleOmega,
		m_poleAlpha,
		m_cartVelocity,
		m_cartAcceleration,
		m_stateSize
	};

	class ndCartpole: public ndModelArticulation
	{
		public:
		class ndDQNAgent : public ndBrainAgentDQN_Trainner<m_stateSize, m_actionsSize>
		{
			public:
			ndDQNAgent(ndCartpole* const model, ndSharedPtr<ndBrain>& qValuePredictor)
				:ndBrainAgentDQN_Trainner<m_stateSize, m_actionsSize>(qValuePredictor)
				,m_model(model)
			{
			}

			ndReal GetReward() const
			{
				return m_model->GetReward();
			}

			virtual void ApplyActions(ndReal* const actions) const
			{
				m_model->ApplyActions(actions);
			}

			void GetObservation(ndReal* const state) const
			{
				m_model->GetObservation(state);
			}

			bool IsTerminal() const
			{
				return m_model->IsTerminal();
			}

			void ResetModel() const
			{
				m_model->ResetModel();
			}

			void OptimizeStep()
			{
				ndBrainAgentDQN_Trainner::OptimizeStep();
			
				ndInt32 stopTraining = GetFramesCount();
				if (stopTraining > 1000000)
				{
					Save("xxx.nn");
					//ndExpandTraceMessage("%d: episode:%d framesAlive:%d\n", m_frameCount - 1, m_eposideCount, m_framesAlive);
				}
			}

			ndCartpole* m_model;
		};

		ndCartpole()
			:ndModelArticulation()
			,m_cartMatrix(ndGetIdentityMatrix())
			,m_poleMatrix(ndGetIdentityMatrix())
			,m_cart(nullptr)
			,m_pole(nullptr)
			,m_agent(nullptr)
		{
		}

		virtual bool IsTerminal() const
		{
			const ndMatrix& matrix = m_pole->GetMatrix();
			// agent dies if the angle is larger than D_REWARD_MIN_ANGLE * ndFloat32 (2.0f) degrees
			bool fail = ndAbs(matrix.m_front.m_x) > (D_REWARD_MIN_ANGLE * ndFloat32 (2.0f));
			return fail;
		}

		virtual ndReal GetReward() const
		{
			const ndMatrix& matrix = m_pole->GetMatrix();
			ndFloat32 angle = ndMin(ndAbs(matrix.m_front.m_x), D_REWARD_MIN_ANGLE);
			//ndFloat32 angle = ndAbs(matrix.m_front.m_x);
			ndFloat32 reward = ndFloat32(1.0f) - angle / D_REWARD_MIN_ANGLE;
			return ndReal(reward);
		}

		void ApplyActions(ndReal* const actions) const
		{
			ndVector force(m_cart->GetForce());
			ndInt32 action = ndInt32(actions[0]);
			if (action == m_pushLeft)
			{
				force.m_x = -m_cart->GetMassMatrix().m_w * D_PUSH_ACCEL;
			}
			else if (action == m_pushRight)
			{
				force.m_x = m_cart->GetMassMatrix().m_w * D_PUSH_ACCEL;
			}
			m_cart->SetForce(force);
		}

		void GetObservation(ndReal* const state)
		{
			ndSkeletonContainer* const skeleton = m_cart->GetSkeleton();
			ndAssert(skeleton);

			ndWorld* const world = m_cart->GetScene()->GetWorld();
			ndFloat32 timestep = m_cart->GetScene()->GetTimestep();
			m_invDynamicsSolver.SolverBegin(skeleton, nullptr, 0, world, timestep);
			m_invDynamicsSolver.Solve();
			
			const ndVector poleTorque(m_invDynamicsSolver.GetBodyTorque(m_pole));
			ndVector omega(m_pole->GetOmega());
			ndVector alpha(m_pole->GetInvInertiaMatrix().RotateVector(poleTorque));

			const ndVector cartForce(m_invDynamicsSolver.GetBodyForce(m_cart));
			ndVector veloc(m_cart->GetVelocity());
			ndVector accel(cartForce.Scale(m_cart->GetInvMass()));
			
			state[m_poleAlpha] = ndReal(alpha.m_z);
			state[m_poleOmega] = ndReal(omega.m_z);
			state[m_cartVelocity] = ndReal(veloc.m_x);
			state[m_cartAcceleration] = ndReal(accel.m_x);

			m_invDynamicsSolver.SolverEnd();
		}

		virtual void ResetModel() const
		{
			m_pole->SetOmega(ndVector::m_zero);
			m_pole->SetVelocity(ndVector::m_zero);

			m_cart->SetOmega(ndVector::m_zero);
			m_cart->SetVelocity(ndVector::m_zero);

			m_cart->SetMatrix(m_cartMatrix);
			m_pole->SetMatrix(m_poleMatrix);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
			m_agent->Step();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostUpdate(world, timestep);
			m_agent->OptimizeStep();
		
			if (ndAbs(m_cart->GetMatrix().m_posit.m_x) > ndFloat32(40.0f))
			{
				ResetModel();
			}
		}

		ndMatrix m_cartMatrix;
		ndMatrix m_poleMatrix;
		ndBodyDynamic* m_cart;
		ndBodyDynamic* m_pole;
		ndSharedPtr<ndBrainAgent> m_agent;
	};

	void BuildModel(ndCartpole* const model, ndDemoEntityManager* const scene, const ndMatrix& location, ndBodyKinematic* const floorBody)
	{
		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.125f;
		ndFloat32 zSize = 0.15f;
		ndFloat32 cartMass = 5.0f;
		ndFloat32 poleMass = 5.0f;
		ndFloat32 poleLength = 0.4f;
		ndFloat32 poleRadio = 0.05f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// make cart
		ndSharedPtr<ndBody> cartBody(world->GetBody(AddBox(scene, location, cartMass, xSize, ySize, zSize, "smilli.tga")));
		//ndSharedPtr<ndBody> cartBody(world->GetBody(AddSphere(scene, location, cartMass, ySize, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(cartBody);
		cartBody->GetAsBodyDynamic()->SetSleepAccel(cartBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

		ndMatrix matrix(cartBody->GetMatrix());
		matrix.m_posit.m_y += ySize / 2.0f;

		// make pole leg
		ndSharedPtr<ndBody> poleBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), poleMass, poleRadio, poleRadio, poleLength, "smilli.tga")));
		ndMatrix poleLocation(ndRollMatrix(90.0f * ndDegreeToRad) * matrix);
		poleLocation.m_posit.m_y += poleLength * 0.5f;
		poleBody->SetMatrix(poleLocation);
		poleBody->GetAsBodyDynamic()->SetSleepAccel(poleBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));
		
		// link cart and body with a hinge
		ndMatrix polePivot(ndYawMatrix(90.0f * ndDegreeToRad) * poleLocation);
		polePivot.m_posit.m_y -= poleLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
		//ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointFix6dof(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));

		// make the car move alone the z axis only (2d problem)
		ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(cartBody->GetMatrix(), cartBody->GetAsBodyDynamic(), floorBody));
		//ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointFix6dof(cartBody->GetMatrix(), cartBody->GetAsBodyDynamic(), floorBody));
		world->AddJoint(xDirSlider);

		// add path to the model
		world->AddJoint(poleJoint);
		model->AddLimb(modelRoot, poleBody, poleJoint);

		// save some useful data
		model->m_cart = cartBody->GetAsBodyDynamic();
		model->m_pole = poleBody->GetAsBodyDynamic();
		model->m_cartMatrix = cartBody->GetMatrix();
		model->m_poleMatrix = poleBody->GetMatrix();

		// build neutral net controller
		ndSharedPtr<ndBrain> qValuePredictor(new ndBrain());
		ndBrainLayer* const inputLayer = new ndBrainLayer(m_stateSize, 128, m_relu);
		ndBrainLayer* const hiddenLayer0 = new ndBrainLayer(inputLayer->GetOuputSize(), 128, m_relu);
		ndBrainLayer* const hiddenLayer1 = new ndBrainLayer(hiddenLayer0->GetOuputSize(), 128, m_relu);
		ndBrainLayer* const ouputLayer = new ndBrainLayer(hiddenLayer1->GetOuputSize(), m_actionsSize, m_lineal);

		qValuePredictor->BeginAddLayer();
		qValuePredictor->AddLayer(inputLayer);
		qValuePredictor->AddLayer(hiddenLayer0);
		qValuePredictor->AddLayer(hiddenLayer1);
		qValuePredictor->AddLayer(ouputLayer);
		qValuePredictor->EndAddLayer(ndReal(0.25f));

		// add a reinforcement learning controller 
		model->m_agent = ndSharedPtr<ndBrainAgent>(new ndCartpole::ndDQNAgent(model, qValuePredictor));
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, ndBodyKinematic* const floorBody)
	{
		ndCartpole* const model = new ndCartpole();
		BuildModel(model, scene, location, floorBody);
		return model;
	}
}

using namespace ndController_0;
void ndCartpoleController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	ndBodyKinematic* const floorBody = BuildFlatPlane(scene, true);

	ndSetRandSeed(42);
	scene->SetAcceleratedUpdate();

	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	ndSharedPtr<ndModel> model(CreateModel(scene, matrix, floorBody));
	world->AddModel(model);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
