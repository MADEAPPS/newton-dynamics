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
#include "ndCartpolePlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

using namespace ndCarpolePlayer;

namespace ndCartpoleTrainer_sac
{
	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			const ndVector color(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
			scene->Print(color, "training a cart pole using Soft Actor Critic method");
			scene->Print(color, "training goes for 200k steps. Therefore the training");
			scene->Print(color, "seccion takes from one two twp hours with GPU backend");
		}
	};

	class ndAgent : public ndBrainAgentOffPolicyGradient_Agent
	{
		public:
		ndAgent(ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master, ndController* const owner)
			:ndBrainAgentOffPolicyGradient_Agent(*master)
			,m_owner(owner)
		{
		}

		ndBrainFloat CalculateReward()
		{
			return m_owner->CalculateReward();
		}

		bool IsTerminal() const
		{
			return m_owner->IsTerminal();
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			m_owner->GetObservation(observation);
		}

		virtual void ApplyActions(ndBrainFloat* const actions)
		{
			m_owner->ApplyActions(actions);
		}

		void ResetModel()
		{
			m_owner->ResetModel();
		}

		ndController* m_owner;
	};

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_saveScore(m_maxScore)
			,m_discountRewardFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountRewardFactor))
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(500000)
			,m_modelIsTrained(false)
		{
			char name[256];
			snprintf(name, sizeof(name), "%s-vpg.csv", CONTROLLER_NAME_SAC);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "vpg\n");

			// create a Soft Actor Critic traniing agent
			ndBrainAgentOffPolicyGradient_Trainer::HyperParameters hyperParameters;

			//hyperParameters.m_useGpuBackend = false;
			hyperParameters.m_hiddenLayersNumberOfNeurons = 64;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			m_master = ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>(new ndBrainAgentOffPolicyGradient_Trainer(hyperParameters));
			
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME_SAC);
			m_master->SetName(name);

			// create a visual mesh and add to the scene.
			ndWorld* const world = scene->GetWorld();
			ndMatrix matrix(location);
			matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
			matrix.m_posit.m_y += ndFloat32 (0.1f);
			loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;

			ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
			visualMesh->SetTransform(loader.m_mesh->m_matrix);
			visualMesh->SetTransform(loader.m_mesh->m_matrix);

			// create an articulated model
			ndSharedPtr<ndModel>model(CreateModel(scene, loader.m_mesh, visualMesh));

			// add model a visual mesh to the scene and world
			world->AddModel(model);
			scene->AddEntity(visualMesh);
			model->AddBodiesAndJointsToWorld();
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		ndModelArticulation* CreateModel(
			ndDemoEntityManager* const scene, 
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh)
		{
			ndModelArticulation* const model = new ndModelArticulation();
			ndSharedPtr<ndModelNotify> controller(new ndController());
			model->SetNotifyCallback(controller);

			ndController* const playerController = (ndController*)(*controller);
			playerController->CreateArticulatedModel(scene, model, mesh, visualMesh);
			ndSharedPtr<ndBrainAgentOffPolicyGradient_Agent> agent(new ndAgent(m_master, playerController));
			playerController->m_agent = *agent;

			m_master->AddAgent(agent);

			return model;
		}

		void OnDebug(ndDemoEntityManager* const, bool)
		{
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			ndUnsigned32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndUnsigned32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();

				episodeCount -= m_master->GetEposideCount();
				ndFloat32 trajectoryLog = ndLog(m_master->GetAverageFrames() + 0.001f);
				ndFloat32 rewardTrajectory = m_master->GetAverageScore() * trajectoryLog;
				if (rewardTrajectory >= ndFloat32(m_maxScore))
				{
					if (m_lastEpisode != m_master->GetEposideCount())
					{
						m_maxScore = rewardTrajectory;
						m_bestActor->CopyFrom(*m_master->GetPolicyNetwork());
						ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
						m_lastEpisode = m_master->GetEposideCount();
					}
				}

				if (rewardTrajectory > m_saveScore)
				{
					m_saveScore = ndFloor(rewardTrajectory) + 2.0f;

					// save partial controller in case of crash 
					ndBrain* const actor = m_master->GetPolicyNetwork();
					ndString fileName(ndGetWorkingFileName(m_master->GetName().GetStr()));
					m_master->GetPolicyNetwork()->SaveToFile(fileName.GetStr());
					actor->SaveToFile(fileName.GetStr());
				}

				if (episodeCount && !m_master->IsSampling())
				{
					ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
					if (m_outFile)
					{
						fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
						fflush(m_outFile);
					}
				}
			}

			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > ndBrainFloat(0.95f)))
			{
				m_modelIsTrained = true;
				m_master->GetPolicyNetwork()->CopyFrom(*(*m_bestActor));
				ndString fileName (ndGetWorkingFileName(m_master->GetName().GetStr()));
				m_master->GetPolicyNetwork()->SaveToFile(fileName.GetStr());
				ndExpandTraceMessage("saving to file: %s\n", fileName.GetStr());
				ndExpandTraceMessage("training complete\n");
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));

				manager->Terminate();
			}
		}

		ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_saveScore;
		ndFloat32 m_discountRewardFactor;
		ndFloat32 m_horizon;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndCartpoleTrainer_sac;

void ndCartpoleSacTraining(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));

	ndSetRandSeed(42);
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer (new TrainingUpdata(scene, matrix, loader));
	scene->RegisterPostUpdate(trainer);

	// supress v sync refress rate
	scene->SetAcceleratedUpdate();
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

