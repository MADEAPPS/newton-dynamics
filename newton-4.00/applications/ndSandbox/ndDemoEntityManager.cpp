/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndFileBrowser.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndTestDeepBrain.h"
#include "ndDemoCameraNode.h"
#include "ndMenuRenderPass.h"
#include "ndDemoEntityManager.h"
#include "ndHighResolutionTimer.h"
#include "ndDemoCameraNodeFlyby.h"
#include "ndDebugDisplayRenderPass.h"

#define MACHINE_LEARNING_BASE	100

//#define DEFAULT_SCENE	0		// basic rigidbody
//#define DEFAULT_SCENE	1		// basic Stacks 
//#define DEFAULT_SCENE	2		// basic friction
//#define DEFAULT_SCENE	3		// basic sliding platform
//#define DEFAULT_SCENE	4		// basic Trigger
//#define DEFAULT_SCENE	5		// conservation of momentum 
//#define DEFAULT_SCENE	6		// basic joints
//#define DEFAULT_SCENE	7		// static mesh collision
//#define DEFAULT_SCENE	8		// heighfield collision
//#define DEFAULT_SCENE	9		// static compound scene collision
#define DEFAULT_SCENE	10		// basic convex approximate compound shapes
//#define DEFAULT_SCENE	11		// basic model, a npd vehicle prop
//#define DEFAULT_SCENE	12		// basic ragdoll
//#define DEFAULT_SCENE	13		// complex model, implement a complex model with joints
//#define DEFAULT_SCENE	14		// basics mutibody vehicle
//#define DEFAULT_SCENE	15		// object Placement
//#define DEFAULT_SCENE	16		// third person player capsule
//#define DEFAULT_SCENE	17		// cart pole SAC trained controller
//#define DEFAULT_SCENE	18		// cart pole PPO trained controller

// These are the machine learning training demos
//#define DEFAULT_SCENE			(MACHINE_LEARNING_BASE + 0)	// train cart pole using SAC agent
//#define DEFAULT_SCENE			(MACHINE_LEARNING_BASE + 1)	// train cart pole using PPO agent


// legacy demos 
//#define DEFAULT_SCENE	8		// particle fluid
//#define DEFAULT_SCENE	12		// basic vehicle
//#define DEFAULT_SCENE	13		// heavy vehicle
//#define DEFAULT_SCENE	19		// unit cycle controller
//#define DEFAULT_SCENE	20		// quadruped animated 1
//#define DEFAULT_SCENE	21		// quadruped sac trained
//#define DEFAULT_SCENE	22		// quadruped ppo trained
//#define DEFAULT_SCENE	23		// quadruped test 3
//#define DEFAULT_SCENE	24		// quadruped test 4
//#define DEFAULT_SCENE	25		// simple industrial robot
//#define DEFAULT_SCENE	26		// advanced industrial robot
//#define DEFAULT_SCENE	27		// biped test 1
//#define DEFAULT_SCENE	28		// biped test 2
//#define DEFAULT_SCENE	29		// train biped test 2
//#define DEFAULT_SCENE	31		// simple fracture
//#define DEFAULT_SCENE	32		// basic fracture
//#define DEFAULT_SCENE	33		// linked fracture
//#define DEFAULT_SCENE	34		// skin peel fracture
						 
void ndBasicModel(ndDemoEntityManager* const scene);
void ndBasicJoints(ndDemoEntityManager* const scene);
void ndBasicStacks(ndDemoEntityManager* const scene);
void ndComplexModel(ndDemoEntityManager* const scene);
void ndBasicTrigger(ndDemoEntityManager* const scene);
void ndBasicRagdoll(ndDemoEntityManager* const scene);
void ndBasicVehicle(ndDemoEntityManager* const scene);
void ndBasicFriction(ndDemoEntityManager* const scene);
void ndBasicRigidBody(ndDemoEntityManager* const scene);
void ndObjectPlacement(ndDemoEntityManager* const scene);
void ndCartpolePlayer_SAC(ndDemoEntityManager* const scene);
void ndCartpolePlayer_PPO(ndDemoEntityManager* const scene);
void ndBasicAngularMomentum(ndDemoEntityManager* const scene);
void ndBasicSlidingPlatform(ndDemoEntityManager* const scene);
void ndBasicCompoundCollision(ndDemoEntityManager* const scene);
void ndBasicHeighfieldCollision(ndDemoEntityManager* const scene);
void ndBasicStaticMeshCollision(ndDemoEntityManager* const scene);
void ndPlayerCapsule_ThirdPerson(ndDemoEntityManager* const scene);
void ndBasicSceneCompoundCollision(ndDemoEntityManager* const scene);

void ndCartpoleSacTraining(ndDemoEntityManager* const scene);
void ndCartpolePpoTraining(ndDemoEntityManager* const scene);

ndDemoEntityManager::ndDemos ndDemoEntityManager::m_demosSelection[] =
{
	{ "basic rigidbody", ndBasicRigidBody},
	{ "basic stacking", ndBasicStacks},
	{ "basic friction", ndBasicFriction},
	{ "basic sliding ground", ndBasicSlidingPlatform},
	{ "basic trigger", ndBasicTrigger},
	{ "basic momentum conservation", ndBasicAngularMomentum},
	{ "basic joints", ndBasicJoints},
	{ "basic static mesh collision", ndBasicStaticMeshCollision},
	{ "basic heighfield collision", ndBasicHeighfieldCollision},
	{ "basic static compound scene collision", ndBasicSceneCompoundCollision},
	{ "basic compound collision", ndBasicCompoundCollision},
	{ "basic model", ndBasicModel},
	{ "basic ragdoll", ndBasicRagdoll},
	{ "complex model", ndComplexModel},
	{ "basic vehicle", ndBasicVehicle},
	{ "object placement", ndObjectPlacement},
	{ "basic player", ndPlayerCapsule_ThirdPerson},
	{ "cart pole SAC player controller", ndCartpolePlayer_SAC},
	{ "cart pole PPO player controller", ndCartpolePlayer_PPO},
	

#if 0
	{ "basic object placement", ndObjectPlacement},
	{ "basic particle fluid", ndBasicParticleFluid},
	{ "basic vehicle", ndBasicVehicle},
	{ "heavy vehicle", ndHeavyVehicle},

	{ "cartpole discrete controller", ndCartpoleDiscrete},
	{ "unicycle controller", ndUnicycleController},
	{ "quadruped animated", ndQuadruped_animation_test},
	{ "quadruped sac", ndQuadruped_sac_test},
	{ "quadruped ppo", ndQuadruped_ppo_test},
	//{ "quadruped test 3", ndQuadrupedTest_3},
	//{ "quadruped test 4", ndQuadrupedTest_4},
	//{ "quadruped test 5", ndQuadrupedTest_4},
	{ "simple industrial robot", ndSimpleIndustrialRobot},
	{ "advanced industrial robot", ndAdvancedIndustrialRobot},
	//{ "biped test one", ndBipedTest_1},
	//{ "biped test two", ndBipedTest_2},
	//{ "train biped test two", ndBipedTest_2Trainer},
	//{ "simple convex fracture", ndBasicExplodeConvexShape},
	//{ "basic convex fracture", ndBasicFracture_0},
	//{ "linked convex fracture", ndBasicFracture_2},
	//{ "simple skin peeling fracture", ndBasicFracture_4},
#endif
};

ndDemoEntityManager::ndDemos ndDemoEntityManager::m_machineLearning[]
{
	{ "cartpole SAC training", ndCartpoleSacTraining},
	{ "Cartpole PPO Training", ndCartpolePpoTraining},
};

ndDemoEntityManager::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

ndInt32 ndDemoEntityManager::ButtonKey::UpdateTrigger (bool triggerValue)
{
	m_memory0 = m_memory1;
	m_memory1 = triggerValue;
	return (!m_memory0 && m_memory1) ? 1 : 0;
}

ndInt32 ndDemoEntityManager::ButtonKey::UpdatePushButton (bool triggerValue)
{
	if (UpdateTrigger (triggerValue)) 
	{
		m_state = ! m_state;
	}
	return m_state ? 1 : 0;
}

static void Test0__()
{
	ndFixSizeArray<ndFloat32, 6> B(6);
	ndFixSizeArray<ndFloat32, 6> x0(6);
	ndFixSizeArray<ndFloat32, 6> x1(6);
	ndFixSizeArray<ndFixSizeArray<ndFloat32, 6>, 6> A(6);
	for (ndInt32 i = 0; i < A.GetCount(); ++i)
	{
		A[i].SetCount(A.GetCount());
		ndMemSet(&A[i][0], ndFloat32(0.0f), A.GetCount());
	}

	ndFloat32 data[] = { 1.0f, -2.0f, 1.0f, 2.5f, 3.0f, -1.0f };
	ndInt32 stride = ndInt32 (&A[1][0] - &A[0][0]);
	//ndCovarianceMatrix<ndFloat32>(6, stride, &A[0][0], data, data);
	//for (ndInt32 i = 0; i < A.GetCount(); ++i)
	//{
	//	A[i][i] *= 1.001f;
	//}
	ndFloat32 t = ndFloat32(0.99f);
	A[0][0] = t;
	for (ndInt32 i = 1; i < A.GetCount(); ++i)
	{
		A[i][i] = ndFloat32(1.0f) + t;
	}
	ndFloat32 ot = ndSqrt(t);
	for (ndInt32 i = 0; i < A.GetCount() - 1; ++i)
	{
		A[i][i + 1] = ot;
		A[i + 1][i] = ot;
	}

	for (ndInt32 i = 0; i < A.GetCount(); ++i)
	{
		x0[i] = ndFloat32(i) + 1.0f;
		x1[i] = ndFloat32(i) - 1.0f;
	}
	ndAssert(ndTestPSDmatrix(6, stride, &A[0][0]));

	ndMatrixTimeVector<ndFloat32>(A.GetCount(), stride, &A[0][0], &x0[0], &B[0]);

	ndConjugateGradient<ndFloat32> cgd(true);
	cgd.Solve(A.GetCount(), stride, ndFloat32(1.0e-5f), &x1[0], &B[0], &A[0][0]);

	B[0] = 1.0f;
	B[1] = 1.0f;
	x1[0] = 0.0f;
	x1[1] = 0.0f;
	A[0][0] = 4.0f;
	A[0][1] = 1.0f;
	A[1][0] = 1.0f;
	A[1][1] = 9.0f;
	cgd.Solve(2, stride, ndFloat32(1.0e-5f), &x1[0], &B[0], &A[0][0]);
}

static ndInt32 Fibonacci(ndInt32 n)
{
	ndInt32 a = 0;
	ndInt32 b = 1;
	
	if (n == 0)
	{
		return a;
	}
	for (ndInt32 i = 2; i <= n; i++)
	{
		ndInt32 c = a + b;
		a = b;
		b = c;
	}
	return b;
}

static void Test1__()
{
	//ndFloat32 A[2][2];
	//ndFloat32 x[2];
	//ndFloat32 b[2];
	//ndFloat32 l[2];
	//ndFloat32 h[2];
	//
	//A[0][0] = 2.0f;
	//A[0][1] = 1.0f;
	//A[1][0] = 1.0f;
	//A[1][1] = 2.0f;
	//b[0] = 1.0f;
	//b[1] = 1.0f;
	//x[0] = 1;
	//x[1] = 2;
	//
	//l[0] = 0.0f;
	//l[1] = 0.0f;
	//h[0] = 0.25f;
	//h[1] = 1.0f;
	//
	//ndMatrixTimeVector(2, &A[0][0], x, b);
	//dSolveDantzigLCP(2, &A[0][0], x, b, l, h);
	//
	//ndInt32 xxx = 0;
	//const ndInt32 xxxxxx = 450;
	//dDownHeap<ndInt32, unsigned> xxxxx (xxxxxx + 2);
	//for (ndInt32 i = 0; i < xxxxxx; ++i)
	//{
	//	xxxxx.Push (xxx, i);
	//}
	//
	//for (ndInt32 i = 0; i < 10000; ++i)
	//{
	//	ndInt32 index = dRandInt() % xxxxxx;
	//	ndInt32 key = xxxxx.Value(index);
	//	xxxxx.Remove (index);
	//	xxxxx.Push (xxx, key);
	//}

	//ndBrainVector xxx;
	//ndBrainVector xxx1;
	//xxx.PushBack(0.1f);
	//xxx.PushBack(0.7f);
	//xxx.PushBack(0.2f);
	//
	//xxx1.SetCount(3);
	//for (ndInt32 i = 0; i < 20; i++)
	//{
	//	xxx1.CategoricalSample(xxx, 0.5f);
	//}
}

static void SimpleRegressionBrainStressTest()
{
	ndInt32 nunberOfSamples = 1024 * 32;

	ndBrainVector data;
	ndBrainVector truth;
	ndArray<ndUnsigned32> shuffleBuffer;
	ndBrainFloat sinWavePeriod = 100.0f;

	data.SetCount(nunberOfSamples);
	truth.SetCount(nunberOfSamples);
	shuffleBuffer.SetCount(nunberOfSamples);
	for (ndInt32 i = 0; i < nunberOfSamples; ++i)
	{
		ndBrainFloat arg = ndBrainFloat (2.0f * ndPi * ndBrainFloat(i) / sinWavePeriod);
		ndBrainFloat a = ndBrainFloat(ndSin(arg));
		truth[i] = a;
		data[i] = arg;
		shuffleBuffer[i] = ndUnsigned32(i);
	}

	ndSetRandSeed(42);
	//ndInt32 inputSize = 1;
	ndInt32 minibatchSize = 32;
	ndInt32 numberOfEpochs = 10;
	ndSharedPtr<ndBrain> brain(new ndBrain);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

#if 0
	ndInt32 hidenLayerWidth = 32;
	layers.PushBack(new ndBrainLayerLinear(inputSize, hidenLayerWidth));
	layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hidenLayerWidth));
	layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
#else
	layers.PushBack(new ndBrainLayerLinear(1, 1));
#endif

	ndSharedPtr<ndBrainContext> context(new ndBrainGpuContext);
	ndTrainerDescriptor descriptor;
	descriptor.m_brain = brain;
	descriptor.m_context = context;
	descriptor.m_minibatchSize = minibatchSize;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		brain->AddLayer(layers[i]);
	}
	brain->InitWeights();

	ndBrainOptimizerAdam::ndCommandSharedInfo optimizeInfo;
	ndSharedPtr<ndBrainTrainer> trainer(new ndBrainTrainer(descriptor, ndSharedPtr<ndBrainOptimizer>(new ndBrainOptimizerAdam(context))));
	ndBrainFloatBuffer* const minibatchInputBuffer = trainer->GetInputBuffer();
	ndBrainFloatBuffer* const minibatchOutpuBuffer = trainer->GetOuputBuffer();
	ndBrainFloatBuffer* const minibatchInputGradientBuffer = trainer->GetInputGradientBuffer();
	ndBrainFloatBuffer* const minibatchOutpuGradientBuffer = trainer->GetOuputGradientBuffer();
	
	ndBrainFloatBuffer trainingData(*context, data);
	ndBrainIntegerBuffer indirectMiniBatch(*context, minibatchSize);
	
	ndInt32 batchesCount = nunberOfSamples / minibatchSize;
	ndInt32 batchesSize = batchesCount * minibatchSize;
	size_t strideInBytes = size_t(1 * sizeof(ndReal));
	
	ndCopyBufferCommandInfo copyBufferInfo;
	copyBufferInfo.m_dstOffsetInByte = 0;
	copyBufferInfo.m_srcOffsetInByte = 0;
	copyBufferInfo.m_strideInByte = ndInt32(strideInBytes);
	copyBufferInfo.m_srcStrideInByte = ndInt32(strideInBytes);
	copyBufferInfo.m_dstStrideInByte = ndInt32(strideInBytes);
	
	ndBrainVector miniBatchOutput;
	ndBrainVector miniBatchOutputGradients;
	ndBrainLossLeastSquaredError loss(1);
	ndBrainVector miniBatchInputGradients;
	
	miniBatchOutputGradients.SetCount(minibatchSize);
	
	ndUnsigned64 time = ndGetTimeInMicroseconds();
	for (ndInt32 epoch = 0; epoch < numberOfEpochs; ++epoch)
	{
		shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
		for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += minibatchSize)
		{
			indirectMiniBatch.MemoryToDevice(0, minibatchSize * sizeof(ndUnsigned32), &shuffleBuffer[batchStart]);
			minibatchInputBuffer->CopyBufferIndirect(copyBufferInfo, indirectMiniBatch, trainingData);
			trainer->MakePrediction();
	
			context->SyncBufferCommandQueue();
			minibatchOutpuBuffer->VectorFromDevice(miniBatchOutput);
	
			for (ndInt32 i = 0; i < minibatchSize; ++i)
			{
				ndUnsigned32 index = shuffleBuffer[batchStart + i];
				ndBrainMemVector grad(&miniBatchOutputGradients[i], 1);
				const ndBrainMemVector output(&miniBatchOutput[i], 1);
				const ndBrainMemVector groundTruth(&truth[index], 1);
				loss.SetTruth(groundTruth);
				loss.GetLoss(output, grad);
			}
			minibatchOutpuGradientBuffer->VectorToDevice(miniBatchOutputGradients);
			trainer->BackPropagate();
			trainer->ApplyLearnRate(ndBrainFloat(1.0e-4f));
			context->SyncBufferCommandQueue();
	
			minibatchInputGradientBuffer->VectorFromDevice(miniBatchInputGradients);
			epoch *= 1;
		}
	}
	time = ndGetTimeInMicroseconds() - time;
	
	ndExpandTraceMessage("stress test Regression\n");
	ndExpandTraceMessage(" training time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
}

// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
ndDemoEntityManager::ndDemoEntityManager()
	:ndClassAlloc()
	,m_world(nullptr)
	,m_renderer(nullptr)
	,m_menuRenderPass(nullptr)
	,m_colorRenderPass(nullptr)
	,m_shadowRenderPass(nullptr)
	,m_environmentRenderPass(nullptr)
	,m_transparentRenderPass(nullptr)
	,m_debugDisplayRenderPass(nullptr)
	,m_environmentTexture(nullptr)
	,m_demoHelper(nullptr)
	,m_currentScene(DEFAULT_SCENE)
	,m_lastCurrentScene(DEFAULT_SCENE)
	,m_framesCount(0)
	,m_physicsFramesCount(0)
	,m_currentPlugin(0)
	,m_solverPasses(6)
	,m_solverSubSteps(2)
	,m_workerThreads(4)
	,m_debugDisplayMode(0)
	,m_collisionDisplayMode(0)
	,m_fps(0.0f)
	,m_timestepAcc(0.0f)
	,m_currentListenerTimestep(0.0f)
	,m_addDeleteLock()
	,m_showUI(true)
	,m_showAABB(false)
	,m_showStats(true)
	,m_helperLegend(false)
	,m_autoSleepMode(true)
	,m_showScene(false)
	,m_showConcaveEdge(false)
	,m_hideVisualMeshes(false)
	,m_showNormalForces(false)
	,m_showCenterOfMass(false)
	,m_showBodyFrame(false)
	,m_showMeshSkeleton(false)
	,m_updateMenuOptions(true)
	,m_showContactPoints(false)
	,m_showJointDebugInfo(false)
	,m_showModelsDebugInfo(false)
	,m_showCollidingFaces(false)
	,m_hidePostUpdate(false)
	,m_suspendPhysicsUpdate(false)
	,m_synchronousPhysicsUpdate(false)
	,m_synchronousParticlesUpdate(false)
	,m_showRaycastHit(false)
	,m_profilerMode(false)
	,m_nextActiveCamera()
	,m_solverMode(ndWorld::ndSimdSoaSolver)
{
	// Setup window
	char title[256];

	ndSharedPtr<ndRender::ndUserCallback> callbacks(new ndRenderCallback(this));
	snprintf(title, sizeof(title), "Newton Dynamics %d.%.2i sandbox demos", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_renderer = ndSharedPtr<ndRender>(new ndRender(callbacks, 1280, 768, title));

	//char fontPathName[2048];
	//char* const name = "calibri.ttf";
	//char* const name = "courbd.ttf";
	//const char* const name = "Cousine-Regular.ttf";
	const ndString fontPathName(ndGetWorkingFileName("Cousine-Regular.ttf"));
	m_renderer->InitImGui(fontPathName.GetStr());

	// load the environment texture
	ndFixSizeArray<ndString, 6> environmentTexturePath;
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negx.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posx.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posy.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negy.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negz.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posz.png"));
	m_environmentTexture = m_renderer->GetTextureCache()->GetCubeMap(environmentTexturePath);

	// create render passes
	m_menuRenderPass = ndSharedPtr<ndRenderPass>(new ndMenuRenderPass(this));
	m_debugDisplayRenderPass = ndSharedPtr<ndRenderPass>(new ndDebugDisplayRenderPass(this));
	m_colorRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassColor(*m_renderer));
	m_shadowRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassShadows(*m_renderer));
	m_transparentRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassTransparency(*m_renderer));
	m_environmentRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassEnvironment(*m_renderer, m_environmentTexture));

	// add render passes in order of execution
	m_renderer->AddRenderPass(m_shadowRenderPass);
	m_renderer->AddRenderPass(m_colorRenderPass);
	m_renderer->AddRenderPass(m_environmentRenderPass);
	m_renderer->AddRenderPass(m_transparentRenderPass);
	m_renderer->AddRenderPass(m_debugDisplayRenderPass);
	m_renderer->AddRenderPass(m_menuRenderPass);

	//add main directional light
	m_renderer->SetSunLight(ndVector(-0.5f, 1.0f, -0.5f, 0.0f), ndVector(0.7f, 0.7f, 0.7f, 0.0f));

	// initialized the physics world for the new scene
	//m_showUI = false;
	//m_showAABB = true;
	//m_showScene = true;
	//m_showConcaveEdge = true;
	//m_showMeshSkeleton = true;
	//m_autoSleepMode = false;
	///m_hidePostUpdate = true;
	//m_hideVisualMeshes = true;
	//m_solverMode = ndWorld::ndStandardSolver;
	//m_solverMode = ndWorld::ndSimdSoaSolver;
	//m_solverMode = ndWorld::ndSimdAvx2Solver;
	//m_solverPasses = 4;
	m_workerThreads = 1;
	//m_solverSubSteps = 2;
	//m_showRaycastHit = true;
	//m_showCenterOfMass = false;
	//m_showNormalForces = true;
	//m_showContactPoints = true;
	//m_showJointDebugInfo = true;
	//m_showModelsDebugInfo = true;
	//m_collisionDisplayMode = 1;
	//m_collisionDisplayMode = 2;	
	//m_collisionDisplayMode = 3;		// solid wire frame
	m_synchronousPhysicsUpdate = true;
	m_synchronousParticlesUpdate = true;

	Cleanup();
	ndResetTimer();
	ApplyOptions();

#if 0
	//Test0__();
	//Test1__();
	//SimpleRegressionBrainStressTest();
	ndHandWrittenDigits();
	//ndCifar10ImageClassification();
#endif
}

ndDemoEntityManager::~ndDemoEntityManager ()
{
	Cleanup ();

	// destroy the empty world
	if (m_world) 
	{
		delete m_world;
	}
}

ndPhysicsWorld* ndDemoEntityManager::GetWorld() const
{
	return m_world;
}

ndSharedPtr<ndRender>& ndDemoEntityManager::GetRenderer()
{
	return m_renderer;
}

void ndDemoEntityManager::Terminate()
{
	m_renderer->Terminate();
}

ndInt32 ndDemoEntityManager::GetWidth() const
{
	return m_renderer->GetWidth();
}

ndInt32 ndDemoEntityManager::GetHeight() const
{
	return m_renderer->GetHeight();
}

bool ndDemoEntityManager::GetKeyState(ndInt32 key) const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[key];
	return state;
}

bool ndDemoEntityManager::AnyKeyDown() const
{
	const ImGuiIO& io = ImGui::GetIO();
	for (ndInt32 i = 0; i < ImGuiKey_COUNT; ++i)
	{
		if (io.KeysDown[i])
		{
			return true;
		}
	}
	return false;
}

void ndDemoEntityManager::CharCallback(ndUnsigned32 ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}

void ndDemoEntityManager::CursorposCallback(ndReal x, ndReal y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2(x, y);
}

void ndDemoEntityManager::MouseScrollCallback(ndReal, ndReal y)
{
	//ndTrace(("%f %f\n", x, y));
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += y;
}

void ndDemoEntityManager::MouseButtonCallback(ndInt32 button, ndInt32 action)
{
	const ndInt32 KEY_PRESS = 1;
	const ndInt32 KEY_RELEASE = 0;

	if (button >= 0 && button < 3) 
	{
		ImGuiIO& io = ImGui::GetIO();
		if (action == KEY_PRESS)
		{
			io.MouseDown[button] = true;    
		} 
		else if (action == KEY_RELEASE)
		{
			io.MouseDown[button] = false;    
		}
	}
}

void ndDemoEntityManager::KeyCallback(ndInt32 key, ndInt32)
{
	if (key == ImGuiKey_F1)
	{
		// reload the demo. 
		const ndTransform transform(m_renderer->GetCamera()->GetTransform());
		LoadDemo(m_lastCurrentScene);
		m_renderer->GetCamera()->SetTransform(transform);
		m_renderer->GetCamera()->SetTransform(transform);
	}
	else if (key == ImGuiKey_F10)
	{
		// set debug tracer here;
		//ndAssert(0);
	}
}

bool ndDemoEntityManager::IsShiftKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	const ndInt32 KEY_LEFT_SHIFT = 340;
	const ndInt32 KEY_RIGHT_SHIFT = 344;
	bool state = io.KeysDown[KEY_LEFT_SHIFT] || io.KeysDown[KEY_RIGHT_SHIFT];
	return state;
}

bool ndDemoEntityManager::IsControlKeyDown () const
{
	ndAssert(0);
	return 0;
	//const ImGuiIO& io = ImGui::GetIO();
	//bool state = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	//return state;
}

bool ndDemoEntityManager::GetCaptured() const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse;
}

bool ndDemoEntityManager::GetMouseKeyState (ndInt32 button) const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.MouseDown[button];
}

bool ndDemoEntityManager::JoystickDetected() const
{
	ndAssert(0);
	return 0;
	//return glfwJoystickPresent(0) ? true : false;
}

void ndDemoEntityManager::GetJoystickAxis(ndFixSizeArray<ndFloat32, 8>&)
{
	ndAssert(0);
	//if (JoystickDetected())
	//{
	//	bool isInitialized = false;
	//	static ndFixSizeArray<ndFloat32, 8> initialValues;
	//	if (!initialValues.GetCount())
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		axisCount = ndMin(axisCount, axisValues.GetCapacity());
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			initialValues.PushBack(axis[i]);
	//		}
	//	}
	//	
	//	if (!isInitialized)
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			ndFloat32 diff = ndAbs(axis[i] - initialValues[i]);
	//			isInitialized = isInitialized || (diff != ndFloat32(0.0f));
	//		}
	//	}
	//
	//	axisValues.SetCount(0);
	//	for (ndInt32 i = 0; i < axisValues.GetCapacity(); ++i)
	//	{
	//		axisValues.PushBack(ndFloat32 (1.0f));
	//	}
	//	axisValues[0] = 0.0f;
	//
	//	if (isInitialized)
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		axisCount = ndMin(axisCount, axisValues.GetCapacity());
	//
	//		axisValues.SetCount(0);
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			axisValues.PushBack(axis[i]);
	//		}
	//	}
	//}
}

void ndDemoEntityManager::GetJoystickButtons(ndFixSizeArray<char, 32>&)
{
	ndAssert(0);
	//if (JoystickDetected())
	//{
	//	ndInt32 buttonsCount = 0;
	//	axisbuttons.SetCount(0);
	//	const unsigned char* const buttons = glfwGetJoystickButtons(0, &buttonsCount);
	//	buttonsCount = ndMin(buttonsCount, axisbuttons.GetCapacity());
	//
	//	for (ndInt32 i = 0; i < buttonsCount; ++i)
	//	{
	//		axisbuttons.PushBack(char(buttons[i]));
	//	}
	//}
}

void ndDemoEntityManager::RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate)
{
	m_onPostUpdate = postUpdate;
}

void ndDemoEntityManager::AddEntity(const ndSharedPtr<ndRenderSceneNode>& entity)
{
	ndScopeSpinLock lock(m_addDeleteLock);
	m_renderer->AddSceneNode(entity);
}

void ndDemoEntityManager::RemoveEntity (const ndSharedPtr<ndRenderSceneNode>& entity)
{
	ndScopeSpinLock lock(m_addDeleteLock);
	m_renderer->RemoveSceneNode(entity);
}

void ndDemoEntityManager::Cleanup ()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) 
	{
		m_world->Sync();
	}

	m_renderer->ResetScene();
	RegisterPostUpdate(ndSharedPtr<OnPostUpdate>(nullptr));
	
	// destroy the Newton world
	if (m_world) 
	{
		// get serialization call back before destroying the world
		m_world->CleanUp();
		delete m_world;
	}
	
	// create the newton world
	m_world = new ndPhysicsWorld(this);
	ApplyMenuOptions();
}

void ndDemoEntityManager::ApplyMenuOptions()
{
	m_world->Sync();
	m_world->SetSubSteps(m_solverSubSteps);
	m_world->SetSolverIterations(m_solverPasses);
	m_world->SetThreadCount(m_workerThreads);
	
	bool state = m_autoSleepMode ? true : false;
	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		body->SetAutoSleep(state);
	}
	
	SetParticleUpdateMode();
	m_world->SelectSolver(m_solverMode);
	m_solverMode = m_world->GetSelectedSolver();
}

void ndDemoEntityManager::ApplyOptions()
{
	m_colorRenderPass->MakeActive(!m_hideVisualMeshes);
	m_shadowRenderPass->MakeActive(!m_hideVisualMeshes);
	m_transparentRenderPass->MakeActive(!m_hideVisualMeshes);

	ndDebugDisplayRenderPass* const debugDisplay = (ndDebugDisplayRenderPass*)*m_debugDisplayRenderPass;
	debugDisplay->SetDisplayMode(m_collisionDisplayMode);
}

void ndDemoEntityManager::ShowMainMenuBar()
{
	ndMenuSelection menuSelection = m_none;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File")) 
		{
			m_suspendPhysicsUpdate = true;
	
			if (ImGui::MenuItem("Preferences", "")) 
			{
				ndAssert (0);
			}
			ImGui::Separator();
	
			if (ImGui::MenuItem("New", "")) 
			{
				menuSelection = m_new;
			}
	
			ImGui::Separator();
			if (ImGui::MenuItem("import ply file", "")) 
			{
				//mainMenu = 4;
			}
	
			ImGui::Separator();
			if (ImGui::MenuItem("Exit", "")) 
			{
				m_renderer->Terminate();
			}
	
			ImGui::EndMenu();
		}
	
		if (ImGui::BeginMenu("Demos")) 
		{
			m_suspendPhysicsUpdate = true;
			ndInt32 demosCount = ndInt32 (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
			for (ndInt32 i = 0; i < demosCount; ++i) 
			{
				if (ImGui::MenuItem(m_demosSelection[i].m_name, "")) 
				{
					m_currentScene = i;
				}
			}
	
			ImGui::EndMenu();
		}
	
		bool optionsOn = ImGui::BeginMenu("Options");
		if (optionsOn) 
		{
			m_updateMenuOptions = true;
			m_suspendPhysicsUpdate = true;
	
			ImGui::Checkbox("auto sleep mode", &m_autoSleepMode);
			ImGui::Checkbox("show UI", &m_showUI);
			ImGui::Checkbox("show stats", &m_showStats);
			ImGui::Checkbox("show helper legend", &m_helperLegend);
			ImGui::Checkbox("synchronous physics update", &m_synchronousPhysicsUpdate);
			ImGui::Checkbox("synchronous particle update", &m_synchronousParticlesUpdate);
			ImGui::Separator();
	
			ImGui::Text("solvers");
			ndInt32 solverMode(m_solverMode);
			ImGui::RadioButton("default", &solverMode, ndWorld::ndStandardSolver);
			ImGui::RadioButton("sse", &solverMode, ndWorld::ndSimdSoaSolver);
			ImGui::RadioButton("avx2", &solverMode, ndWorld::ndSimdAvx2Solver);
	
			m_solverMode = ndWorld::ndSolverModes(solverMode);
			ImGui::Separator();
	
			ImGui::Text("solver sub steps");
			ImGui::SliderInt("##solv", &m_solverSubSteps, 2, 8);
			ImGui::Text("iterative solver passes");
			ImGui::SliderInt("##intera", &m_solverPasses, 4, 32);
			ImGui::Text("worker threads");
			ImGui::SliderInt("##worker", &m_workerThreads, 1, ndThreadPool::GetMaxThreads());
			ImGui::Separator();

			//ImGui::RadioButton("show UI", &m_showUI);
			ImGui::RadioButton("hide collision Mesh", &m_collisionDisplayMode, 0);
			ImGui::RadioButton("show solid collision", &m_collisionDisplayMode, 1);
			ImGui::RadioButton("show wire frame collision", &m_collisionDisplayMode, 2);
			ImGui::RadioButton("show hidden wire frame collision", &m_collisionDisplayMode, 3);
			ImGui::Separator();
	
			ImGui::Checkbox("show aabb", &m_showAABB);
			ImGui::Checkbox("show broad phase", &m_showScene);
			ImGui::Checkbox("show concave edges", &m_showConcaveEdge);
			ImGui::Checkbox("hide post update", &m_hidePostUpdate);
			ImGui::Checkbox("hide visual meshes", &m_hideVisualMeshes);
			ImGui::Checkbox("show mesh skeleton", &m_showMeshSkeleton);
			ImGui::Checkbox("show contact points", &m_showContactPoints);
			ImGui::Checkbox("show ray cast hit point", &m_showRaycastHit);
			ImGui::Checkbox("show normal forces", &m_showNormalForces);
			ImGui::Checkbox("show center of mass", &m_showCenterOfMass);
			ImGui::Checkbox("show body frame", &m_showBodyFrame);
			ImGui::Checkbox("show joints debug info", &m_showJointDebugInfo);
			ImGui::Checkbox("show models debug info", &m_showModelsDebugInfo);
			ImGui::Checkbox("show colliding faces", &m_showCollidingFaces);

			ApplyOptions();
	
			ImGui::EndMenu();
	
			SetParticleUpdateMode();
		}

		if (ImGui::BeginMenu("MachineLearning"))
		{
			ndInt32 demosCount = ndInt32(sizeof(m_machineLearning) / sizeof m_machineLearning[0]);
			for (ndInt32 i = 0; i < demosCount; ++i)
			{
				if (ImGui::MenuItem(m_machineLearning[i].m_name, ""))
				{
					m_currentScene = i + MACHINE_LEARNING_BASE;
				}
			}
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help")) 
		{
			m_suspendPhysicsUpdate = true;
			ImGui::EndMenu();
		}
	
		ImGui::EndMainMenuBar();
	
		if (!optionsOn && m_updateMenuOptions) 
		{
			m_updateMenuOptions = false;
			ApplyMenuOptions();
		}
	}
	
	switch (menuSelection)
	{
		case m_new:
		{
			// menu new 
			ndAssert(0);
			//ndMatrix matrix (GetCamera()->GetCurrentMatrix());
			//Cleanup();
			//ApplyMenuOptions();
			//ResetTimer();
			//m_currentScene = -1;
			//SetCameraMatrix(ndQuaternion(matrix), matrix.m_posit);
			break;
		}
	
		case m_none:
		default:
		{
			// load a demo 
			if (m_currentScene != -1) 
			{
				//m_selectedModel = nullptr;
				//RegisterPostUpdate(nullptr);
				LoadDemo(m_currentScene);
				m_lastCurrentScene = m_currentScene;
				m_currentScene = -1;
			}
		}
	}
}

void ndDemoEntityManager::LoadDemo(ndInt32 menuIndex)
{
	Cleanup();
	
	char newTitle[256];

	// add a demo camera per demo
	m_demoHelper = ndSharedPtr<ndDemoHelper>(nullptr);
	m_defaultCamera = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFlyby(*m_renderer));
	m_renderer->SetCamera(m_defaultCamera);

	if (menuIndex < MACHINE_LEARNING_BASE)
	{
		m_demosSelection[menuIndex].m_demoLauncher(this);
		snprintf(newTitle, sizeof(newTitle), "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_demosSelection[menuIndex].m_name);
	}
	else
	{
		menuIndex -= MACHINE_LEARNING_BASE;
		m_machineLearning[menuIndex].m_demoLauncher(this);
		snprintf(newTitle, sizeof(newTitle), "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_machineLearning[menuIndex].m_name);
	}

	m_renderer->SetTitle(newTitle);
	ApplyMenuOptions();
	ndResetTimer();
	
	ndAssert (m_world->ValidateScene());
}

bool ndDemoEntityManager::GetMouseSpeed(ndFloat32& speedX, ndFloat32& speedY) const
{
	ImVec2 speed(ImGui::GetMouseDragDelta(0, 0.0f));
	speedX = speed.x;
	speedY = speed.y;
	return true;
}

bool ndDemoEntityManager::GetMousePosition (ndFloat32& posX, ndFloat32& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = ndClamp(posit.x, ndReal(-1.0e10f), ndReal(1.0e10f));
	posY = ndClamp(posit.y, ndReal(-1.0e10f), ndReal(1.0e10f));
	return true;
}

void ndDemoEntityManager::ToggleProfiler()
{
	#ifdef D_PROFILER
		ndAssert(m_world);
		ndTrace(("profiler Enable\n"));
		m_world->Sync();
		dProfilerEnableProling();
	#endif
}

ndInt32 ndDemoEntityManager::ParticleCount() const
{
	ndInt32 count = 0;
	const ndBodyList& particles = m_world->GetParticleList();
	for (ndBodyList::ndNode* node = particles.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const set = node->GetInfo()->GetAsBodyParticleSet();
		count += ndInt32(set->GetPositions().GetCount());
	}
	return count;
}

void ndDemoEntityManager::SetParticleUpdateMode() const
{
	const ndBodyList& particles = m_world->GetParticleList();
	for (ndBodyList::ndNode* node = particles.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const set = node->GetInfo()->GetAsBodyParticleSet();
		set->SetAsynUpdate(!m_synchronousParticlesUpdate);
	}
}

void ndDemoEntityManager::RenderStats()
{
	if (m_showStats) 
	{
		char text[1024];
		
		if (ImGui::Begin("statistics", &m_showStats)) 
		{
			snprintf(text, sizeof (text), "fps:            %6.3f", m_fps);
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "physics time:  %6.3f ms", m_world->GetAverageUpdateTime() * 1.0e3f);
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "update mode:    %s", m_synchronousPhysicsUpdate ? "synchronous" : "asynchronous");
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "particle mode:  %s", m_synchronousParticlesUpdate ? "synchronous" : "asynchronous");
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "bodies:         %d", m_world->GetBodyList().GetCount());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "joints:         %d", m_world->GetJointList().GetCount());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "contact joints: %d", m_world->GetContactList().GetActiveContacts());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "particles:      %d", ParticleCount());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "memory used:   %6.3f mbytes", ndFloat32(ndFloat64(ndMemory::GetMemoryUsed()) / (1024 * 1024)));
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "threads:        %d", m_world->GetThreadCount());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "iterations:     %d", m_world->GetSolverIterations());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "Substeps:       %d", m_world->GetSubSteps());
			ImGui::Text(text, "");
	
			snprintf(text, sizeof (text), "solver:         %s", m_world->GetSolverString());
			ImGui::Text(text, "");
	
			m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsWindowHovered() && ImGui::IsMouseDown(0));
			ImGui::End();
		}
	}
	
	if (*m_demoHelper)
	{
		if (m_helperLegend)
		{	
			m_helperLegend = false;
			m_demoHelper->ResetTime();
		}

		if (!m_demoHelper->ExpirationTime())
		{
			bool dummy = true;
			if (ImGui::Begin("User Interface", &dummy))
			{
				m_demoHelper->PresentHelp(this);
				ImGui::End();
			}
		}

		//m_demoHelper->ExpirationTime();
		//{
		//	m_demoHelper = ndSharedPtr<ndDemoHelper>(nullptr);
		//}
		//if (!m_demoHelper->ExpirationTime())
		//{
		//	m_demoHelper->PresentHelp(this);
		//	ImGui::End();
		//}
	}
	
	ShowMainMenuBar();
}

void ndDemoEntityManager::SetDemoHelp(ndSharedPtr<ndDemoHelper>& helper)
{
	m_demoHelper = helper;
}

void ndDemoEntityManager::SetNextActiveCamera()
{
	if (!m_nextActiveCamera.Update(GetKeyState(ImGuiKey_C) ? true : false))
	{
		return;
	}

	ndFixSizeArray<const ndRenderSceneCamera*, 256> cameraPallete;
	cameraPallete.PushBack(m_defaultCamera->FindCameraNode());

	ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_renderer->GetScene();
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* sceneNode = scene.GetFirst(); sceneNode; sceneNode = sceneNode->GetNext())
	{
		ndSharedPtr<ndRenderSceneNode>& node = sceneNode->GetInfo();
		const ndRenderSceneCamera* cameraNode = node->FindCameraNode();
		if (cameraNode)
		{
			cameraPallete.PushBack(cameraNode);
		}
	}

	const ndRenderSceneCamera* const currentCamera = m_renderer->GetCamera()->FindCameraNode();
	for (ndInt32 i = 0; i < cameraPallete.GetCount(); ++i)
	{
		if (cameraPallete[i] == currentCamera)
		{
			ndInt32 j = (i + 1) % cameraPallete.GetCount();
			if (j == 0)
			{
				m_renderer->SetCamera(m_defaultCamera);
			}
			else
			{
				ndRenderSceneNode* const camera = cameraPallete[j]->FindByName("__PlayerCamera__");
				ndSharedPtr<ndRenderSceneNode> cameraNode(camera->GetSharedPtr());
				m_renderer->SetCamera(cameraNode);
			}
			break;
		}
	}
}

void ndDemoEntityManager::CalculateFPS(ndFloat32 timestep)
{
	m_framesCount ++;
	m_timestepAcc += timestep;

	// this probably happing on loading of and a pause, just rest counters
	if ((m_timestepAcc <= 0.0f) || (m_timestepAcc > 4.0f))
	{
		m_timestepAcc = 0;
		m_framesCount = 0;
	}

	//update fps every quarter of a second
	const ndFloat32 movingAverage = 0.5f;
	if (m_timestepAcc >= movingAverage)
	{
		m_fps = ndFloat32 (m_framesCount) / m_timestepAcc;
		m_timestepAcc -= movingAverage;
		m_framesCount = 0;
	}
}

//void ndDemoEntityManager::ImportPLYfile (const char* const fileName)
void ndDemoEntityManager::ImportPLYfile(const char* const)
{
	ndAssert(0);
	//m_collisionDisplayMode = 2;
	//CreatePLYMesh (this, fileName, true);
}

ndInt32 ndDemoEntityManager::Print (const ndVector&, const char *fmt, ... ) const
{
	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsnprintf (string, sizeof (string), fmt, argptr);
	va_end( argptr );
	ImGui::Text(string, "");
	return 0;
}

void ndDemoEntityManager::SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position)
{
	ndRenderSceneNode* const cameraNode = *m_renderer->GetCamera();
	cameraNode->SetTransform(rotation, position);
	cameraNode->SetTransform(rotation, position);
}

void ndDemoEntityManager::UpdatePhysics(ndFloat32 timestep)
{
	// update the physics
	if (m_world && !m_suspendPhysicsUpdate) 
	{
		m_world->AdvanceTime(timestep);
	}
}

void ndDemoEntityManager::SetAcceleratedUpdate()
{
	m_world->AccelerateUpdates();
}

//void ndDemoEntityManager::OnSubStepPostUpdate(ndFloat32 timestep)
void ndDemoEntityManager::OnSubStepPostUpdate(ndFloat32)
{
	//if (m_colorRenderPass)
	//{
	////	((ndRenderPassColor*)m_colorRenderPass)->UpdateDebugDisplay(timestep);
	//}
}

void ndDemoEntityManager::RenderScene()
{
	D_TRACKTIME();
	ndFloat32 timestep = ndGetElapsedSeconds();	
	CalculateFPS(timestep);
	UpdatePhysics(timestep);
	m_renderer->Render();
}

void ndDemoEntityManager::TestImGui()
{
	// Main loop
	bool show_demo_window = true;
	bool show_another_window = false;

	// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
	if (show_demo_window)
	{
		ImGui::ShowDemoWindow(&show_demo_window);
	}

	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	if (1)
	{
		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
		ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Another Window", &show_another_window);

		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
		ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
		ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

		if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
			counter++;
		ImGui::SameLine();
		ImGui::Text("counter = %d", counter);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

	// 3. Show another simple window.
	if (show_another_window)
	{
		ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Hello from another window!");
		if (ImGui::Button("Close Me"))
		{
			show_another_window = false;
		}
		ImGui::End();
	}
	ImGui::Render();
}

void ndDemoEntityManager::Run()
{
	// Main loop
	ndFloatExceptions exception;
	while (!m_renderer->ShouldFinish())
	{
		if (m_profilerMode)
		{
			ToggleProfiler();
			m_profilerMode = false;
		}
	
		m_suspendPhysicsUpdate = false;
		D_TRACKTIME();
	
		if (m_renderer->PollEvents())
		{
			RenderScene();
		}
	}
}

