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
#include "ndDemoMesh.h"
#include "ndVehicleUI.h"
#include "ndMeshLoader.h"
#include "ndDemoEntity.h"
#include "ndDemoCamera.h"
#include "ndShaderCache.h"
#include "ndFileBrowser.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndTestDeepBrain.h"
#include "ndPngToOpenGl.h"
#include "ndColorRenderPass.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraManager.h"
#include "ndHighResolutionTimer.h"
#include "ndShadowsMapRenderPass.h"

//#define DEFAULT_SCENE	0		// basic rigidbody
//#define DEFAULT_SCENE	1		// Gpu basic rigidbody
//#define DEFAULT_SCENE	2		// friction ramp
//#define DEFAULT_SCENE	3		// basic compound shapes
//#define DEFAULT_SCENE	4		// conservation of momentum 
//#define DEFAULT_SCENE	5		// basic Stacks
//#define DEFAULT_SCENE	6		// basic Trigger
//#define DEFAULT_SCENE	7		// object Placement
//#define DEFAULT_SCENE	8		// particle fluid
//#define DEFAULT_SCENE	9		// static mesh collision 
//#define DEFAULT_SCENE	10		// static user mesh collision 
//#define DEFAULT_SCENE	11		// basic joints
//#define DEFAULT_SCENE	12		// basic vehicle
//#define DEFAULT_SCENE	13		// heavy vehicle
//#define DEFAULT_SCENE	14		// background vehicle prop
//#define DEFAULT_SCENE	15		// basic player
//#define DEFAULT_SCENE	16		// rag doll
//#define DEFAULT_SCENE	17		// cart pole discrete controller
//#define DEFAULT_SCENE	18		// cart pole continue controller
//#define DEFAULT_SCENE	19		// unit cycle controller
//#define DEFAULT_SCENE	20		// quadruped test 1
#define DEFAULT_SCENE	21		// quadruped test 2
//#define DEFAULT_SCENE	22		// quadruped test 3
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
						 
// demos forward declaration 
void ndRagdollTest(ndDemoEntityManager* const scene);
void ndBipedTest_1(ndDemoEntityManager* const scene);
void ndBipedTest_2(ndDemoEntityManager* const scene);
void ndBasicStacks(ndDemoEntityManager* const scene);
void ndBasicJoints(ndDemoEntityManager* const scene);
void ndBasicVehicle(ndDemoEntityManager* const scene);
void ndHeavyVehicle(ndDemoEntityManager* const scene);
void ndBasicTrigger(ndDemoEntityManager* const scene);
void ndBasicGpuTest0(ndDemoEntityManager* const scene);
void ndBasicRigidBody(ndDemoEntityManager* const scene);
void ndQuadruped_animation_test(ndDemoEntityManager* const scene);
void ndQuadruped_sac_test(ndDemoEntityManager* const scene);
void ndQuadrupedTest_3(ndDemoEntityManager* const scene);
void ndQuadrupedTest_4(ndDemoEntityManager* const scene);
void ndQuadrupedTest_4(ndDemoEntityManager* const scene);
void ndObjectPlacement(ndDemoEntityManager* const scene);
void ndBasicGpuRigidBody(ndDemoEntityManager* const scene);
void ndBasicFrictionRamp(ndDemoEntityManager* const scene);
void ndPlayerCapsuleDemo(ndDemoEntityManager* const scene);
void ndBipedTest_2Trainer(ndDemoEntityManager* const scene);
void ndBasicParticleFluid(ndDemoEntityManager* const scene);
void ndUnicycleController(ndDemoEntityManager* const scene);
void ndBasicAngularMomentum(ndDemoEntityManager* const scene);
void ndBagroundLowLodVehicle(ndDemoEntityManager* const scene);
void ndSimpleIndustrialRobot(ndDemoEntityManager* const scene);
void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene);
void ndAdvancedIndustrialRobot(ndDemoEntityManager* const scene);
void ndBasicExplodeConvexShape(ndDemoEntityManager* const scene);
void ndCartpoleDiscrete(ndDemoEntityManager* const scene);
void ndCartpoleContinue(ndDemoEntityManager* const scene);

//void ndBasicFracture_0(ndDemoEntityManager* const scene);
//void ndBasicFracture_2(ndDemoEntityManager* const scene);
//void ndBasicFracture_4(ndDemoEntityManager* const scene);
void ndStaticMeshCollisionDemo(ndDemoEntityManager* const scene);
void ndStaticUserMeshCollisionDemo(ndDemoEntityManager* const scene);

ndDemoEntityManager::SDKDemos ndDemoEntityManager::m_demosSelection[] = 
{
	{ "basic rigidbody", ndBasicRigidBody},
	{ "basic gpu rigidbody", ndBasicGpuRigidBody},
	{ "basic friction ramp", ndBasicFrictionRamp},
	{ "basic compound shapes", ndBasicCompoundShapeDemo},
	{ "basic conservation of momentum", ndBasicAngularMomentum},
	{ "basic stack", ndBasicStacks},
	{ "basic trigger", ndBasicTrigger},
	{ "basic object placement", ndObjectPlacement},
	{ "basic particle fluid", ndBasicParticleFluid},
	{ "static mesh", ndStaticMeshCollisionDemo},
	{ "static user mesh", ndStaticUserMeshCollisionDemo},
	{ "basic joints", ndBasicJoints},
	{ "basic vehicle", ndBasicVehicle},
	{ "heavy vehicle", ndHeavyVehicle},
	{ "low lod vehicle", ndBagroundLowLodVehicle},
	{ "basic player", ndPlayerCapsuleDemo},
	{ "rag doll", ndRagdollTest},
	{ "cartpole discrete controller", ndCartpoleDiscrete},
	{ "cartpole continue controller", ndCartpoleContinue},
	{ "unicycle controller", ndUnicycleController},
	{ "quadruped test 1", ndQuadruped_animation_test},
	{ "quadruped test 2", ndQuadruped_sac_test},
	{ "quadruped test 3", ndQuadrupedTest_3},
//	{ "quadruped test 4", ndQuadrupedTest_4},
//	{ "quadruped test 5", ndQuadrupedTest_4},
	{ "simple industrial robot", ndSimpleIndustrialRobot},
	{ "advanced industrial robot", ndAdvancedIndustrialRobot},
//	{ "biped test one", ndBipedTest_1},
//	{ "biped test two", ndBipedTest_2},
//	{ "train biped test two", ndBipedTest_2Trainer},
//	{ "simple convex fracture", ndBasicExplodeConvexShape},
//	//{ "basic convex fracture", ndBasicFracture_0},
//	//{ "linked convex fracture", ndBasicFracture_2},
//	//{ "simple skin peeling fracture", ndBasicFracture_4},
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
	return (!m_memory0 & m_memory1) ? 1 : 0;
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
		ndBrainFloat arg = 2.0f * ndPi * ndBrainFloat(i) / sinWavePeriod;
		ndBrainFloat a = ndSin(arg);
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
	//ndSharedPtr<ndBrainContext> context(new ndBrainCpuContext);
	ndTrainerDescriptor descriptor;
	descriptor.m_brain = brain;
	descriptor.m_context = context;
	descriptor.m_learnRate = 1.0e-4f;
	descriptor.m_minibatchSize = minibatchSize;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		brain->AddLayer(layers[i]);
	}
	brain->InitWeights();

	ndSharedPtr<ndBrainTrainer> trainer(new ndBrainTrainer(descriptor));
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
			trainer->ApplyLearnRate();
			context->SyncBufferCommandQueue();

			minibatchInputGradientBuffer->VectorFromDevice(miniBatchInputGradients);
			epoch *= 1;
		}
	}
	time = ndGetTimeInMicroseconds() - time;

	ndExpandTraceMessage("Stress test Regresion\n");
	ndExpandTraceMessage(" training time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
}

// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
ndDemoEntityManager::ndDemoEntityManager()
	:m_mainFrame(nullptr)
	,m_defaultFont(0)
	,m_sky(nullptr)
	,m_world(nullptr)
	,m_cameraManager(nullptr)
	,m_updateCameraContext(nullptr)
	,m_renderDemoGUI()
	,m_updateCamera(nullptr)
	,m_microsecunds(0)
	,m_transparentHeap()
	,m_animationCache()
	,m_currentScene(DEFAULT_SCENE)
	,m_lastCurrentScene(DEFAULT_SCENE)
	,m_framesCount(0)
	,m_physicsFramesCount(0)
	,m_currentPlugin(0)
	,m_solverPasses(4)
	,m_solverSubSteps(2)
	,m_workerThreads(1)
	,m_debugDisplayMode(0)
	,m_collisionDisplayMode(0)
	,m_selectedModel(nullptr)
	,m_onPostUpdate(nullptr)
	,m_fps(0.0f)
	,m_timestepAcc(0.0f)
	,m_currentListenerTimestep(0.0f)
	,m_addDeleteLock()
	,m_showUI(true)
	,m_showAABB(false)
	,m_showStats(true)
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
	,m_solverMode(ndWorld::ndSimdSoaSolver)
	,m_colorRenderPass(new ndColorRenderPass())
	,m_shadowRenderPass(new ndShadowMapRenderPass())
{
	// Setup window
	glfwSetErrorCallback(ErrorCallback);
	glfwInit();

	// Decide GL+GLSL versions
	#if defined(IMGUI_IMPL_OPENGL_ES2)
	// GL ES 2.0 + GLSL 100
	const char* glsl_version = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
	#elif defined(__APPLE__)
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
	#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);

	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
	#endif

	#if defined (_DEBUG)
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
	#endif

	//m_hasJoytick = glfwJoystickPresent(0) ? true : false;

	// Create window with graphics context
	char version[256];
	snprintf(version, sizeof (version), "Newton Dynamics %d.%.2i sandbox demos", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_mainFrame = glfwCreateWindow(1280, 768, version, nullptr, nullptr);
	glfwMakeContextCurrent(m_mainFrame);
	glfwSwapInterval(0); // Enable vsync

	 // Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	//io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking

	ndInt32 monitorsCount;
	GLFWmonitor** monitors = glfwGetMonitors(&monitorsCount);
	if (monitorsCount > 1)
	{
		ndInt32 window_x;
		ndInt32 window_y;
		ndInt32 monitor_x;
		ndInt32 monitor_y;

		glfwGetMonitorPos(monitors[1], &monitor_x, &monitor_y);
		glfwGetWindowPos(m_mainFrame, &window_x, &window_y);
		glfwSetWindowPos(m_mainFrame, monitor_x + window_x, monitor_y + 64);
	}

	// Setup Dear ImGui style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsLight();
	//ImGui::StyleColorsClassic();
	ImGuiStyle* const style = &ImGui::GetStyle();
	style->Colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 0.5f);

	// Setup Platform/Renderer back ends
	ImGui_ImplGlfw_InitForOpenGL(m_mainFrame, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// attach myself to the main frame
	glfwSetWindowUserPointer(m_mainFrame, this);

#if (defined(_DEBUG) && defined(WIN32))	
	glDebugMessageCallback(OpenMessageCallback, m_mainFrame);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
#endif

	// Load Fonts
	LoadFont();

	glfwSetKeyCallback(m_mainFrame, KeyCallback);
	glfwSetCharCallback(m_mainFrame, CharCallback);
	glfwSetScrollCallback(m_mainFrame, MouseScrollCallback);
	glfwSetCursorPosCallback(m_mainFrame, CursorposCallback);
	glfwSetMouseButtonCallback(m_mainFrame, MouseButtonCallback);

	// Setup ImGui binding
	io.UserData = this;

	// Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
	io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;
	io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
	io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
	io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
	io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
	io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
	io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
	io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
	io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
	io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
	io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
	io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
	io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
	io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
	io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
	io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
	io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
	io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
	io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

	#ifdef _MSC_VER 
	io.ImeWindowHandle = glfwGetWin32Window(m_mainFrame);
	#endif

	m_mousePressed[0] = false;
	m_mousePressed[1] = false;
	m_mousePressed[2] = false;
	
	// initialized the physics world for the new scene
	//m_showUI = false;
	//m_showAABB = true;
	//m_showScene = true;
	//m_showConcaveEdge = true;
	//m_showMeshSkeleton = true;
	//m_autoSleepMode = false;
	m_hidePostUpdate = true;
	//m_hideVisualMeshes = true;
	//m_solverMode = ndWorld::ndCudaSolver;
	//m_solverMode = ndWorld::ndSimdSoaSolver;
	m_solverMode = ndWorld::ndStandardSolver;
	//m_solverMode = ndWorld::ndSimdAvx2Solver;
	//m_solverPasses = 4;
	m_workerThreads = 4;
	//m_solverSubSteps = 2;
	//m_showRaycastHit = true;
	//m_showCenterOfMass = false;
	//m_showNormalForces = true;
	//m_showContactPoints = true;
	//m_showJointDebugInfo = true;
	//m_showModelsDebugInfo = true;
	//m_collisionDisplayMode = 1;
	//m_collisionDisplayMode = 2;	
	m_collisionDisplayMode = 3;		// solid wire frame
	m_synchronousPhysicsUpdate = true;
	m_synchronousParticlesUpdate = true;

	m_shaderCache.CreateAllEffects();
	m_colorRenderPass->Init(this, 0);
	m_shadowRenderPass->Init(this, 1, m_shaderCache.m_shadowMaps);

	Cleanup();
	ResetTimer();

	m_diretionalLightDir = ndVector(-1.0f, 1.0f, 1.0f, 0.0f).Normalize();

	//Test0__();
	//Test1__();
	//SimpleRegressionBrainStressTest();
	ndHandWrittenDigits();
	//ndCifar10ImageClassification();
	//TargaToPng();
}

ndDemoEntityManager::~ndDemoEntityManager ()
{
	Cleanup ();

	if (m_cameraManager)
	{
		delete m_cameraManager;
	}

	// destroy the empty world
	if (m_world) 
	{
		delete m_world;
	}

	delete m_colorRenderPass;
	delete m_shadowRenderPass;

	// Cleanup
	GLuint font_texture = GLuint(m_defaultFont);
	glDeleteTextures(1, &font_texture);
	ImGui::GetIO().Fonts->TexID = 0;

	m_shaderCache.Cleanup();
	TextureCacheCleanUp();

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(m_mainFrame);
	glfwTerminate();
}

void ndDemoEntityManager::Terminate()
{
	glfwSetWindowShouldClose(m_mainFrame, 1);
}

#if (defined(_DEBUG) && defined(WIN32))
void APIENTRY ndDemoEntityManager::OpenMessageCallback(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam)
{
	if (userParam)
	{
		switch(id)
		{
			case 2:		 // no sure why on Intel embedded systems I get this warding, 
				         // ID_RECOMPILE_FRAGMENT_SHADER performance warning has been generated.
						 // Fragment shader recompiled due to state change., length = 120
			case 131154:  // Pixel-path performance warning: Pixel transfer is synchronized with 3D rendering.
			case 131185:  // nvidia driver report will use VIDEO memory as the source for buffer object operations
			case 131139: //	for some reason when using different target I get this on nvidia gpus.
						 //	no one seems to know what cause this
					     // Rasterization quality warning : A non - fullscreen clear caused a fallback from CSAA to MSAA.
				return;
		}
		ndTrace(("GL CALLBACK: %s source = 0x%x, type = 0x%x, id = %d, severity = 0x%x, message = %s, length = %d \n",
			(type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), source, type, id, severity, message, length));
		ndAssert(0);
	}
}
#endif

ndInt32 ndDemoEntityManager::GetWidth() const
{
	//ImGuiIO& io = ImGui::GetIO();
	//return (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return w;
}

ndInt32 ndDemoEntityManager::GetHeight() const
{
	//ImGuiIO& io = ImGui::GetIO();
	//return (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return h;
}

ndDemoCamera* ndDemoEntityManager::GetCamera() const
{
	return m_cameraManager->GetCamera();
}

const ndShadowMapRenderPass* ndDemoEntityManager::GetShadowMapRenderPass() const
{
	return (ndShadowMapRenderPass*)m_shadowRenderPass;
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

ndSharedPtr<ndAnimationSequence> ndDemoEntityManager::GetAnimationSequence(ndMeshLoader& loader, const char* const fileName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fileName);
	if (!node)
	{
		ndAnimationSequence* const sequence = loader.LoadAnimation(fileName);
		if (sequence)
		{
			node = m_animationCache.Insert(sequence, fileName);
		}
	}
	return node ? node->GetInfo() : nullptr;
}

bool ndDemoEntityManager::IsShiftKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	return state;
}

bool ndDemoEntityManager::IsControlKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	return state;
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

void ndDemoEntityManager::Set2DDisplayRenderFunction(ndSharedPtr<ndUIEntity>& demoGui)
{
	m_renderDemoGUI = demoGui;
}

void* ndDemoEntityManager::GetUpdateCameraContext() const
{
	return m_updateCameraContext;
}

void ndDemoEntityManager::SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context)
{
	m_updateCamera = callback;
	m_updateCameraContext = context;
}

bool ndDemoEntityManager::JoystickDetected() const
{
	return glfwJoystickPresent(0) ? true : false;
}

void ndDemoEntityManager::GetJoystickAxis (ndFixSizeArray<ndFloat32, 8>& axisValues)
{
	if (JoystickDetected())
	{
		bool isInitialized = false;
		static ndFixSizeArray<ndFloat32, 8> initialValues;
		if (!initialValues.GetCount())
		{
			ndInt32 axisCount = 0;
			const float* const axis = glfwGetJoystickAxes(0, &axisCount);
			axisCount = ndMin(axisCount, axisValues.GetCapacity());
			for (ndInt32 i = 0; i < axisCount; ++i)
			{
				initialValues.PushBack(axis[i]);
			}
		}
		
		if (!isInitialized)
		{
			ndInt32 axisCount = 0;
			const float* const axis = glfwGetJoystickAxes(0, &axisCount);
			for (ndInt32 i = 0; i < axisCount; ++i)
			{
				ndFloat32 diff = ndAbs(axis[i] - initialValues[i]);
				isInitialized = isInitialized || (diff != ndFloat32(0.0f));
			}
		}

		axisValues.SetCount(0);
		for (ndInt32 i = 0; i < axisValues.GetCapacity(); ++i)
		{
			axisValues.PushBack(ndFloat32 (1.0f));
		}
		axisValues[0] = 0.0f;

		if (isInitialized)
		{
			ndInt32 axisCount = 0;
			const float* const axis = glfwGetJoystickAxes(0, &axisCount);
			axisCount = ndMin(axisCount, axisValues.GetCapacity());

			axisValues.SetCount(0);
			for (ndInt32 i = 0; i < axisCount; ++i)
			{
				axisValues.PushBack(axis[i]);
			}
		}
	}
}

void ndDemoEntityManager::GetJoystickButtons(ndFixSizeArray<char, 32>& axisbuttons)
{
	if (JoystickDetected())
	{
		ndInt32 buttonsCount = 0;
		axisbuttons.SetCount(0);
		const unsigned char* const buttons = glfwGetJoystickButtons(0, &buttonsCount);
		buttonsCount = ndMin(buttonsCount, axisbuttons.GetCapacity());

		for (ndInt32 i = 0; i < buttonsCount; ++i)
		{
			axisbuttons.PushBack(char(buttons[i]));
		}
	}
}

void ndDemoEntityManager::ResetTimer()
{
	dResetTimer();
	m_microsecunds = ndGetTimeInMicroseconds ();
}

void ndDemoEntityManager::AddEntity(const ndSharedPtr<ndDemoEntity>& entity)
{
	ndScopeSpinLock lock(m_addDeleteLock);
	ndAssert(!entity->m_rootNode);
	entity->m_rootNode = Append(entity);
}

void ndDemoEntityManager::RemoveEntity (const ndSharedPtr<ndDemoEntity>& entity)
{
	ndScopeSpinLock lock(m_addDeleteLock);
	ndAssert(entity->m_rootNode);
	Remove(entity->m_rootNode);
}

void ndDemoEntityManager::Cleanup ()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) 
	{
		m_world->Sync();
	}
	
	//ndTree<ndAnimationSequence*, ndString>::Iterator iter(m_animationCache);
	//for (iter.Begin(); iter; iter++)
	//{
	//	delete *iter;
	//}

	RegisterPostUpdate(nullptr);
	m_animationCache.RemoveAll();
	
	m_colorRenderPass->Cleanup();
	m_shadowRenderPass->Cleanup();
	
	if (m_cameraManager) 
	{
		delete m_cameraManager;
		m_cameraManager = nullptr;
	}

	if (m_sky)
	{
		delete m_sky;
	}
	
	m_sky = nullptr;
	m_updateCamera = nullptr;
	
	// destroy the Newton world
	if (m_world) 
	{
		// get serialization call back before destroying the world
		m_world->CleanUp();
		delete m_world;
	}
	
	// destroy all remaining visual objects
	while (GetFirst())
	{
		RemoveEntity(GetFirst()->GetInfo());
	}
	
	// create the newton world
	m_world = new ndPhysicsWorld(this);
	
	// add the camera manager
	m_cameraManager = new ndDemoCameraManager(this);
	
	ApplyMenuOptions();
	
	// we start without 2d render
	m_renderDemoGUI = ndSharedPtr<ndUIEntity>();

	m_colorRenderPass->Init(this, 0);
	m_shadowRenderPass->Init(this, 1, m_shaderCache.m_shadowMaps);
}

void ndDemoEntityManager::LoadFont()
{
	// Build texture atlas
	ImGuiIO& io = ImGui::GetIO();

    // Load Fonts
    // (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
    //io.Fonts->AddFontDefault();

	float pixedSize = 18;
	char pathName[2048];
	const char* const name = "Cousine-Regular.ttf";
	//char* const name = "calibri.ttf";
	//char* const name = "courbd.ttf";

	ndGetWorkingFileName (name, pathName);
    io.Fonts->AddFontFromFileTTF(pathName, pixedSize);
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());

	// Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) 
	// because it is more likely to be compatible with user's existing shaders. 
	// If your ImTextureId represent a higher-level concept than just a GL texture id, 
	// consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
	unsigned char* pixels;
	ndInt32 width, height;
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   

	// Upload texture to graphics system
	GLint last_texture;
	GLuint font_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &font_texture);
	glBindTexture(GL_TEXTURE_2D, font_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	m_defaultFont = GLint(font_texture);
	io.Fonts->TexID = (void *)(intptr_t)m_defaultFont;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, GLuint(last_texture));
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

			//ImGui::Separator();
			//if (ImGui::MenuItem("Open", "")) 
			//{
			//	menuSelection = m_load;
			//}
			//if (ImGui::MenuItem("Save", "")) 
			//{
			//	menuSelection = m_save;
			//}
			//if (ImGui::MenuItem("Save model", ""))
			//{
			//	menuSelection = m_saveModel;
			//}

			ImGui::Separator();
			if (ImGui::MenuItem("import ply file", "")) 
			{
				//mainMenu = 4;
			}

			ImGui::Separator();
			if (ImGui::MenuItem("Exit", "")) 
			{
				glfwSetWindowShouldClose (m_mainFrame, 1);
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
			ImGui::Checkbox("synchronous physics update", &m_synchronousPhysicsUpdate);
			ImGui::Checkbox("synchronous particle update", &m_synchronousParticlesUpdate);
			ImGui::Separator();

			ImGui::Text("solvers");
			ndInt32 solverMode(m_solverMode);
			ImGui::RadioButton("default", &solverMode, ndWorld::ndStandardSolver);
			ImGui::RadioButton("sse", &solverMode, ndWorld::ndSimdSoaSolver);
			ImGui::RadioButton("avx2", &solverMode, ndWorld::ndSimdAvx2Solver);
			ImGui::RadioButton("cuda", &solverMode, ndWorld::ndCudaSolver);

			m_solverMode = ndWorld::ndSolverModes(solverMode);
			ImGui::Separator();

			ImGui::Text("solver sub steps");
			ImGui::SliderInt("##solv", &m_solverSubSteps, 2, 8);
			ImGui::Text("iterative solver passes");
			ImGui::SliderInt("##intera", &m_solverPasses, 4, 32);
			ImGui::Text("worker threads");
			ImGui::SliderInt("##worker", &m_workerThreads, 1, ndThreadPool::GetMaxThreads());
			ImGui::Separator();

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

			ImGui::EndMenu();

			SetParticleUpdateMode();
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
			ndMatrix matrix (GetCamera()->GetCurrentMatrix());
			Cleanup();
			ApplyMenuOptions();
			ResetTimer();
			m_currentScene = -1;
			SetCameraMatrix(ndQuaternion(matrix), matrix.m_posit);
			break;
		}

		//case m_load:
		//{
		//	break;
		//}
		//
		//case m_save:
		//{
		//	break;
		//}
		//
		//case m_saveModel:
		//{
		//	break;
		//}

		case m_none:
		default:
		{
			// load a demo 
			if (m_currentScene != -1) 
			{
				m_selectedModel = nullptr;
				RegisterPostUpdate(nullptr);
				LoadDemo(m_currentScene);
				m_lastCurrentScene = m_currentScene;
				m_currentScene = -1;
			}
		}
	}
}

void ndDemoEntityManager::LoadDemo(ndInt32 menu)
{
	char newTitle[256];
	Cleanup();

	// make the sky box 
	CreateSkyBox();
	m_demosSelection[menu].m_launchDemoCallback(this);

	snprintf(newTitle, sizeof(newTitle), "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_demosSelection[menu].m_name);
	glfwSetWindowTitle(m_mainFrame, newTitle);
	ApplyMenuOptions();
	ResetTimer();

	ndAssert (m_world->ValidateScene());
}

void ndDemoEntityManager::ErrorCallback(ndInt32 error, const char* description)
{
	ndTrace (("Error %d: %s\n", error, description));
	fprintf(stderr, "Error %d: %s\n", error, description);
	ndAssert (0);
}

void ndDemoEntityManager::MouseButtonCallback(GLFWwindow*, ndInt32 button, ndInt32 action, ndInt32)
{
	if (button >= 0 && button < 3) 
	{
		ImGuiIO& io = ImGui::GetIO();
		if (action == GLFW_PRESS) 
		{
			io.MouseDown[button] = true;    
		} 
		else if (action == GLFW_RELEASE) 
		{
			io.MouseDown[button] = false;    
		}
	}
}

void ndDemoEntityManager::MouseScrollCallback(GLFWwindow* const, double, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += float (y);
}

void ndDemoEntityManager::CursorposCallback  (GLFWwindow* , double x, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
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

void ndDemoEntityManager::CharCallback(GLFWwindow*, ndUnsigned32 ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}

void ndDemoEntityManager::KeyCallback(GLFWwindow* const window, ndInt32 key, ndInt32, ndInt32 action, ndInt32 mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (action == GLFW_PRESS)
		io.KeysDown[key] = true;
	if (action == GLFW_RELEASE)
		io.KeysDown[key] = false;

	(void)mods; // Modifiers are not reliable across systems
	io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
	io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
	
	static ndInt32 prevKey;
	ndDemoEntityManager* const manager = (ndDemoEntityManager*)glfwGetWindowUserPointer(window);
	if ((key == GLFW_KEY_F10) && (key != prevKey)) 
	{
		manager->m_profilerMode = true;
	}

	if (key == GLFW_KEY_ESCAPE)
	{
		glfwSetWindowShouldClose (window, 1);
	}

	if (key == GLFW_KEY_F1) 
	{
		ndMatrix cameraMatrix(manager->GetCamera()->GetCurrentMatrix());
		manager->LoadDemo(manager->m_lastCurrentScene);
		manager->SetCameraMatrix(cameraMatrix, cameraMatrix.m_posit);
	}

	prevKey = io.KeysDown[key] ? key : 0;
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

bool ndDemoEntityManager::PollEvents()
{
	// Poll and handle events (inputs, window resize, etc.)
	// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
	// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
	// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
	// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
	glfwPollEvents();

	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return (w != 0) && (h != 0);
}

void ndDemoEntityManager::BeginFrame()
{
	ImGuiIO& io = ImGui::GetIO();

	// Setup display size (every frame to accommodate for window resizing)
	ndInt32 w, h;
	ndInt32 display_w, display_h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	glfwGetFramebufferSize(m_mainFrame, &display_w, &display_h);
	io.DisplaySize = ImVec2((ndReal)w, (ndReal)h);
	io.DisplayFramebufferScale = ImVec2(w > 0 ? ((ndReal)display_w / (ndReal)w) : 0, h > 0 ? ((ndReal)display_h / (ndReal)h) : 0);

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
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

	if (m_showUI && *m_renderDemoGUI)
	{
		if (ImGui::Begin("User Interface", &m_showUI))
		{
			m_renderDemoGUI->RenderHelp();
			ImGui::End();
		}
	}

	ShowMainMenuBar();
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

void ndDemoEntityManager::CreateSkyBox()
{
	if (!m_sky)
	{
		m_sky = new ndSkyBox(m_shaderCache.m_skyBox);
		//ndScopeSpinLock lock(m_addDeleteLock);
		//ndAssert(!m_sky->m_rootNode);
		//m_sky->m_rootNode = Addtop(m_sky);
	}
}

void ndDemoEntityManager::PushTransparentMesh (const ndDemoMeshInterface* const mesh, const ndMatrix& modelMatrix)
{
	ndVector dist (m_cameraManager->GetCamera()->GetInvViewMatrix().TransformVector(modelMatrix.m_posit));
	TransparentMesh entry (modelMatrix, (ndDemoMesh*) mesh);
	m_transparentHeap.Push (entry, dist.m_z);
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
	m_cameraManager->SetCameraMatrix(rotation, position);
}

void ndDemoEntityManager::UpdatePhysics(ndFloat32 timestep)
{
	// update the physics
	if (m_world && !m_suspendPhysicsUpdate) 
	{
		m_world->AdvanceTime(timestep);
	}
}

ndFloat32 ndDemoEntityManager::CalculateInteplationParam () const
{
	ndUnsigned64 timeStep = ndGetTimeInMicroseconds () - m_microsecunds;		
	ndFloat32 param = (ndFloat32 (timeStep) * MAX_PHYSICS_FPS) / 1.0e6f;
	ndAssert (param >= 0.0f);
	if (param > 1.0f) 
	{
		param = 1.0f;
	}
	return param;
}

void ndDemoEntityManager::RenderScene(ImDrawData* const)
{
	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	ImGuiIO& io = ImGui::GetIO();

	ndInt32 fb_width = (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 fb_height = (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
	{
		return;
	}

	ndDemoEntityManager* const window = (ndDemoEntityManager*)io.UserData;
	window->RenderScene();

	if (*window->m_renderDemoGUI) 
	{
		window->m_renderDemoGUI->RenderUI();
	}
}

void ndDemoEntityManager::SetAcceleratedUpdate()
{
	m_world->AccelerateUpdates();
}

void ndDemoEntityManager::RegisterPostUpdate(OnPostUpdate* const postUpdate)
{
	if (m_onPostUpdate)
	{
		delete m_onPostUpdate;
	}
	m_onPostUpdate = postUpdate;
}

void ndDemoEntityManager::OnSubStepPostUpdate(ndFloat32 timestep)
{
	if (m_colorRenderPass)
	{
		((ndColorRenderPass*)m_colorRenderPass)->UpdateDebugDisplay(timestep);
	}
}

void ndDemoEntityManager::RenderScene()
{
	D_TRACKTIME();
	ndFloat32 timestep = dGetElapsedSeconds();	
	CalculateFPS(timestep);
	UpdatePhysics(timestep);
	
	ImGuiIO& io = ImGui::GetIO();
	ndInt32 display_w = (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 display_h = (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);

	ndDemoCamera* const camera = GetCamera();
	camera->SetViewMatrix(display_w, display_h);

	// Get the interpolated location of each body in the scene
	ndFloat32 interpolateParam = CalculateInteplationParam();
	m_cameraManager->InterpolateMatrices (this, interpolateParam);

	// apply pre-render passes
	m_shadowRenderPass->RenderScene(timestep);
	m_colorRenderPass->RenderScene(timestep);
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
}

void ndDemoEntityManager::Run()
{
	// Main loop
	ndFloatExceptions exception;
	while (!glfwWindowShouldClose(m_mainFrame))
	{
		if (m_profilerMode)
		{
			ToggleProfiler();
			m_profilerMode = false;
		}

		m_suspendPhysicsUpdate = false;
		D_TRACKTIME();

		if (PollEvents())
		{
			BeginFrame();
			//TestImGui();
			RenderScene(ImGui::GetDrawData());

			glfwSwapBuffers(m_mainFrame);
		}
	}
}

