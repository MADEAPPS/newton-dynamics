/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndDemoEntityManager.h"
#include "ndHighResolutionTimer.h"


#define MAX_PHYSICS_FPS				60.0f
#define MAX_PHYSICS_SUB_STEPS		2

#if 0
#include "SkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "DemoCamera.h"
#include "FileBrowser.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "TargaToOpenGl.h"
#include "ShaderPrograms.h"
#include "ndDemoEntityManager.h"
#include "DemoCameraManager.h"
#include "DemoCameraManager.h"


#ifdef _MACOSX_VER
	#include "CocoaOpenglGlue.h"
#endif

#define PROJECTILE_INITIAL_SPEED	20.0f

//#define DEFAULT_SCENE	0		// using NewtonMesh tool
//#define DEFAULT_SCENE	1		// coefficients of friction
//#define DEFAULT_SCENE	2		// coefficients of restitution
//#define DEFAULT_SCENE	3		// newton cradle
//#define DEFAULT_SCENE	4		// gyroscope precession
//#define DEFAULT_SCENE	5		// closest distance
//#define DEFAULT_SCENE	6		// primitive collision
//#define DEFAULT_SCENE	7		// kinematic bodies
//#define DEFAULT_SCENE	8		// Object Placement
//#define DEFAULT_SCENE	9		// primitive convex cast 
#define DEFAULT_SCENE	10		// box stacks
//#define DEFAULT_SCENE	11		// simple level mesh collision
//#define DEFAULT_SCENE	12		// optimized level mesh collision
//#define DEFAULT_SCENE	13		// height field Collision
//#define DEFAULT_SCENE	14		// infinite user plane collision
//#define DEFAULT_SCENE	15		// user height field Collision
//#define DEFAULT_SCENE	16		// compound Collision
//#define DEFAULT_SCENE	17		// simple Archimedes buoyancy
//#define DEFAULT_SCENE	18		// uniform Scaled Collision
//#define DEFAULT_SCENE	19		// non uniform Scaled Collision
//#define DEFAULT_SCENE	20		// scaled mesh collision
//#define DEFAULT_SCENE	21		// continuous collision
//#define DEFAULT_SCENE	22		// flatland Game
//#define DEFAULT_SCENE	23		// scene Collision
//#define DEFAULT_SCENE	24		// simple convex fracturing 
//#define DEFAULT_SCENE	25		// multi ray casting using the threading Job scheduler
//#define DEFAULT_SCENE	26		// standard joints
//#define DEFAULT_SCENE	27		// servo joints
//#define DEFAULT_SCENE	28		// construction vehicle
//#define DEFAULT_SCENE	29		// six axis manipulator
//#define DEFAULT_SCENE	30		// hexapod Robot
//#define DEFAULT_SCENE	31		// basic rag doll
//#define DEFAULT_SCENE	32		// balancing biped
//#define DEFAULT_SCENE	33		// multi body body vehicle
//#define DEFAULT_SCENE	34		// super Car
//#define DEFAULT_SCENE	35		// heavy vehicles
//#define DEFAULT_SCENE	36		// basic player controller
//#define DEFAULT_SCENE	37		// animated player controller
//#define DEFAULT_SCENE	38		// advanced player controller
//#define DEFAULT_SCENE	39		// cloth patch			
//#define DEFAULT_SCENE	40		// soft bodies	
						 
/// demos forward declaration 
void Friction (ndDemoEntityManager* const scene);
void Restitution (ndDemoEntityManager* const scene);
void NewtonCradle (ndDemoEntityManager* const scene);
void GyroscopyPrecession(ndDemoEntityManager* const scene);
void ClosestDistance (ndDemoEntityManager* const scene);
void ConvexCast (ndDemoEntityManager* const scene);
void PrimitiveCollision (ndDemoEntityManager* const scene);
void KinematicBodies (ndDemoEntityManager* const scene);
void ObjectPlacement (ndDemoEntityManager* const scene);
void ClothPatch(ndDemoEntityManager* const scene);
void SoftBodies (ndDemoEntityManager* const scene);
void BasicBoxStacks (ndDemoEntityManager* const scene);
void SimpleMeshLevelCollision (ndDemoEntityManager* const scene);
void OptimizedMeshLevelCollision (ndDemoEntityManager* const scene);
void UniformScaledCollision (ndDemoEntityManager* const scene);
void NonUniformScaledCollision (ndDemoEntityManager* const scene);
void ScaledMeshCollision (ndDemoEntityManager* const scene);
void ContinuousCollision (ndDemoEntityManager* const scene);
void FlatLandGame (ndDemoEntityManager* const scene);
void SceneCollision (ndDemoEntityManager* const scene);
void CompoundCollision(ndDemoEntityManager* const scene);
void AlchimedesBuoyancy(ndDemoEntityManager* const scene);
void SimpleConvexFracturing (ndDemoEntityManager* const scene);
void UsingNewtonMeshTool (ndDemoEntityManager* const scene);
void MultiRayCast (ndDemoEntityManager* const scene);
void MultibodyBodyCar(ndDemoEntityManager* const scene);
void SuperCar (ndDemoEntityManager* const scene);
void MilitaryTransport (ndDemoEntityManager* const scene);
void BasicPlayerController (ndDemoEntityManager* const scene);
void AnimatedPlayerController (ndDemoEntityManager* const scene);
void AdvancedPlayerController (ndDemoEntityManager* const scene);
void HeightFieldCollision (ndDemoEntityManager* const scene);
void UserPlaneCollision (ndDemoEntityManager* const scene);
void UserHeightFieldCollision (ndDemoEntityManager* const scene);
void PassiveRagdoll (ndDemoEntityManager* const scene);
void BalancingBiped (ndDemoEntityManager* const scene);
void ServoJoints (ndDemoEntityManager* const scene);
void ConstructionVehicle (ndDemoEntityManager* const scene);
void StandardJoints (ndDemoEntityManager* const scene);
void SixAxisManipulators(ndDemoEntityManager* const scene);
void Hexapod(ndDemoEntityManager* const scene);

ndDemoEntityManager::SDKDemos ndDemoEntityManager::m_demosSelection[] = 
{
	{"Using the newton mesh tool", "demonstrate how to use the newton mesh tool for mesh manipulation", UsingNewtonMeshTool},
	{"Coefficients of friction", "demonstrate the effect of various coefficient of friction", Friction},
	{"Coefficients of restitution", "demonstrate the effect of various coefficient of restitution", Restitution},
	{"Newton Cradle", "demonstrate the effect of various versus multiple bodes", NewtonCradle},
	{"Gyroscopic precession", "show natural precession", GyroscopyPrecession},
	{"Closest distance", "demonstrate closest distance to a convex shape", ClosestDistance},
	{"Primitive Collision", "demonstrate separate collision of primitives", PrimitiveCollision},
	{"Kinematic bodies", "demonstrate separate collision of primitives", KinematicBodies},
	{"Object Placement", "demonstrate separate collision of primitives", ObjectPlacement},
	{"Primitive convex cast", "demonstrate separate primitive convex cast", ConvexCast},
	{"Simple box Stacks", "show simple stack of Boxes", BasicBoxStacks},
	{"Unoptimized mesh collision", "show simple level mesh", SimpleMeshLevelCollision},
	{"Optimized mesh collision", "show optimized level mesh", OptimizedMeshLevelCollision},
	{"Height field collision mesh", "show high file collision mesh", HeightFieldCollision},
	{"User infinite Plane collision mesh", "show high file collision mesh", UserPlaneCollision},
	{"User Height field collision mesh", "show high file collision mesh", UserHeightFieldCollision},
	{"Compound collision shape", "demonstrate compound collision", CompoundCollision},
	{"Archimedes Buoyancy", "show Archimedes Buoyancy using the trigger volume manager", AlchimedesBuoyancy},
	{"Uniform scaled collision shape", "demonstrate scaling shape", UniformScaledCollision},
	{"Non uniform scaled collision shape", "demonstrate scaling shape", NonUniformScaledCollision},
	{"Scaled mesh collision", "demonstrate scaling mesh scaling collision", ScaledMeshCollision},
	{"Continuous collision", "show continuous collision", ContinuousCollision },
	{"Flat Land Game", "show usage of tow dimension joints", FlatLandGame },
	{"Multi geometry collision", "show static mesh with the ability of moving internal parts", SceneCollision},
	{"Simple convex fracture", "demonstrate simple fracture destruction using Voronoi partition", SimpleConvexFracturing},
	{"Parallel ray cast", "using the threading Job scheduler", MultiRayCast},
	{"Standard Joints", "show some of the common joints", StandardJoints},
	{"Servo actuators joints", "demonstrate complex array of bodies interconnect by joints", ServoJoints},
	{"Construction Vehicle", "demonstrate complex array of bodies interconnect by joints", ConstructionVehicle},
	{"Six axis manipulator", "show using inverse dynamics to control robots", SixAxisManipulators },
	{"Hexapod walker", "show using inverse dynamics to control robots", Hexapod },
	{"Passive rag doll", "demonstrate passive rag doll", PassiveRagdoll},
	{"Balancing  (WIP)", "demonstrate self balancing rag doll biped", BalancingBiped},
	{"Single body car", "show a generalized coordinate system body", MultibodyBodyCar },
	{"Super car", "implement a hight performance sport car", SuperCar},
	{"Heavy vehicles", "implement military type heavy Vehicles", MilitaryTransport},
	{"Basic player controller", "demonstrate simple player controller", BasicPlayerController},
	{"Animated player controller", "demonstrate inverse kinematic player controller", AnimatedPlayerController},
	{"Advanced player controller", "demonstrate player interacting with other objects", AdvancedPlayerController},
	{"Simple cloth Patch", "show simple cloth patch", ClothPatch},
	{"Simple soft Body", "show simple soft body", SoftBodies},
};

ndDemoEntityManager::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

int ndDemoEntityManager::ButtonKey::UpdateTrigger (bool triggerValue)
{
	m_memory0 = m_memory1;
	m_memory1 = triggerValue;
	return (!m_memory0 & m_memory1) ? 1 : 0;
}

int ndDemoEntityManager::ButtonKey::UpdatePushButton (bool triggerValue)
{
	if (UpdateTrigger (triggerValue)) {
		m_state = ! m_state;
	}
	return m_state ? 1 : 0;
}


// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.

DemoCamera* ndDemoEntityManager::GetCamera() const
{
	return m_cameraManager->GetCamera();
}


bool ndDemoEntityManager::GetKeyState(int key) const
{
	const ImGuiIO& io = ImGui::GetIO();
	return io.KeysDown[key];
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

bool ndDemoEntityManager::GetMouseKeyState (int button) const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.MouseDown[button];
}

void ndDemoEntityManager::Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context)
{
	m_renderDemoGUI = UIcallback;
	m_renderHelpMenus = helpCallback;
	m_renderUIContext = context;
}

void ndDemoEntityManager::SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context)
{
	m_updateCamera = callback;
	m_updateCameraContext = context;
}

int ndDemoEntityManager::GetJoystickAxis (dFloat32* const axisValues, int maxAxis) const
{
	int axisCount = 0;
	if (m_hasJoytick) {
		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
		axisCount = dMin (axisCount, maxAxis);
		for (int i = 0; i < axisCount; i ++) {
			axisValues[i] = axis[i];
		}
	}
	return axisCount;
}

int ndDemoEntityManager::GetJoystickButtons (char* const axisbuttons, int maxButton) const
{
	int buttonsCount = 0;
	if (m_hasJoytick) {
		const unsigned char* const buttons = glfwGetJoystickButtons(0, &buttonsCount);
		buttonsCount = dMin (buttonsCount, maxButton);
		for (int i = 0; i < buttonsCount; i ++) {
			axisbuttons[i] = buttons[i];
		}
	}
	return buttonsCount;
}

void ndDemoEntityManager::RemoveEntity (dListNode* const entNode)
{
	ndDemoEntity* const entity = entNode->GetInfo();
	entity->Release();
	Remove(entNode);
}

void ndDemoEntityManager::RemoveEntity (ndDemoEntity* const ent)
{
	dCustomScopeLock lock(&m_deleteLock);
	for (dListNode* node = dList<ndDemoEntity*>::GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo() == ent) {
			RemoveEntity (node);
			break;
		}
	}
}

void ndDemoEntityManager::ApplyMenuOptions()
{
	NewtonWaitForUpdateToFinish(m_world);

	// clean up all caches the engine have saved
	//NewtonInvalidateCache(m_world);
	NewtonSetNumberOfSubsteps (m_world, m_solverSubSteps);
	NewtonSetSolverIterations(m_world, m_solverPasses);
	NewtonSetThreadsCount(m_world, m_workerThreads);

	int state = m_autoSleepMode ? 1 : 0;
	for (const NewtonBody* body = NewtonWorldGetFirstBody(m_world); body; body = NewtonWorldGetNextBody(m_world, body)) {
		NewtonBodySetAutoSleep(body, state);
	}

	NewtonSelectBroadphaseAlgorithm(m_world, m_broadPhaseType);
	NewtonSetParallelSolverOnLargeIsland (m_world, m_solveLargeIslandInParallel ? 1 : 0);	

	void* plugin = nullptr;
	if (m_currentPlugin) {
		int index = 1;
		for (void* ptr = NewtonGetFirstPlugin(m_world); ptr; ptr = NewtonGetNextPlugin(m_world, ptr)) {
			if (index == m_currentPlugin) {
				plugin = ptr;
			}
			index ++;
		}
	}
	NewtonSelectPlugin(m_world, plugin);
}


void ndDemoEntityManager::LoadDemo(int menu)
{
	char newTitle[256];
	Cleanup();
	m_demosSelection[menu].m_launchDemoCallback(this);
	sprintf(newTitle, "Newton Game Dynamics 3.14 demo: %s", m_demosSelection[menu].m_name);
	glfwSetWindowTitle(m_mainFrame, newTitle);
	ApplyMenuOptions();
	ResetTimer();
}

bool ndDemoEntityManager::GetMousePosition (int& posX, int& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = int (posit.x);
	posY = int (posit.y);
	return true;
}

void ndDemoEntityManager::ToggleProfiler()
{
	#ifdef D_PROFILER
		dAssert(m_world);
		dTrace(("profiler Enable\n"));
		NewtonWaitForUpdateToFinish(m_world);
		m_profilerMode = !m_profilerMode;
		dProfilerEnableProling(m_profilerMode);
	#endif
}



void ndDemoEntityManager::CreateSkyBox()
{
	if (!m_sky) {
		m_sky = new SkyBox(m_shadeCache.m_solidColor);
		Append(m_sky);
	}
}


void ndDemoEntityManager::PushTransparentMesh (const ndDemoMeshInterface* const mesh)
{
	dMatrix matrix;
	glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);
	TransparentMesh entry (matrix, (ndDemoMesh*) mesh);
	m_tranparentHeap.Push (entry, matrix.m_posit.m_z);
}


void ndDemoEntityManager::LoadVisualScene(dScene* const scene, EntityDictionary& dictionary)
{
	// load all meshes into a Mesh cache for reuse
	dTree<ndDemoMeshInterface*, dScene::dTreeNode*> meshDictionary;
	for (dScene::dTreeNode* node = scene->GetFirstNode (); node; node = scene->GetNextNode (node)) {
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
			ndDemoMeshInterface* const mesh = new ndDemoMesh(scene, node, m_shadeCache);
			meshDictionary.Insert(mesh, node);
		}
	}

	// create an entity for every root node in the mesh
	// a root node or scene entity is a dSceneNodeInfo with a direct link to the root of the dScene node.
	dScene::dTreeNode* const root = scene->GetRootNode();
	for (void* child = scene->GetFirstChildLink(root); child; child = scene->GetNextChildLink (root, child)) {
		dScene::dTreeNode* node = scene->GetNodeFromLink(child);
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
			// we found a root dSceneNodeInfo, convert it to a Scene entity and load all it children 
			ndDemoEntity* const entityRoot = new ndDemoEntity (*this, scene, node, meshDictionary, dictionary);
			Append(entityRoot);
		}
	}

	// release all meshes before exiting
	dTree<ndDemoMeshInterface*, dScene::dTreeNode*>::Iterator iter (meshDictionary);
	for (iter.Begin(); iter; iter++) {
		ndDemoMeshInterface* const mesh = iter.GetNode()->GetInfo();
		mesh->Release();
	}
}

void ndDemoEntityManager::ImportPLYfile (const char* const fileName)
{
	m_collisionDisplayMode = 2;
	CreatePLYMesh (this, fileName, true);
}

void ndDemoEntityManager::LoadScene (const char* const fileName)
{
	dScene database (m_world);

	database.Deserialize(fileName);

	// this will apply all global the scale to the mesh
	database.FreezeScale();
	// this will apply all local scale and transform to the mesh
	//database.FreezePivot();

	// Load the Visual Scene
	EntityDictionary entDictionary;
	LoadVisualScene(&database, entDictionary);

	//Load the physics world
	dList<NewtonBody*> bodyList;
	database.SceneToNewtonWorld(m_world, bodyList);

	// bind every rigidBody loaded to the scene entity
	for (dList<NewtonBody*>::dListNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext()) {
		// find the user data and set to the visual entity in the scene
		NewtonBody* const body = bodyNode->GetInfo();
		dScene::dTreeNode* const sceneNode = (dScene::dTreeNode*)NewtonBodyGetUserData(body);
		ndDemoEntity* const entity = entDictionary.Find(sceneNode)->GetInfo();
		NewtonBodySetUserData(body, entity);

		// see if this body have some special setups
		dScene::dTreeNode* const node = database.FindChildByType(sceneNode, dRigidbodyNodeInfo::GetRttiType());
		dAssert (node);
		dRigidbodyNodeInfo* const bodyData = (dRigidbodyNodeInfo*) database.GetInfoFromNode(node);
		dVariable* bodyType = bodyData->FindVariable("rigidBodyType");

		// set the default call backs
		if (!bodyType || !strcmp (bodyType->GetString(), "default gravity")) {
			NewtonBodySetTransformCallback(body, ndDemoEntity::TransformCallback);
			NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
			NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);
		}
	}

	// clean up all caches the engine have saved
	NewtonInvalidateCache (m_world);
}


void ndDemoEntityManager::SerializeFile (void* const serializeHandle, const void* const buffer, int size)
{
	// check that each chunk is a multiple of 4 bytes, this is useful for easy little to big Indian conversion
	dAssert ((size & 0x03) == 0);
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

void ndDemoEntityManager::DeserializeFile (void* const serializeHandle, void* const buffer, int size)
{
	// check that each chunk is a multiple of 4 bytes, this is useful for easy little to big Indian conversion
	dAssert ((size & 0x03) == 0);
	size_t ret = fread (buffer, size, 1, (FILE*) serializeHandle);
	ret = 0;
}


void ndDemoEntityManager::BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	// here the use can save information of this body, ex:
	// a string naming the body,  
	// serialize the visual mesh, or save a link to the visual mesh
	// save labels for looking up the body call backs

	// for the demos I will simple write three stream to identify what body it is, the application can do anything
	const char* const bodyIndentification = "gravityBody\0\0\0\0";
	int size = (strlen (bodyIndentification) + 3) & -4;
	serializeCallback (serializeHandle, &size, sizeof (size));
	serializeCallback (serializeHandle, bodyIndentification, size);
}

void ndDemoEntityManager::BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle)
{
	int size;
	char bodyIndentification[256];

	deserializecallback (serializeHandle, &size, sizeof (size));
	deserializecallback (serializeHandle, bodyIndentification, size);

	// get the world and the scene form the world user data
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	ndDemoEntityManager* const scene = (ndDemoEntityManager*)NewtonWorldGetUserData(world);

	// here we attach a visual object to the entity, 
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	scene->Append (entity);

	NewtonBodySetUserData (body, entity);
	NewtonBodySetTransformCallback(body, ndDemoEntity::TransformCallback);
	NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
	NewtonCollision* const collision = NewtonBodyGetCollision(body);

	#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	if (NewtonCollisionGetType(collision) == SERIALIZE_ID_TREE) {
		NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
	}
	#endif

	//for visual mesh we will collision mesh and convert it to a visual mesh using NewtonMesh 
	dTree <ndDemoMeshInterface*, const void*>* const cache = (dTree <ndDemoMeshInterface*, const void*>*)bodyUserData;
	dTree <ndDemoMeshInterface*, const void*>::dTreeNode* node = cache->Find(NewtonCollisionDataPointer (collision));
	if (!node) {
		ndDemoMeshInterface* mesh = new ndDemoMesh(bodyIndentification, scene->m_shadeCache, collision, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga");
		node = cache->Insert(mesh, NewtonCollisionDataPointer (collision));
	} else {
		node->GetInfo()->AddRef();
	}

	ndDemoMeshInterface* const mesh = node->GetInfo();
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();
}

void ndDemoEntityManager::SerializedPhysicScene(const char* const name)
{
//	NewtonSerializeToFile(m_world, name, nullptr, nullptr);
	NewtonSerializeToFile(m_world, name, BodySerialization, nullptr);
}

void ndDemoEntityManager::DeserializedPhysicScene(const char* const name)
{
	// add the sky
	CreateSkyBox();

	dQuaternion rot;
	dVector origin(-30.0f, 10.0f, 10.0f, 0.0f);
	SetCameraMatrix(rot, origin);

	dTree <ndDemoMeshInterface*, const void*> cache;
	NewtonDeserializeFromFile(m_world, name, BodyDeserialization, &cache);
}

int ndDemoEntityManager::Print (const dVector& color, const char *fmt, ... ) const
{
	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );
	ImGui::Text(string, "");
	return 0;
}

void ndDemoEntityManager::OnCreateContact(const NewtonWorld* const world, NewtonJoint* const contact)
{
//	ndDemoEntityManager* const scene = (ndDemoEntityManager*) NewtonWorldGetUserData(world);
//	dCustomScopeLock lock(&scene->m_contactLock);
//	NewtonJointSetUserData(contact, scene->m_contactList.Append(contact));
}

void ndDemoEntityManager::OnDestroyContact(const NewtonWorld* const world, NewtonJoint* const contact)
{
//	ndDemoEntityManager* const scene = (ndDemoEntityManager*)NewtonWorldGetUserData(world);
//	dList<NewtonJoint*>::dListNode* const cooky = (dList<NewtonJoint*>::dListNode*)NewtonJointGetUserData(contact);
//	dCustomScopeLock lock(&scene->m_contactLock);
//	scene->m_contactList.Remove(cooky);
}


void ndDemoEntityManager::SetCameraMatrix (const dQuaternion& rotation, const dVector& position)
{
	m_cameraManager->SetCameraMatrix(this, rotation, position);
}


dFloat32 ndDemoEntityManager::CalculateInteplationParam () const
{
	dUnsigned64 timeStep = dGetTimeInMicrosenconds () - m_microsecunds;		
	dFloat32 param = (dFloat32 (timeStep) * MAX_PHYSICS_FPS) / 1.0e6f;
	dAssert (param >= 0.0f);
	if (param > 1.0f) {
		param = 1.0f;
	}
	return param;
}


void ndDemoEntityManager::PostUpdateCallback(const NewtonWorld* const world, dFloat32 timestep)
{
	ndDemoEntityManager* const scene = (ndDemoEntityManager*) NewtonWorldGetUserData(world);
	scene->m_cameraManager->FixUpdate(scene->GetNewton(), timestep);
	if (scene->m_updateCamera) {
		scene->m_updateCamera(scene, scene->m_updateCameraContext, timestep);
	}
}


#endif


ndDemoEntityManager::ndDemoEntityManager()
	:m_mainFrame(nullptr)
	//, m_defaultFont(0)
	//, m_sky(nullptr)
	//, m_world(nullptr)
	//, m_cameraManager(nullptr)
	//, m_renderUIContext(nullptr)
	//, m_updateCameraContext(nullptr)
	//, m_renderDemoGUI(nullptr)
	//, m_renderHelpMenus(nullptr)
	//, m_updateCamera(nullptr)
	//, m_microsecunds(0)
	//, m_tranparentHeap()
	//, m_currentScene(DEFAULT_SCENE)
	//, m_lastCurrentScene(DEFAULT_SCENE)
	//, m_framesCount(0)
	//, m_physicsFramesCount(0)
	//, m_currentPlugin(0)
	//, m_fps(0.0f)
	//, m_timestepAcc(0.0f)
	//, m_currentListenerTimestep(0.0f)
	//, m_mainThreadPhysicsTime(0.0f)
	//, m_mainThreadPhysicsTimeAcc(0.0f)
	//, m_broadPhaseType(0)
	//, m_debugDisplayMode(0)
	//, m_collisionDisplayMode(0)
	//, m_showUI(true)
	//, m_showAABB(false)
	//, m_showStats(true)
	//, m_autoSleepMode(true)
	//, m_hideVisualMeshes(false)
	//, m_showNormalForces(false)
	//, m_showCenterOfMass(false)
	//, m_showBodyFrame(false)
	//, m_showContactPoints(false)
	//, m_showJointDebugInfo(false)
	//, m_showListenersDebugInfo(false)
	//, m_showCollidingFaces(false)
	//, m_asynchronousPhysicsUpdate(false)
	//, m_solveLargeIslandInParallel(true)
	//, m_showRaycastHit(false)
	//, m_profilerMode(0)
	//, m_contactLock(0)
	//, m_deleteLock(0)
	//, m_contactList()
{
	// Setup window
	glfwSetErrorCallback(ErrorCallback);

	glfwInit();

	m_hasJoytick = glfwJoystickPresent(0) ? true : false;

	char version[256];
	sprintf(version, "Newton Dynamics %d.%.2i sandbox demos", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_mainFrame = glfwCreateWindow(1280, 720, version, nullptr, nullptr);
	glfwMakeContextCurrent(m_mainFrame);

	int monitorsCount;
	GLFWmonitor** monitors = glfwGetMonitors(&monitorsCount);
	if (monitorsCount > 1) 
	{
		int window_x;
		int window_y;
		int monitor_x;
		int monitor_y;

		glfwGetMonitorPos(monitors[1], &monitor_x, &monitor_y);
		glfwGetWindowPos(m_mainFrame, &window_x, &window_y);
		glfwSetWindowPos(m_mainFrame, monitor_x + window_x, monitor_y + 64);
	}

	// attach myself to the main frame
	glfwSetWindowUserPointer(m_mainFrame, this);

	// Setup ImGui binding
	//	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
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

	// Alternatively you can set this to nullptr and call ImGui::GetDrawData() after ImGui::Render() to get the same ImDrawData pointer.
	io.RenderDrawListsFn = RenderDrawListsCallback;
	//	io.SetClipboardTextFn = ImGui_ImplGlfw_SetClipboardText;
	//	io.GetClipboardTextFn = ImGui_ImplGlfw_GetClipboardText;

#ifdef _MSC_VER 
	io.ImeWindowHandle = glfwGetWin32Window(m_mainFrame);
#else 
	dTrace(("no sure what to set this to for non windews systems\n"))
		//	dAssert (0);
#endif

	glfwSwapInterval(0);
	glfwSetKeyCallback(m_mainFrame, KeyCallback);
	glfwSetCharCallback(m_mainFrame, CharCallback);
	glfwSetScrollCallback(m_mainFrame, MouseScrollCallback);
	glfwSetCursorPosCallback(m_mainFrame, CursorposCallback);
	glfwSetMouseButtonCallback(m_mainFrame, MouseButtonCallback);

	LoadFont();

	m_showUI = false;
	m_showAABB = false;
	m_showStats = true;
	m_autoSleepMode = false;
	m_updateMenuOptions = false;
	m_suspendPhysicsUpdate = false;
	m_mousePressed[0] = false;
	m_mousePressed[1] = false;
	m_mousePressed[2] = false;

	m_workerThreads = 1;
	m_solverPasses = 4;
	m_solverSubSteps = 2;

	m_fps = 0.0f;
	m_timestepAcc = 0.0f;
	m_framesCount = 0;
	m_renderDemoGUI = nullptr;

	// initialized the physics world for the new scene
	//	m_showUI = false;
	//	m_showAABB = false;
	//	m_hideVisualMeshes = true;
	//	m_autoSleepMode = false;
	//	m_broadPhaseType = 1;
	//	m_solverPasses = 4;
	//	m_workerThreads = 4;
	//	m_solverSubSteps = 2;
	//	m_showRaycastHit = true;
	//	m_showCenterOfMass = false;
	//	m_showNormalForces = true;
	//	m_showContactPoints = true;
	//	m_showJointDebugInfo = true;
	//	m_collisionDisplayMode = 2;
	//	m_showListenersDebugInfo = true;
	//  m_asynchronousPhysicsUpdate = true;

	m_microsecunds = 0;
	m_world = nullptr;
	Cleanup();
	ResetTimer();
	
	//m_currentPlugin = 0;
	//void* preferedPlugin = NewtonGetPreferedPlugin(m_world);
	//for (void* ptr = NewtonGetFirstPlugin(m_world); ptr; ptr = NewtonGetNextPlugin(m_world, ptr)) {
	//	m_currentPlugin++;
	//	if (ptr == preferedPlugin) {
	//		break;
	//	}
	//}
	//m_currentPlugin = 0;

	//m_shadeCache.CreateAllEffects();
}

ndDemoEntityManager::~ndDemoEntityManager()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) 
	{
		m_world->Sync();
	}

	Cleanup();
	
	// destroy the empty world
	if (m_world) 
	{
		delete m_world;
	}
	
	//if (m_cameraManager) 
	//{
	//	delete m_cameraManager;
	//}

	// Cleanup
	GLuint font_texture(m_defaultFont);
	glDeleteTextures(1, &font_texture);
	ImGui::GetIO().Fonts->TexID = 0;

	ImGui::Shutdown();
	glfwTerminate();
	//dAssert(NewtonGetMemoryUsed() == 0);
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

	dGetWorkingFileName(name, pathName);
	io.Fonts->AddFontFromFileTTF(pathName, pixedSize);
	//io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());

	// Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) 
	// because it is more likely to be compatible with user's existing shaders. 
	// If your ImTextureId represent a higher-level concept than just a GL texture id, 
	// consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
	unsigned char* pixels;
	int width, height;
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
	m_defaultFont = int(font_texture);
	io.Fonts->TexID = (void *)(intptr_t)m_defaultFont;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, last_texture);
}

void ndDemoEntityManager::ResetTimer()
{
	m_microsecunds = dGetTimeInMicrosenconds();
}

void ndDemoEntityManager::ErrorCallback(int error, const char* description)
{
	dTrace(("Error %d: %s\n", error, description));
	fprintf(stderr, "Error %d: %s\n", error, description);
	dAssert(0);
}

void ndDemoEntityManager::CursorposCallback(GLFWwindow*, double x, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
}

void ndDemoEntityManager::MouseScrollCallback(GLFWwindow* const window, double x, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += float(y);
}

void ndDemoEntityManager::CharCallback(GLFWwindow* window, unsigned int ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}

void ndDemoEntityManager::MouseButtonCallback(GLFWwindow*, int button, int action, int)
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

void ndDemoEntityManager::KeyCallback(GLFWwindow* const window, int key, int, int action, int mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (action == GLFW_PRESS)
	{
		io.KeysDown[key] = true;
	}
	if (action == GLFW_RELEASE)
	{
		io.KeysDown[key] = false;
	}

	(void)mods; // Modifiers are not reliable across systems
	io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
	io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];

	static int prevKey;
	//ndDemoEntityManager* const manager = (ndDemoEntityManager*)glfwGetWindowUserPointer(window);
	if ((key == GLFW_KEY_F10) && (key != prevKey)) 
	{
		dAssert(0);
		//manager->ToggleProfiler();
	}

	if (key == GLFW_KEY_ESCAPE) 
	{
		glfwSetWindowShouldClose(window, 1);
	}

	if (key == GLFW_KEY_F1) 
	{
		dAssert(0);
		//manager->LoadDemo(manager->m_lastCurrentScene);
	}

	prevKey = io.KeysDown[key] ? key : 0;
}

// This is the main rendering function that you have to implement and provide to ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or (0.375f,0.375f)
void ndDemoEntityManager::RenderDrawListsCallback(ImDrawData* const draw_data)
{
	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	ImGuiIO& io = ImGui::GetIO();

	int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
	{
		return;
	}

	ndDemoEntityManager* const window = (ndDemoEntityManager*)io.UserData;

	ImVec4 clearColor = ImColor(114, 144, 154);
	glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);

	window->RenderScene();

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_SCISSOR_TEST);
	glEnable(GL_TEXTURE_2D);
	//glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

	// Setup viewport, orthographic projection matrix
	glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	if (window->m_renderDemoGUI) 
	{
		dAssert(0);
	//	window->m_renderDemoGUI(window, window->m_renderUIContext);
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	// Render command lists
	draw_data->ScaleClipRects(io.DisplayFramebufferScale);

	#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
	for (int n = 0; n < draw_data->CmdListsCount; n++)
	{
		const ImDrawList* cmd_list = draw_data->CmdLists[n];
		const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
		const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;
		glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, pos)));
		glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, uv)));
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, col)));

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
			if (pcmd->UserCallback)
			{
				pcmd->UserCallback(cmd_list, pcmd);
			}
			else
			{
				glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
				glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
				glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
			}
			idx_buffer += pcmd->ElemCount;
		}
	}
	#undef OFFSETOF

	// Restore modified state
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

void ndDemoEntityManager::BeginFrame()
{
	glfwPollEvents();
	ImGuiIO& io = ImGui::GetIO();

	// Setup display size (every frame to accommodate for window resizing)
	int w, h;
	int display_w, display_h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	glfwGetFramebufferSize(m_mainFrame, &display_w, &display_h);
	io.DisplaySize = ImVec2((float)w, (float)h);
	io.DisplayFramebufferScale = ImVec2(w > 0 ? ((float)display_w / w) : 0, h > 0 ? ((float)display_h / h) : 0);

	// Start the frame
	ImGui::NewFrame();
}


void ndDemoEntityManager::Run()
{
	// Main loop
	while (!glfwWindowShouldClose(m_mainFrame))
	{
		//m_suspendPhysicsUpdate = false;
		BeginFrame();

		D_TRACKTIME();

		RenderStats();
		ImGui::Render();
		glfwSwapBuffers(m_mainFrame);
	}
}

void ndDemoEntityManager::CalculateFPS(dFloat32 timestep)
{
	m_framesCount++;
	m_timestepAcc += timestep;

	// this probably happing on loading of and a pause, just rest counters
	if ((m_timestepAcc <= 0.0f) || (m_timestepAcc > 2.0f)) 
	{
		m_timestepAcc = 0;
		m_framesCount = 0;
	}

	//update fps every quarter of a second
	if (m_timestepAcc >= 0.25f) 
	{
		m_fps = dFloat32(m_framesCount) / m_timestepAcc;
		m_timestepAcc -= 0.25f;
		m_timestepAcc -= 0.0f;
		m_framesCount = 0;
	}
}

void ndDemoEntityManager::ShowMainMenuBar()
{
	int mainMenu = 0;
	//dAssert (m_autoSleepMode);
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File")) 
		{
			m_suspendPhysicsUpdate = true;

			if (ImGui::MenuItem("Preferences", "")) 
			{
				dAssert(0);
			}
			ImGui::Separator();

			if (ImGui::MenuItem("New", "")) 
			{
				mainMenu = 1;
			}
			ImGui::Separator();

			if (ImGui::MenuItem("Open", "")) 
			{
				mainMenu = 2;
			}
			if (ImGui::MenuItem("Save", "")) 
			{
				mainMenu = 3;
			}
			ImGui::Separator();

			if (ImGui::MenuItem("Serialize", "")) 
			{
				mainMenu = 4;
			}
			if (ImGui::MenuItem("Deserialize", "")) 
			{
				mainMenu = 5;
			}

			ImGui::Separator();
			if (ImGui::MenuItem("import ply file", "")) 
			{
				mainMenu = 6;
			}

			ImGui::Separator();
			if (ImGui::MenuItem("Exit", "")) 
			{
				glfwSetWindowShouldClose(m_mainFrame, 1);
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Demos")) 
		{
			m_suspendPhysicsUpdate = true;
			dAssert(0);
			//int demosCount = int(sizeof(m_demosSelection) / sizeof m_demosSelection[0]);
			//for (int i = 0; i < demosCount; i++) 
			//{
			//	if (ImGui::MenuItem(m_demosSelection[i].m_name, "")) 
			//	{
			//		m_currentScene = i;
			//	}
			//}

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
			//ImGui::Checkbox("concurrent physics update", &m_asynchronousPhysicsUpdate);
			//ImGui::Checkbox("solve large island in parallel", &m_solveLargeIslandInParallel);
			ImGui::Separator();

			//int index = 0;
			//ImGui::RadioButton("default solver", &m_currentPlugin, index);
			//char ids[32][32];
			//for (void* plugin = NewtonGetFirstPlugin(m_world); plugin; plugin = NewtonGetNextPlugin(m_world, plugin)) 
			//{
			//	index++;
			//	const char* const id = NewtonGetPluginString(m_world, plugin);
			//	sprintf(&ids[index][0], "%s", id);
			//	ImGui::RadioButton(&ids[index][0], &m_currentPlugin, index);
			//}
			//ImGui::Separator();

			ImGui::SliderInt_DoubleSpace("solver sub steps", &m_solverSubSteps, 2, 8);
			ImGui::SliderInt_DoubleSpace("iterative solver passes", &m_solverPasses, 4, 64);

			//ImGui::Text("worker threads %d", m_workerThreads);
			ImGui::SliderInt_DoubleSpace("worker threads", &m_workerThreads, 1, 20);
			ImGui::Separator();

			//ImGui::RadioButton("default broad phase", &m_broadPhaseType, 0);
			//ImGui::RadioButton("persistence broad phase", &m_broadPhaseType, 1);
			ImGui::Separator();

			//ImGui::RadioButton("hide collision Mesh", &m_collisionDisplayMode, 0);
			//ImGui::RadioButton("show solid collision Mesh", &m_collisionDisplayMode, 1);
			//ImGui::RadioButton("show wire frame collision Mesh", &m_collisionDisplayMode, 2);
			ImGui::Separator();

			ImGui::Checkbox("show aabb", &m_showAABB);
			//ImGui::Checkbox("hide visual meshes", &m_hideVisualMeshes);
			//ImGui::Checkbox("show contact points", &m_showContactPoints);
			//ImGui::Checkbox("show ray cast hit point", &m_showRaycastHit);
			//ImGui::Checkbox("show normal forces", &m_showNormalForces);
			//ImGui::Checkbox("show center of mass", &m_showCenterOfMass);
			//ImGui::Checkbox("show body frame", &m_showBodyFrame);
			//ImGui::Checkbox("show joint debug info", &m_showJointDebugInfo);
			//ImGui::Checkbox("show listeners debug info", &m_showListenersDebugInfo);
			//ImGui::Checkbox("show colliding faces", &m_showCollidingFaces);

			ImGui::EndMenu();

			//SetDebugDisplayMode(m_showCollidingFaces ? 1 : 0);
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
			dAssert(0);
			//ApplyMenuOptions();
		}
	}

	switch (mainMenu)
	{
		case 1:
		{
			// menu new 
			dAssert(0);
			//Cleanup();
			//ApplyMenuOptions();
			//ResetTimer();
			//m_currentScene = -1;
			break;
		}

		case 2:
		{
			// open Scene
			dAssert(0);
			//m_currentScene = -1;
			//char fileName[1024];
			//Cleanup();
			//if (dGetOpenFileNameNgd(fileName, 1024)) {
			//	ApplyMenuOptions();
			//	LoadScene(fileName);
			//	ResetTimer();
			//}
			break;
		}

		case 3:
		{
			dAssert(0);
			//m_currentScene = -1;
			//char fileName[1024];
			//if (dGetSaveFileNameNgd(fileName, 1024)) {
			//	MakeViualMesh context(m_world);
			//	dScene testScene(m_world);
			//	testScene.NewtonWorldToScene(m_world, &context);
			//	testScene.Serialize(fileName);
			//}
			break;
		}

		case 4:
		{
			dAssert(0);
			//m_currentScene = -1;
			//char fileName[1024];
			//if (dGetSaveFileNameSerialization(fileName, 1024)) {
			//	SerializedPhysicScene(fileName);
			//}
			break;
		}

		case 5:
		{
			// open Scene
			dAssert(0);
			//m_currentScene = -1;
			//char fileName[1024];
			//Cleanup();
			//if (dGetOpenFileNameSerialization(fileName, 1024)) {
			//	ApplyMenuOptions();
			//	DeserializedPhysicScene(fileName);
			//	ResetTimer();
			//}
			break;
		}

		case 6:
		{
			// open Scene
			dAssert(0);
			//m_currentScene = -1;
			//char fileName[1024];
			//Cleanup();
			//if (dGetOpenFileNamePLY(fileName, 1024)) {
			//	ApplyMenuOptions();
			//	ImportPLYfile(fileName);
			//	ResetTimer();
			//}
			break;
		}

		default:
		{
			// load a demo 
			//dAssert(0);
			//if (m_currentScene != -1) 
			//{
			//	//DeserializedPhysicScene("C:/temp/test.bin");
			//	LoadDemo(m_currentScene);
			//	m_lastCurrentScene = m_currentScene;
			//	m_currentScene = -1;
			//}
		}
	}
}

void ndDemoEntityManager::RenderStats()
{
	if (m_showStats) 
	{
		char text[1024];

		if (ImGui::Begin("statistics", &m_showStats)) 
		{
			sprintf(text, "fps:           %6.3f", m_fps);
			ImGui::Text(text, "");

			sprintf(text, "physics time: %6.3f ms", m_mainThreadPhysicsTime * 1000.0f);
			ImGui::Text(text, "");

			sprintf(text, "memory used:   %d kbytes", int (dMemory::GetMemoryUsed() / 1024));
			ImGui::Text(text, "");

			//if (m_currentPlugin) 
			//{
			//	int index = 1;
			//	for (void* plugin = NewtonGetFirstPlugin(m_world); plugin; plugin = NewtonGetNextPlugin(m_world, plugin)) 
			//	{
			//		if (index == m_currentPlugin) {
			//			sprintf(text, "plugin:        %s", NewtonGetPluginString(m_world, plugin));
			//			ImGui::Text(text, "");
			//		}
			//		index++;
			//	}
			//}

			//sprintf(text, "bodies:        %d", NewtonWorldGetBodyCount(m_world));
			//ImGui::Text(text, "");
			//
			//sprintf(text, "threads:       %d", NewtonGetThreadsCount(m_world));
			//ImGui::Text(text, "");
			//
			//sprintf(text, "iterations:	%d", NewtonGetSolverIterations(m_world));
			//ImGui::Text(text, "");
			//
			//sprintf(text, "sub steps:     %d", NewtonGetNumberOfSubsteps(m_world));
			//ImGui::Text(text, "");
			//
			//m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0));
			ImGui::End();
		}
	}

	//if (m_showUI && m_renderHelpMenus) 
	//{
	//	if (ImGui::Begin("User Interface", &m_showUI)) 
	//	{
	//		m_renderHelpMenus(this, m_renderUIContext);
	//		//m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0));  
	//		ImGui::End();
	//	}
	//}

	ShowMainMenuBar();
}

void ndDemoEntityManager::Cleanup()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) 
	{
		m_world->Sync();
	}

	// destroy all remaining visual objects
	//while (dList<ndDemoEntity*>::GetFirst()) 
	//{
	//	RemoveEntity(dList<ndDemoEntity*>::GetFirst());
	//}
	//if (m_cameraManager) 
	//{
	//	delete m_cameraManager;
	//}
	//m_sky = nullptr;
	//m_updateCamera = nullptr;

	// destroy the Newton world
	if (m_world) 
	{
		// get serialization call back before destroying the world
		delete m_world;
	}

	// create the newton world
	m_world = new ndWorld();

	//// link the work with this user data
	//NewtonWorldSetUserData(m_world, this);
	//
	//// set a post update callback which is call after all simulation and all listeners updates
	//NewtonSetPostUpdateCallback(m_world, PostUpdateCallback);
	//
	//// set joint serialization call back
	//dCustomJoint::Initalize(m_world);
	//
	//// add the camera manager
	//m_cameraManager = new DemoCameraManager(this);
	//
	//ApplyMenuOptions();
	//
	//// Set the Newton world user data
	//NewtonWorldSetUserData(m_world, this);
	//
	//// set the number of sub steps
	//NewtonSetNumberOfSubsteps(m_world, MAX_PHYSICS_SUB_STEPS);
	//
	//// register contact creation destruction callbacks
	//NewtonWorldSetCreateDestroyContactCallback(m_world, OnCreateContact, OnDestroyContact);
	//
	//// load all available plug ins
	//char plugInPath[2048];
	//plugInPath[0] = 0;
	//#if defined (_MSC_VER)
	//	GetModuleFileNameA(nullptr, plugInPath, 256);
	//#endif
	//
	//for (int i = int(strlen(plugInPath) - 1); i >= 0; i--) {
	//	if ((plugInPath[i] == '\\') || (plugInPath[i] == '/')) {
	//		plugInPath[i] = 0;
	//		break;
	//	}
	//}

#ifdef _DEBUG
	//#ifdef __linux__
	//	strcat(plugInPath, "newtonPlugins");
	//#else
	//	#ifdef _NEWTON_USE_DOUBLE
	//		strcat(plugInPath, "/newtonPlugins/debug_double");
	//	#else
	//		strcat(plugInPath, "/newtonPlugins/debug");
	//	#endif
	//#endif
#else
	#ifdef __linux__
		strcat(plugInPath, "newtonPlugins");
	#else
		#ifdef _NEWTON_USE_DOUBLE
			strcat(plugInPath, "/newtonPlugins/release_double");
		#else
			strcat(plugInPath, "/newtonPlugins/release");
		#endif
	#endif
#endif
	//NewtonLoadPlugins(m_world, plugInPath);

	// we start without 2d render
	m_renderDemoGUI = nullptr;
	//m_renderHelpMenus = nullptr;
	//m_renderUIContext = nullptr;
}

void ndDemoEntityManager::UpdatePhysics(dFloat32 timestep)
{
#if 0
	// update the physics
	if (m_world && !m_suspendPhysicsUpdate) 
	{
		D_TRACKTIME();

		dFloat32 timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
		dUnsigned64 timestepMicrosecunds = dUnsigned64(timestepInSecunds * 1000000.0f);

		dUnsigned64 currentTime = dGetTimeInMicrosenconds();
		dUnsigned64 nextTime = currentTime - m_microsecunds;
		if (nextTime > timestepMicrosecunds * 2) 
		{
			m_microsecunds = currentTime - timestepMicrosecunds * 2;
			nextTime = currentTime - m_microsecunds;
		}

		bool newUpdate = false;
		dFloat32 physicsTime = 0.0f;
		//while (nextTime >= timestepMicrosecunds) 
		if (nextTime >= timestepMicrosecunds)
		{
			newUpdate = true;
			ClearDebugDisplay(m_world);

#ifdef DEMO_CHECK_ASYN_UPDATE
			g_checkAsyncUpdate = 1;
#endif
			if (m_asynchronousPhysicsUpdate) 
			{
				NewtonUpdateAsync(m_world, timestepInSecunds);
#ifdef DEMO_CHECK_ASYN_UPDATE
				NewtonWaitForUpdateToFinish(m_world);
				g_checkAsyncUpdate = 0;
#endif
			}
			else 
			{
				NewtonUpdate(m_world, timestepInSecunds);
			}

			physicsTime += NewtonGetLastUpdateTime(m_world);

			nextTime -= timestepMicrosecunds;
			m_microsecunds += timestepMicrosecunds;
		}

		if (newUpdate) 
		{
			m_physicsFramesCount++;
			m_mainThreadPhysicsTimeAcc += physicsTime;
			if (m_physicsFramesCount >= 16) 
			{
				m_mainThreadPhysicsTime = m_mainThreadPhysicsTimeAcc / m_physicsFramesCount;
				m_physicsFramesCount = 0;
				m_mainThreadPhysicsTimeAcc = 0.0f;
			}
		}

		//dTrace (("%f\n", m_mainThreadPhysicsTime));
	}
#endif
}

void ndDemoEntityManager::RenderScene()
{
	dFloat32 timestep = dGetElapsedSeconds();
	CalculateFPS(timestep);
	UpdatePhysics(timestep);

	D_TRACKTIME();
	// Get the interpolated location of each body in the scene
//	m_cameraManager->InterpolateMatrices(this, CalculateInteplationParam());

	ImGuiIO& io = ImGui::GetIO();
	int display_w = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	int display_h = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	glViewport(0, 0, display_w, display_h);
	glScissor(0, 0, display_w, display_h);
	glEnable(GL_SCISSOR_TEST);

	// Rendering
	// Our shading model--Goraud (smooth). 
	glShadeModel(GL_SMOOTH);

	// Culling. 
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);

	//	glEnable(GL_DITHER);
	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

	// set default lightning
	glEnable(GL_LIGHTING);

	dFloat32 cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialParam(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialParam(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// set just one directional light
	GLfloat lightDiffuse0[] = { 0.8f, 0.8f, 0.8f, 0.0f };
	GLfloat lightAmbient0[] = { 0.2f, 0.2f, 0.2f, 0.0f };
	GLfloat lightSpecular0[] = { 1.0f, 1.0f, 1.0f, 0.0f };
	GLfloat lightPosition0[] = { 0.0f, 200.0f, 150.0f, 0.0f };

	glMaterialf(GL_FRONT, GL_SHININESS, 60.0f);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular0);
	glEnable(GL_LIGHT0);

	// one light from the Camera eye point
//	dVector camPosition(m_cameraManager->GetCamera()->m_matrix.m_posit);
//	GLfloat lightDiffuse1[] = { 0.5f, 0.5f, 0.5f, 0.0f };
//	GLfloat lightAmbient1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
//	GLfloat lightSpecular1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
//	GLfloat lightPosition1[] = { 0.0f, 0.0f, 0.0f, 1.0f };
//
//	glMaterialf(GL_FRONT, GL_SHININESS, 60.0f);
//	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
//	glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
//	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
//	glLightfv(GL_LIGHT1, GL_SPECULAR, lightSpecular1);
//	glEnable(GL_LIGHT1);

#if 0
	// Setup matrix
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	//glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// make sure the model view matrix is set to identity before setting world space light sources
	// update Camera
//	m_cameraManager->GetCamera()->SetViewMatrix(display_w, display_h);

	// render all entities

	dMatrix globalMatrix(dGetIdentityMatrix());
	if (m_hideVisualMeshes) 
	{
		if (m_sky) 
		{
			glPushMatrix();
			m_sky->Render(timestep, this, globalMatrix);
			glPopMatrix();
		}
	}
	else 
	{
		for (dListNode* node = dList<DemoEntity*>::GetFirst(); node; node = node->GetNext()) 
		{
			DemoEntity* const entity = node->GetInfo();
			glPushMatrix();
			entity->Render(timestep, this, globalMatrix);
			glPopMatrix();
		}
	}

	DEBUG_DRAW_MODE mode = m_solid;
	if (m_collisionDisplayMode) 
	{
		mode = (m_collisionDisplayMode == 1) ? m_solid : m_lines;
		DebugRenderWorldCollision(m_world, mode);
	}

	if (m_showAABB) 
	{
		RenderAABB(m_world);
	}

	if (m_showContactPoints) 
	{
		RenderContactPoints(m_world);
	}

	if (m_showRaycastHit) 
	{
		RenderRayCastHit(m_world);
	}

	if (m_showBodyFrame) 
	{
		RenderBodyFrame(m_world);
	}

	if (m_showCenterOfMass) 
	{
		RenderCenterOfMass(m_world);
	}

	if (m_showListenersDebugInfo) 
	{
		dJointDebugDisplay listenerDebugRender(m_cameraManager->GetCamera()->GetCurrentMatrix());
		listenerDebugRender.SetScale(0.5f);
		RenderListenersDebugInfo(m_world, &listenerDebugRender);
	}

	if (m_showJointDebugInfo) 
	{
		dJointDebugDisplay jointDebugRender(m_cameraManager->GetCamera()->GetCurrentMatrix());
		//jointDebugRender.SetScale(0.2f);
		jointDebugRender.SetScale(1.0f);

		RenderJointsDebugInfo(m_world, &jointDebugRender);
	}

	if (m_showNormalForces) 
	{
		RenderNormalForces(m_world);
	}

	if (m_tranparentHeap.GetCount()) 
	{
		glPushMatrix();
		while (m_tranparentHeap.GetCount()) 
		{
			const TransparentMesh& transparentMesh = m_tranparentHeap[0];
			glLoadIdentity();
			glLoadMatrix(&transparentMesh.m_matrix[0][0]);
			transparentMesh.m_mesh->RenderTransparency();
			m_tranparentHeap.Pop();
		}
		glPopMatrix();
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
#endif
}
