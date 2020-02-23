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

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


#define PLAYER_MASS						80.0f
#define PLAYER_WALK_SPEED				1.8f
#define PLAYER_JUMP_SPEED				5.8f
#define PLAYER_THIRD_PERSON_VIEW_DIST	6.0f

class dAnimationCache: public dTree<dAnimationSequence, dString>
{
	public:
	dAnimationCache()
	{
		LoadAnimation("whiteman_idle.anm");
		LoadAnimation("whiteman_walk.anm");
		LoadAnimation("whiteman_run.anm");
	}

	private:
	void LoadAnimation(const char* const name)
	{
		char fileName[1024];
		dGetWorkingFileName(name, fileName);
		dTree<dAnimationSequence, dString>::dTreeNode* node = Insert(name);
		node->GetInfo().Load(fileName);
	}
};


class dAnimatedPlayerController: public dPlayerController
{
	public:
	dAnimatedPlayerController(DemoEntity* const playerEntity, const dAnimationCache& animationcache, NewtonWorld* const world, const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height, dFloat stepHeight)
		:dPlayerController(world, location, localAxis, mass, radius, height, stepHeight)
		,m_animBlendTree(NULL)
		,m_poseCount(0)
	{
		// get body from player, and set some parameter
		NewtonBody* const body = GetBody();

		// set the user data
		NewtonBodySetUserData(body, playerEntity);

		// set the transform callback
		NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);

		// save player model with the controller
		SetUserData(playerEntity);

		// bind skeleton to animation blend tree, using idle sequnce
		BindPose(animationcache);

		CreateAnimationBlendTree(animationcache);
	}

	~dAnimatedPlayerController()
	{
		if (m_animBlendTree) {
			delete m_animBlendTree;
		}
	}

	void BindPose(const dAnimationCache& animationcache)
	{
		dAnimationSequence* const sequence =(dAnimationSequence*) &animationcache.Find("whiteman_idle.anm")->GetInfo();

		int count = 0;
		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(GetBody());
		const dList<dAnimimationKeyFramesTrack>& tracks = sequence->GetTracks();

		m_output.Resize(tracks.GetCount());
		for (dList<dAnimimationKeyFramesTrack>::dListNode* node = tracks.GetFirst(); node; node = node->GetNext()) {
			dAnimimationKeyFramesTrack& track = node->GetInfo();
			DemoEntity* const ent = entity->Find(track.GetName().GetStr());
			m_output[count].m_userData = ent;
			count++;
		}
		m_poseCount = count;
		dAssert (m_poseCount == m_output.GetSize());
	}

	void CreateAnimationBlendTree(const dAnimationCache& animationcache)
	{
		//dAnimationSequence* const idleSequence =(dAnimationSequence*) &animationcache.Find("whiteman_idle.anm")->GetInfo();
		dAnimationSequence* const walkSequence =(dAnimationSequence*) &animationcache.Find("whiteman_walk.anm")->GetInfo();
		dAnimationSequence* const runSequence =(dAnimationSequence*) &animationcache.Find("whiteman_run.anm")->GetInfo();

		//dAnimationSequencePlayer* const Idle = new dAnimationSequencePlayer(idleSequence);
		dAnimationSequencePlayer* const walk = new dAnimationSequencePlayer(walkSequence);
		dAnimationSequencePlayer* const run = new dAnimationSequencePlayer(runSequence);

		dAnimationTwoWayBlend* const walkRunBlend = new dAnimationTwoWayBlend (walk, run);
		walkRunBlend->SetParam(0.0f);

		m_animBlendTree = walkRunBlend;
	}

	void ApplyMove(dFloat timestep)
	{
		// calculate the gravity contribution to the velocity
		dFloat g = 2.0f * DEMO_GRAVITY;
		dVector gravity(GetLocalFrame().RotateVector(dVector(g, 0.0f, 0.0f, 0.0f)));
		dVector totalImpulse(GetImpulse() + gravity.Scale(GetMass() * timestep));
		SetImpulse(totalImpulse);
	}

	dFloat ContactFrictionCallback(const dVector& position, const dVector& normal, int contactId, const NewtonBody* const otherbody) const
	{
		if (normal.m_y < 0.9f) {
			// steep slope are friction less
			return 0.0f;
		} else {
			switch (contactId) 
			{
				case 1:
					// this the brick wall
					return 0.5f;
				case 2:
					// this the wood floor
					return 1.0f;
				case 3:
					// this the cement floor
					return 2.0f;
				default:
					// this is everything else
					return 1.0f;
			}
		}
	}

	void PostUpdate(dFloat timestep)
	{
		// update the animation tree
		m_animBlendTree->Evaluate(m_output, timestep);

		for (int i = 0; i < m_poseCount; i ++) {
		//for (int i = 0; i < 9; i ++) {
			const dAnimKeyframe& keyFrame = m_output[i];
			DemoEntity* const entity = (DemoEntity*)keyFrame.m_userData;

			//dTrace (("%s\n", entity->GetName().GetStr()));
			//dMatrix matrix (keyFrame.m_rotation, keyFrame.m_posit);
			//entity->SetMatrix(keyFrame.m_rotation, keyFrame.m_posit);
			entity->SetMatrixUsafe(keyFrame.m_rotation, keyFrame.m_posit);
		}
	}

	dAnimationPose m_output;
	dAnimationBlendTreeNode* m_animBlendTree;
	int m_poseCount;
};


class dAnimatedPlayerControllerManager: public dVehicleManager
{
	public:
	dAnimatedPlayerControllerManager(NewtonWorld* const world)
		:dVehicleManager(world)
		,m_animationCache()
		,m_player(NULL)
		,m_crowchKey(false)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
	}

	~dAnimatedPlayerControllerManager()
	{
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		dAnimatedPlayerControllerManager* const me = (dAnimatedPlayerControllerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		dAnimatedPlayerControllerManager* const me = (dAnimatedPlayerControllerManager*)context;
		me->SetCamera();
	}

	void RenderPlayerHelp(DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "walk forward:            W");
		scene->Print(color, "walk backward:           S");
		scene->Print(color, "strafe right:            D");
		scene->Print(color, "strafe left:             A");
		scene->Print(color, "crouch:				  C");
		scene->Print(color, "jump:                    Space");
		//scene->Print(color, "toggle camera mode:    C");
		//scene->Print(color, "hide help:             H");
	}

	void SetAsPlayer(dPlayerController* const controller)
	{
		m_player = controller;
	}

	void SetCamera()
	{
		if (m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix(camera->GetNextMatrix());

			DemoEntity* player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
			dMatrix playerMatrix(player->GetNextMatrix());

			dFloat height = 2.0f;
			dVector frontDir(camMatrix[0]);
			dVector upDir(0.0f, 1.0f, 0.0f, 0.0f);
			dVector camOrigin = playerMatrix.TransformVector(upDir.Scale(height));
			camOrigin -= frontDir.Scale(PLAYER_THIRD_PERSON_VIEW_DIST);

			camera->SetNextMatrix(*scene, camMatrix, camOrigin);
		}
	}

	void ApplyInputs(dVehicle* const model, dFloat timestep)
	{
		dAnimatedPlayerController* controller = (dAnimatedPlayerController*)model->GetAsPlayerController();
		dAssert(controller);
		controller->ApplyMove(timestep);

		if (controller == m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
			dFloat forwarSpeed = (int(scene->GetKeyState('W')) - int(scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
			dFloat strafeSpeed = (int(scene->GetKeyState('D')) - int(scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;

			bool crowchKey = scene->GetKeyState('C') ? true : false;
			if (m_crowchKey.UpdateTrigger(crowchKey)) {
				dAssert (0);
				//controller->ToggleCrouch();
				//DemoEntity* const playerEntity = (DemoEntity*)NewtonBodyGetUserData(controller->GetBody());
				//if (controller->IsCrouched()) {
				//	playerEntity->SetMesh(m_crouchMesh, dGetIdentityMatrix());
				//} else {
				//	playerEntity->SetMesh(m_standingMesh, dGetIdentityMatrix());
				//}
			}

			if (scene->GetKeyState(' ') && controller->IsOnFloor()) {
				dVector jumpImpule(controller->GetLocalFrame().RotateVector(dVector(PLAYER_JUMP_SPEED * controller->GetMass(), 0.0f, 0.0f, 0.0f)));
				dVector totalImpulse(controller->GetImpulse() + jumpImpule);
				controller->SetImpulse(totalImpulse);
			}

			if (forwarSpeed && strafeSpeed) {
				dFloat invMag = PLAYER_WALK_SPEED / dSqrt(forwarSpeed * forwarSpeed + strafeSpeed * strafeSpeed);
				forwarSpeed *= invMag;
				strafeSpeed *= invMag;
			}

			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix(camera->GetNextMatrix());

			controller->SetForwardSpeed(forwarSpeed);
			controller->SetLateralSpeed(strafeSpeed);
			controller->SetHeadingAngle(camera->GetYawAngle());
		}
	}

	dAnimatedPlayerController* CreatePlayer(const dMatrix& location, dFloat height, dFloat radius, dFloat mass)
	{
		// get the scene 
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		NewtonWorld* const world = scene->GetNewton();

		// set the play coordinate system
		dMatrix localAxis(dGetIdentityMatrix());

		//up is first vector
		localAxis[0] = dVector(0.0, 1.0f, 0.0f, 0.0f);
		// up is the second vector
		localAxis[1] = dVector(1.0, 0.0f, 0.0f, 0.0f);
		// size if the cross product
		localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);


		DemoEntity* const playerEntity = DemoEntity::LoadNGD_mesh("whiteman.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(playerEntity);

		// make a play controller with default values.
		dAnimatedPlayerController* const controller = new dAnimatedPlayerController(playerEntity, m_animationCache, world, location, localAxis, mass, radius, height, height / 3.0f);
		AddRoot(controller);

		return controller;
	}

	dAnimationCache m_animationCache;
	dPlayerController* m_player;
	DemoEntityManager::ButtonKey m_crowchKey;
};

static NewtonBody* CreateCylinder(DemoEntityManager* const scene, const dVector& location, dFloat mass, dFloat radius, dFloat height)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID = NewtonMaterialGetDefaultGroupID(world);
	dVector size(radius, height, radius, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dMatrix matrix(dRollMatrix (90.0f * dDegreeToRad));
	matrix.m_posit = location;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, matrix, collision, materialID);

	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}

static void AddMerryGoRound(DemoEntityManager* const scene, const dVector& location)
{
	NewtonBody* const pole = CreateCylinder(scene, location, 0.0f, 0.2f, 3.0f);

	dVector platformPosit(location);
	platformPosit.m_y += 0.2f;
	NewtonBody* const platform = CreateCylinder(scene, platformPosit, 200.0f, 10.0f, 0.2f);

	dMatrix pivot;
	NewtonBodyGetMatrix(platform, &pivot[0][0]);
	dCustomHinge* const hinge = new dCustomHinge(pivot, platform, pole);
	hinge;
}

static void CreateBridge(DemoEntityManager* const scene, NewtonBody* const playgroundBody)
{
	dVector p0(1.35f, 8.35f, -28.1f, 0.0f);
	dVector p1(1.35f, 8.40f, 28.9f, 0.0f);
	dVector p2(1.35f, 6.0f, 0.0, 0.0f);

	dFloat y[3];
	dFloat splineMatrix[3][3];

	y[0] = p0.m_y;
	splineMatrix[0][0] = p0.m_z * p0.m_z;
	splineMatrix[0][1] = p0.m_z;
	splineMatrix[0][2] = 1.0f;

	y[1] = p1.m_y;
	splineMatrix[1][0] = p1.m_z * p1.m_z;
	splineMatrix[1][1] = p1.m_z;
	splineMatrix[1][2] = 1.0f;

	y[2] = p2.m_y;
	splineMatrix[2][0] = p2.m_z * p2.m_z;
	splineMatrix[2][1] = p2.m_z;
	splineMatrix[2][2] = 1.0f;

	dSolveGaussian(3, &splineMatrix[0][0], y);

	dFloat plankLentgh = 3.0f;
	NewtonWorld* const world = scene->GetNewton();
	dVector size(8.0f, 0.5f, plankLentgh, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	int count = 0;
	dFloat mass = 10.0f;
	dFloat lenght = 0.0f;
	dFloat step = 1.0e-3f;
	dFloat y0 = y[0] * p0.m_z * p0.m_z + y[1] * p0.m_z + y[2];

	dVector q0(p0);
	NewtonBody* array[256];
	for (dFloat z = p0.m_z + step; z < p1.m_z; z += step) {
		dFloat y1 = y[0] * z * z + y[1] * z + y[2];
		dFloat y10 = y1 - y0;
		lenght += dSqrt(step * step + y10 * y10);
		if (lenght >= plankLentgh) {
			dVector q1(p0.m_x, y1, z, 0.0f);

			dMatrix matrix(dGetIdentityMatrix());
			matrix.m_posit = (q1 + q0).Scale(0.5f);
			matrix.m_posit.m_w = 1.0f;

			dVector right(q1 - q0);
			matrix.m_right = right.Normalize();
			matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);

			array[count] = CreateSimpleSolid(scene, geometry, mass, matrix, collision, 0);

			q0 = q1;
			lenght = 0.0f;
			count++;
		}

		y0 = y1;
	}

	dMatrix matrix;
	NewtonBodyGetMatrix(array[0], &matrix[0][0]);
	matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) - matrix.m_right.Scale(size.m_z * 0.5f);
	dCustomHinge* hinge = new dCustomHinge(matrix, array[0], playgroundBody);
	hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

	for (int i = 1; i < count; i++) {
		dMatrix matrix;
		NewtonBodyGetMatrix(array[i], &matrix[0][0]);
		matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) - matrix.m_right.Scale(size.m_z * 0.5f);
		dCustomHinge* const hinge = new dCustomHinge(matrix, array[i - 1], array[i]);
		hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);
	}

	NewtonBodyGetMatrix(array[count - 1], &matrix[0][0]);
	matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) + matrix.m_right.Scale(size.m_z * 0.5f);
	hinge = new dCustomHinge(matrix, array[count - 1], playgroundBody);
	hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

	geometry->Release();
	NewtonDestroyCollision(collision);
}

void AnimatedPlayerController(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	NewtonBody* const playgroundBody = CreateLevelMesh (scene, "playerarena.ngd", true);

	// create a character controller manager
	dAnimatedPlayerControllerManager* const playerManager = new dAnimatedPlayerControllerManager(world);

	// add main player
	dMatrix location(dGetIdentityMatrix());
	location.m_posit.m_x = 30.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = -24.0f;

	location.m_posit.m_y = 15.0f;

	location.m_posit = FindFloor(scene->GetNewton(), location.m_posit, 20.0f);
	location.m_posit.m_y += 1.0f;

	dAnimatedPlayerController* const player = playerManager->CreatePlayer(location, 1.8f, 0.3f, 100.0f);
	playerManager->SetAsPlayer(player);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	location.m_posit.m_x += 5.0f;

	int count = 1;
	dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	AddPrimitiveArray(scene, 100.0f, location.m_posit, dVector (2.0f, 2.0f, 2.0f, 0.0f), count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);

	// add some objects to interact with
	AddMerryGoRound(scene, dVector(40.0f, 0.0f, -15.0f, 0.0f));

	// add a hanging bridge
	CreateBridge(scene, playgroundBody);

	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

