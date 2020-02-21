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
//#define PLAYER_JUMP_SPEED				5.8f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f

class dAnimationCache: public dTree<dAnimationSequence, dString>
{
	public:
	dAnimationCache()
	{
		LoadAnimation("whiteman_idle.anm");
		LoadAnimation("whiteman_walk.anm");
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

class AnimatedPlayerControllerManager: public dPlayerControllerManager
{
	public:
	AnimatedPlayerControllerManager(NewtonWorld* const world)
		:dPlayerControllerManager(world)
		,m_player(NULL)
		,m_animationCache()
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());

		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
	}

	~AnimatedPlayerControllerManager()
	{
	}

	void SetAsPlayer(dPlayerControllerOld* const controller)
	{
		m_player = controller;
	}

	void RenderPlayerHelp(DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "walk forward:            W");
		scene->Print(color, "walk backward:           S");
		scene->Print(color, "strafe right:            D");
		scene->Print(color, "strafe left:             A");
		//scene->Print(color, "toggle camera mode:      C");
		//scene->Print(color, "jump:                    Space");
		//scene->Print(color, "hide help:               H");
		//scene->Print(color, "change player direction: Left mouse button");
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		AnimatedPlayerControllerManager* const me = (AnimatedPlayerControllerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		AnimatedPlayerControllerManager* const me = (AnimatedPlayerControllerManager*)context;
		me->SetCamera();
	}

/*
	dAnimationKeyframesSequence* GetAnimationSequence(const char* const animName)
	{
		dTree<dAnimationKeyframesSequence*, dString>::dTreeNode* cachedAnimNode = m_animCache.Find(animName);
		if (!cachedAnimNode) {
			dScene scene(GetWorld());
			char pathName[2048];
			dGetWorkingFileName(animName, pathName);
			scene.Deserialize(pathName);

			dScene::dTreeNode* const animTakeNode = scene.FindChildByType(scene.GetRootNode(), dAnimationTake::GetRttiType());
			if (animTakeNode) {
				//dTree<dAnimimationKeyFramesTrack*, dString> map;
				//const dAnimPose& basePose = controller->GetBasePose();
				//dAnimTakeData* const animdata = new dAnimTakeData(basePose.GetCount());
				dAnimationKeyframesSequence* const animdata = new dAnimationKeyframesSequence();
				dAnimationTake* const animTake = (dAnimationTake*)scene.GetInfoFromNode(animTakeNode);
				animdata->SetPeriod(animTake->GetPeriod());

				cachedAnimNode = m_animCache.Insert(animdata, animName);

				//dList<dAnimimationKeyFramesTrack>& tracks = animdata->GetTracks();
				//dList<dAnimimationKeyFramesTrack>::dListNode* ptr = tracks.GetFirst();
				//dList<dAnimimationKeyFramesTrack>::dListNode* ptr = tracks.Append();
				//for (dAnimPose::dListNode* ptrNode = basePose.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
				//	DemoEntity* const entity = (DemoEntity*)ptrNode->GetInfo().m_userData;
				//	map.Insert(&ptr->GetInfo(), entity->GetName());
				//	ptr = ptr->GetNext();
				//}

				dList<dAnimimationKeyFramesTrack>& tracks = animdata->GetTracks();
				for (void* link = scene.GetFirstChildLink(animTakeNode); link; link = scene.GetNextChildLink(animTakeNode, link)) {
					dScene::dTreeNode* const node = scene.GetNodeFromLink(link);
					dAnimationTrack* const srcTrack = (dAnimationTrack*)scene.GetInfoFromNode(node);

					if (srcTrack->IsType(dAnimationTrack::GetRttiType())) {
						//dTree<dAnimimationKeyFramesTrack*, dString>::dTreeNode* const ptrNode = map.Find(srcTrack->GetName());
						//dAssert(ptrNode);
						//dAnimTakeData::dAnimTakeTrack* const dstTrack = ptrNode->GetInfo();
						dAnimimationKeyFramesTrack* const dstTrack = &tracks.Append()->GetInfo();
						dstTrack->SetName(srcTrack->GetName());

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
		dAssert(cachedAnimNode);
		return cachedAnimNode->GetInfo();
	}
*/
	
	dPlayerControllerOld* CreatePlayer(const dMatrix& location, dFloat height, dFloat radius, dFloat mass)
	{
		// get the scene 
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());

		// set the play coordinate system
		dMatrix localAxis(dGetIdentityMatrix());

		//up is first vector
		localAxis[0] = dVector(0.0, 1.0f, 0.0f, 0.0f);
		// up is the second vector
		localAxis[1] = dVector(1.0, 0.0f, 0.0f, 0.0f);
		// size if the cross product
		localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

		// make a play controller with default values.
		dPlayerControllerOld* const controller = CreateController(location, localAxis, mass, radius, height, 0.4f);

		// get body from player, and set some parameter
		NewtonBody* const body = controller->GetBody();
		
		DemoEntity* const playerEntity = DemoEntity::LoadNGD_mesh("whiteman.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(playerEntity);

		// set the user data
		NewtonBodySetUserData(body, playerEntity);

		// set the transform callback
		NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);

		// save player model with the controller
		controller->SetUserData(playerEntity);

		// set higher that 1.0f friction
		//controller->SetFriction(2.0f);
		//controller->SetFriction(1.0f);

		LoadAnimationBlendTree(controller);

		return controller;
	}

	void LoadAnimationBlendTree(dPlayerControllerOld* const controller)
	{
		//dAnimTakeData* const walkCycle = LoadAnimation(controller, "whiteman_idle.ngd");
		//dAnimationSequence* const idleSequence = LoadAnimation("whiteman_idle.ngd");
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

	void ApplyInputs(dPlayerControllerOld* const controller)
	{
		if (controller == m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
			dFloat forwarSpeed = (int(scene->GetKeyState('W')) - int(scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
			dFloat strafeSpeed = (int(scene->GetKeyState('D')) - int(scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;

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

	bool ProccessContact(dPlayerControllerOld* const controller, const dVector& position, const dVector& normal, const NewtonBody* const otherbody) const
	{
		/*
		if (normal.m_y < 0.9f) {
		dMatrix matrix;
		NewtonBodyGetMatrix(controller->GetBody(), &matrix[0][0]);
		dFloat h = (position - matrix.m_posit).DotProduct3(matrix.m_up);
		return (h >= m_stepHigh) ? true : false;
		}
		*/
		return true;
	}

	dFloat ContactFriction(dPlayerControllerOld* const controller, const dVector& position, const dVector& normal, int contactId, const NewtonBody* const otherbody) const
	{
		if (normal.m_y < 0.9f) {
			// steep slope are friction less
			return 0.0f;
		} else {
			//NewtonCollision* const collision = NewtonBodyGetCollision(otherbody);
			//int type = NewtonCollisionGetType (collision);
			//if ((type == SERIALIZE_ID_TREE) || (type == SERIALIZE_ID_TREE)) {
			//} else {
			switch (contactId) {
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

	// apply gravity 
	virtual void ApplyMove(dPlayerControllerOld* const controller, dFloat timestep)
	{
		// calculate the gravity contribution to the velocity
		dVector gravityImpulse(0.0f, DEMO_GRAVITY * controller->GetMass() * timestep, 0.0f, 0.0f);
		dVector totalImpulse(controller->GetImpulse() + gravityImpulse);
		controller->SetImpulse(totalImpulse);

		// apply play movement
		ApplyInputs(controller);
	}

	dPlayerControllerOld* m_player;
	dAnimationCache m_animationCache;
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
	AnimatedPlayerControllerManager* const playerManager = new AnimatedPlayerControllerManager(world);

	// add main player
	dMatrix location(dGetIdentityMatrix());
	location.m_posit.m_x = 30.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = -24.0f;

	location.m_posit.m_y = 15.0f;

	location.m_posit = FindFloor(scene->GetNewton(), location.m_posit, 20.0f);
	location.m_posit.m_y += 1.0f;
	dPlayerControllerOld*  const player = playerManager->CreatePlayer(location, 1.8f, 0.3f, 100.0f);
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

