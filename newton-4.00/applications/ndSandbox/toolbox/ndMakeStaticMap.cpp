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

#if 0
static ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const ndMatrix& location, ndFloat32 mass)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	const char* const textName = "wood_1.png";
	ndMatrix matrix(FindFloor(*world, location, shape, 200.0f));
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, textName, textName, textName));

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(mesh);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}



ndSharedPtr<ndBody> BuildGridPlane(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation, bool kinematic)
{
	ndVector origin(-(ndFloat32)grids * gridSize * 0.5f, 0.0f, -(ndFloat32)grids * gridSize * 0.5f, 1.0f);

	ndArray<ndVector> points;
	for (ndInt32 iz = 0; iz <= grids; iz++)
	{
		ndFloat32 z0 = origin.m_z + (ndFloat32)iz * gridSize;
		for (ndInt32 ix = 0; ix <= grids; ix++)
		{
			ndFloat32 x0 = origin.m_x + (ndFloat32)ix * gridSize;
			points.PushBack(ndVector(x0, ndGaussianRandom(0.0f, perturbation), z0, 1.0f));
		}
	}

	ndMeshEffect meshEffect;
	meshEffect.BeginBuild();

	ndMeshEffect::ndMaterial material;
	ndArray<ndMeshEffect::ndMaterial>& materialArray = meshEffect.GetMaterials();
	strcpy(material.m_textureName, "marbleCheckBoard.png");
	materialArray.PushBack(material);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndVector face[16];
	ndFloat32 uvScale = 1.0 / 16.0f;
	ndInt32 materialIndex = 0;
	for (ndInt32 iz = 0; iz < grids; iz++)
	{
		for (ndInt32 ix = 0; ix < grids; ix++)
		{
			ndVector p0(points[(ix + 0) * (grids + 1) + iz + 0]);
			ndVector p1(points[(ix + 1) * (grids + 1) + iz + 0]);
			ndVector p2(points[(ix + 1) * (grids + 1) + iz + 1]);
			ndVector p3(points[(ix + 0) * (grids + 1) + iz + 1]);

			meshEffect.BeginBuildFace();
			meshEffect.AddMaterial(materialIndex);
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

			meshEffect.AddMaterial(materialIndex);
			meshEffect.AddPoint(p1.m_x, p1.m_y, p1.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p1.m_x * uvScale, p1.m_z * uvScale);

			meshEffect.AddMaterial(materialIndex);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);
			meshEffect.EndBuildFace();

			face[0] = p0;
			face[1] = p1;
			face[2] = p2;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);

			meshEffect.BeginBuildFace();
			meshEffect.AddMaterial(0);
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

			meshEffect.AddMaterial(0);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);

			meshEffect.AddMaterial(0);
			meshEffect.AddPoint(p3.m_x, p3.m_y, p3.m_z);
			meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
			meshEffect.AddUV0(p3.m_x * uvScale, p3.m_z * uvScale);
			meshEffect.EndBuildFace();

			face[0] = p0;
			face[1] = p2;
			face[2] = p3;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
		}
	}
	meshEffect.EndBuild(0.0f);
	meshBuilder.End(false);

	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("plane", &meshEffect, scene->GetShaderCache()));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	ndPhysicsWorld* const world = scene->GetWorld();
	//ndSharedPtr<ndBody> body(new ndBodyDynamic());
	ndSharedPtr<ndBody> body(kinematic ? new ndBodyKinematic() : new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(plane);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

ndSharedPtr<ndBody> BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, bool optimized, bool kinematic)
{
	ndMeshLoader loader;
	ndSharedPtr<ndMesh> meshEffectNode(loader.LoadMesh(meshName));
	ndAssert(*meshEffectNode);

	ndSharedPtr<ndDemoEntity> visualEntity(new ndDemoEntity(scene, *meshEffectNode));
	visualEntity->SetShadowMode(false);
	scene->AddEntity(visualEntity);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndInt32 stack = 1;
	ndMatrix matrixBuffer[1024];
	ndMesh* entBuffer[1024];

	entBuffer[0] = *meshEffectNode;
	matrixBuffer[0] = meshEffectNode->m_matrix.OrthoInverse();

	while (stack)
	{
		stack--;
		ndMesh* const ent = entBuffer[stack];
		ndMatrix matrix(ent->m_matrix * matrixBuffer[stack]);

		ndSharedPtr<ndMeshEffect> meshEffect = ent->GetMesh();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32(sizeof(ndFloat64));
			const ndFloat64* const vertexData = meshEffect->GetVertexPool();

			ndInt32 mark = meshEffect->IncLRU();
			ndPolyhedra::Iterator iter(*(*meshEffect));

			ndVector face[256];
			ndMatrix worldMatrix(ent->m_meshMatrix * matrix);
			for (iter.Begin(); iter; iter++)
			{
				ndEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					ndInt32 count = 0;
					ndEdge* ptr = edge;
					do
					{
						ndInt32 i = ptr->m_incidentVertex * vertexStride;
						ndVector point(ndFloat32(vertexData[i + 0]), ndFloat32(vertexData[i + 1]), ndFloat32(vertexData[i + 2]), ndFloat32(1.0f));
						face[count] = worldMatrix.TransformVector(point);
						count++;
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);

					ndInt32 materialIndex = meshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
				}
			}
		}

		for (ndMesh* child = ent->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			matrixBuffer[stack] = matrix;
			stack++;
		}
	}
	meshBuilder.End(optimized);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));

	ndMatrix matrix(visualEntity->GetCurrentMatrix());
	//ndSharedPtr<ndBody> body(new ndBodyDynamic());
	ndSharedPtr<ndBody> body(kinematic ? new ndBodyKinematic() : new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualEntity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(shape);

	scene->GetWorld()->AddBody(body);
	return body;
}


#endif

static void BuildPlaygroundHangingBridge(ndDemoEntityManager* const scene, const ndSharedPtr<ndMesh>& mesh, const ndSharedPtr<ndBody>& playgroundBody)
{
	// add a hanging bridge as a feature.
	const ndMesh* const end(mesh->FindChild("rampEnd"));
	const ndMesh* const start(mesh->FindChild("rampStart"));

	ndMatrix endMatrix(end->CalculateGlobalMatrix());
	ndMatrix startMatrix(start->CalculateGlobalMatrix());
	ndFloat32 dist = ndAbs(startMatrix.m_right.DotProduct(endMatrix.m_posit - startMatrix.m_posit).GetScalar());

	// calculate how many planks can be inserted between the two hardpoints.
	ndInt32 numberOfPlank = 1;
	while (dist > 2.0f)
	{
		numberOfPlank *= 2;
		dist *= ndFloat32(0.5f);
	}

	ndFloat32 plankSickness = 0.2f;
	ndFloat32 slackDist = dist * 1.01f;
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(11.0f, plankSickness, slackDist)));

	ndRender* const render = *scene->GetRenderer();
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_1.png")));

	ndSharedPtr<ndRenderSceneNode> bridgeMesh(new ndRenderSceneNodeInstance(startMatrix, descriptor));
	scene->AddEntity(bridgeMesh);

	ndMatrix localMatrix(ndGetIdentityMatrix());
	localMatrix.m_posit.m_z = dist * ndFloat32(0.5f);
	localMatrix.m_posit.m_y = -ndFloat32(0.5f) * plankSickness;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndFloat32 linkMass = 100.0f;
	ndFixSizeArray<ndBodyDynamic*, 256> bodyLinks;

	// create all the links and attach them to the parent scene node
	for (ndInt32 i = 0; i < numberOfPlank; ++i)
	{
		ndSharedPtr<ndRenderSceneNode>visualLink(new ndRenderSceneNode(localMatrix));
		bridgeMesh->AddChild(visualLink);

		ndSharedPtr<ndBody> body(new ndBodyDynamic());
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualLink));
		body->SetMatrix(visualLink->CalculateGlobalTransform());
		body->GetAsBodyKinematic()->SetCollisionShape(**shape);
		body->GetAsBodyKinematic()->SetMassMatrix(linkMass, **shape);
		body->GetAsBodyDynamic()->SetAngularDamping(ndVector(ndFloat32(0.5f)));
		world->AddBody(body);
		bodyLinks.PushBack(body->GetAsBodyDynamic());

		localMatrix.m_posit.m_z += dist;
	}
	ndRenderSceneNodeInstance* const instanceRoot = (ndRenderSceneNodeInstance*)*bridgeMesh;
	instanceRoot->Finalize();

	//connect every two adjecent link with a hinge joint
	ndMatrix matrix0(ndGetIdentityMatrix());
	ndMatrix matrix1(ndGetIdentityMatrix());

	matrix0.m_posit.m_z = slackDist * 0.5f;
	matrix0.m_posit.m_y = ndFloat32(0.5f) * plankSickness;

	matrix1.m_posit.m_z = -slackDist * 0.5f;
	matrix1.m_posit.m_y = ndFloat32(0.5f) * plankSickness;

	for (ndInt32 i = 0; i < bodyLinks.GetCount() - 1; ++i)
	{
		ndBodyDynamic* const body0 = bodyLinks[i];
		ndBodyDynamic* const body1 = bodyLinks[i + 1];
		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(matrix0 * body0->GetMatrix(), matrix1 * body1->GetMatrix(), body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}

	// connect the two ends
	{
		// start plank
		ndBodyDynamic* const body0 = bodyLinks[0];
		ndBodyDynamic* const body1 = playgroundBody->GetAsBodyDynamic();
		ndMatrix body0Matrix(matrix1 * body0->GetMatrix());
		ndMatrix body1Matrix(body0Matrix);
		body1Matrix.m_posit += body1Matrix.m_right.Scale(ndFloat32(0.5f) * (slackDist - dist));

		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(body0Matrix, body1Matrix, body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}

	{
		// end plank
		ndBodyDynamic* const body0 = bodyLinks[bodyLinks.GetCount() - 1];
		ndBodyDynamic* const body1 = playgroundBody->GetAsBodyDynamic();
		ndMatrix body0Matrix(matrix0 * body0->GetMatrix());
		ndMatrix body1Matrix(body0Matrix);
		body1Matrix.m_posit -= body1Matrix.m_right.Scale(ndFloat32(0.5f) * (slackDist - dist));

		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(body0Matrix, body1Matrix, body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}
}

ndSharedPtr<ndBody> BuildFloorBox(ndDemoEntityManager* const scene, const ndMatrix& matrix, const char* const textureName, ndFloat32 uvTiling, bool kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndSharedPtr<ndShapeInstance>box(new ndShapeInstance(new ndShapeBox(200.0f, 1.0f, 200.f)));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= uvTiling;
	uvMatrix[1][1] *= uvTiling;
	uvMatrix[2][2] *= uvTiling;

	ndRender* const render = *scene->GetRenderer();

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = box;
	descriptor.m_uvMatrix = uvMatrix;
	descriptor.m_stretchMaping = false;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	material.m_castShadows = false;
	material.m_specular = ndVector::m_zero;

	//ndSharedPtr<ndRenderPrimitive> geometry(ndRenderPrimitive::CreateMeshPrimitive(descriptor));
	ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));
	
	ndMatrix location(matrix);
	location.m_posit.m_y -= 0.5f;
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(location));
	entity->SetPrimitive(geometry);
	
	ndSharedPtr<ndBody> body(kinematic ? new ndBodyKinematic() : new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(location);
	body->GetAsBodyKinematic()->SetCollisionShape(**box);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

ndSharedPtr<ndBody> BuildFlatPlane(ndDemoEntityManager* const scene, const ndMatrix& matrix, const char* const textureName, bool optimized, bool kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector floor[] =
	{
		{ 200.0f, 0.0f,  200.0f, 1.0f },
		{ 200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f,  200.0f, 1.0f },
	};
	ndInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };
	
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	//meshBuilder.LoadPLY("sword.ply");
	//meshBuilder.LoadPLY("static_mesh.ply");
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[1][0], 3);
	meshBuilder.End(optimized);
	
	ndSharedPtr<ndShapeInstance>plane(new ndShapeInstance(new ndShapeStatic_bvh(meshBuilder)));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 10.0f;
	uvMatrix[1][1] *= 1.0f / 10.0f;
	uvMatrix[2][2] *= 1.0f / 10.0f;

	ndRender* const render = *scene->GetRenderer();
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = plane;
	descriptor.m_uvMatrix = uvMatrix;
	descriptor.m_stretchMaping = false;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	material.m_castShadows = false;
	material.m_specular = ndVector::m_zero;

	ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));

	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
	entity->SetPrimitive(geometry);

	ndSharedPtr<ndBody> body(kinematic ? new ndBodyKinematic() : new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(**plane);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

ndSharedPtr<ndBody> BuildPlayground(ndDemoEntityManager* const scene, bool kinematic)
{
	ndMeshLoader loader;
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("playground.fbx"));
	ndSharedPtr<ndShapeInstance>collision(loader.m_mesh->CreateCollision());

	kinematic = false;

	ndMatrix location(loader.m_mesh->CalculateGlobalMatrix());
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, loader.m_renderMesh));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(**collision);

	world->AddBody(body);
	scene->AddEntity(loader.m_renderMesh);

	BuildPlaygroundHangingBridge(scene, loader.m_mesh, body);
	return body;
}

class ndSceneMesh : public ndRenderSceneNode
{
	public:
	ndSceneMesh(
		ndDemoEntityManager* const scene, 
		ndMeshLoader& loader,
		const ndSharedPtr<ndRenderSceneNode>& playground, 
		const ndMatrix& matrix)
		:ndRenderSceneNode(matrix)
		,m_compoundScene(new ndShapeInstance(new ndShapeCompound()))
	{
		ndShapeCompound* const compound = m_compoundScene->GetShape()->GetAsShapeCompound();
		compound->BeginAddRemove();

			ndMatrix subShapeLocation(ndGetIdentityMatrix());

			// add the collision tree map
			playground->SetTransform(subShapeLocation);
			playground->SetTransform(subShapeLocation);
			AddChild(playground);
			AddPlayground(subShapeLocation, loader);

			// add an array of cylinders
			subShapeLocation.m_posit.m_x += 10.0f;
			AddSpeedBumpsSubShape(scene, subShapeLocation, 14);
		
			// we can add a lot more stuff
			subShapeLocation.m_posit.m_z -= 15.0f;
			AddBoxSubShape(scene, subShapeLocation);
		
			subShapeLocation.m_posit.m_z += 30.0f;
			AddBoxSubShape(scene, subShapeLocation);
		
			subShapeLocation.m_posit.m_z += 5.0f;
			AddBoxSubShape(scene, subShapeLocation);

			// keep add more stuff .... 

		compound->EndAddRemove();
	}

	private:
	void AddPlayground(const ndMatrix& location, const ndMeshLoader& loader)
	{
		ndSharedPtr<ndShapeInstance>collision(loader.m_mesh->CreateCollision());
		collision->SetLocalMatrix(location);
		ndShapeCompound* const compound = m_compoundScene->GetShape()->GetAsShapeCompound();
		compound->AddCollision(*collision);
	}

	void AddBoxSubShape(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndSharedPtr<ndShapeInstance> box(new ndShapeInstance(new ndShapeBox(0.5f, 4.0f, 0.5f)));

		ndRender* const render = *scene->GetRenderer();
		ndRenderPrimitive::ndDescriptor descriptor(render);
		descriptor.m_collision = box;
		descriptor.m_mapping = ndRenderPrimitive::m_box;
		descriptor.m_stretchMaping = false;
		descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("blueCheckerboard.png")));
		ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));

		ndMatrix matrix(location);
		matrix.m_posit.m_y += ndFloat32(1.9f);
		ndSharedPtr<ndRenderSceneNode>pole(new ndRenderSceneNode(matrix));
		pole->SetPrimitive(geometry);
		AddChild(pole);

		ndShapeMaterial material(box->GetMaterial());
		material.m_data.m_userData = *pole;
		box->SetMaterial(material);

		box->SetLocalMatrix(matrix);
		ndShapeCompound* const compound = m_compoundScene->GetShape()->GetAsShapeCompound();
		compound->AddCollision(*box);
	}

	void AddSpeedBumpsSubShape(ndDemoEntityManager* const scene, const ndMatrix& location, ndInt32 count)
	{
		ndSharedPtr<ndShapeInstance>capsule (new ndShapeInstance(new ndShapeCapsule(0.75f, 0.75f, 10.0f)));
		ndMatrix uvMatrix(ndGetIdentityMatrix());
		uvMatrix[0][0] *= 0.025f;
		uvMatrix[1][1] *= 0.025f;
		uvMatrix[2][2] *= 0.025f;

		ndRender* const render = *scene->GetRenderer();
		ndRenderPrimitive::ndDescriptor descriptor(render);
		descriptor.m_collision = capsule;
		descriptor.m_mapping = ndRenderPrimitive::m_capsule;
		descriptor.m_uvMatrix = uvMatrix;
		descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("concrete_011_color.png")));
		ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));

		ndFloat32 spacing = 3.0f;
		ndMatrix matrix(location);
		matrix.m_posit.m_x += 10.0f;
		matrix.m_posit.m_y += -0.2f;
		matrix.m_posit.m_z -= (ndFloat32)(count / 2) * spacing;
		ndShapeMaterial material(capsule->GetMaterial());

		ndShapeCompound* const compound = m_compoundScene->GetShape()->GetAsShapeCompound();
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndSharedPtr<ndRenderSceneNode>speedBump(new ndRenderSceneNode(matrix));
			speedBump->SetPrimitive(geometry);
			AddChild(speedBump);

			material.m_data.m_userData = *speedBump;
			capsule->SetMaterial(material);
			capsule->SetLocalMatrix(matrix);
			compound->AddCollision(*capsule);

			matrix.m_posit.m_z += spacing;
		}
	}

	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		// here we can  do a visibility test to check which children are visible from the camera, 
		// but in this demo we are just rendering the entire scene brute force.
		ndRenderSceneNode::Render(owner, parentMatrix, renderMode);
	}

	public:
	ndSharedPtr<ndShapeInstance> m_compoundScene;
};

ndSharedPtr<ndBody> BuildCompoundScene(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	// load the player arena map
	ndMeshLoader loader;
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("playground.fbx"));
	ndSharedPtr<ndRenderSceneNode> rootScene(new ndSceneMesh(scene, loader, loader.m_renderMesh, location));

	ndSceneMesh* const sceneMesh = (ndSceneMesh*)*rootScene;
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, rootScene));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(**sceneMesh->m_compoundScene);

	world->AddBody(body);
	scene->AddEntity(rootScene);

	// add the dynamics bodies part of the scene
	BuildPlaygroundHangingBridge(scene, loader.m_mesh, body);

	return body;
}