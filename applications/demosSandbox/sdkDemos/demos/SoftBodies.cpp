/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "../DemoMesh.h"
#include "../DemoEntityManager.h"
#include "../DemoCamera.h"
#include "PhysicsUtils.h"




#if NEWTON_MAJOR_VERSION == 3
class SimpleSoftBodyEntity: public DemoEntity
{
	public:
	SimpleSoftBodyEntity (DemoEntityManager* const scene, NewtonCollision* const softCollision, const dVector& location)
		:DemoEntity (GetIdentityMatrix(), NULL)
	{
		dAssert (0);

/*
		// add an new entity to the world
		scene->Append (this);

		// create the render mesh
		DemoMesh* const mesh = new DemoMesh ("softMesh");
		SetMesh(mesh);
		mesh->Release();

		NewtonWorld* const world = scene->GetNewton();
		// calculate the moment of inertia and the relative center of mass of the solid

		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (softCollision, &inertia[0], &origin[0]);	

		dFloat mass = 1.0f;
		dFloat Ixx = mass * inertia[0];
		dFloat Iyy = mass * inertia[1];
		dFloat Izz = mass * inertia[2];

		//create the rigid body
		dMatrix matrix (GetIdentityMatrix());
		matrix.m_posit.m_x = location.m_x;
		matrix.m_posit.m_z = location.m_z;
		dAssert (0);
		//matrix.m_posit.m_y = FindFloor (world, matrix.m_posit.m_x, matrix.m_posit.m_z) + 4.0f;
		SetMatrix(*scene, dQuaternion(), matrix.m_posit);
		SetMatrix(*scene, dQuaternion(), matrix.m_posit);

		NewtonBody* const deformableBody = NewtonCreateDeformableBody (world, softCollision, &matrix[0][0]);

		// very important to remember that deformable collisions are instance, 
		// therefore the one in the body is a copy of the one use to create the body 
		m_softCollision = NewtonBodyGetCollision(deformableBody);

		// set the correct center of gravity for this body
		//	NewtonBodySetCentreOfMass (deformableBody, &origin[0]);

		// set the mass matrix
		NewtonBodySetMassMatrix (deformableBody, mass, Ixx, Iyy, Izz);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (deformableBody, this);

		// assign the wood id
		//	NewtonBodySetMaterialGroupID (deformableBody, materialId);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (deformableBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (deformableBody, DemoEntity::TransformCallback);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (deformableBody, PhysicsApplyGravityForce);

		// now create a visual representation for this entity
		CreateVisualMesh ();
*/
	}

	void CreateVisualMesh ()
	{
		DemoMesh* const mesh = GetMesh();

		// now create a visual representation for this mesh
		int vertexCount = NewtonDeformableMeshGetVertexCount (m_softCollision); 
		mesh->AllocVertexData(vertexCount);

		NewtonDeformableMeshUpdateRenderNormals(m_softCollision);
		NewtonDeformableMeshGetVertexStreams (m_softCollision, 3 * sizeof (dFloat), (dFloat*) mesh->m_vertex,
			3 * sizeof (dFloat), (dFloat*) mesh->m_normal,
			2 * sizeof (dFloat), (dFloat*) mesh->m_uv, 
			2 * sizeof (dFloat), (dFloat*) mesh->m_uv);

		for (NewtonDeformableMeshSegment* segmentNode = NewtonDeformableMeshGetFirstSegment (m_softCollision); segmentNode; segmentNode = NewtonDeformableMeshGetNextSegment (m_softCollision, segmentNode)) {
			int materialID = NewtonDeformableMeshSegmentGetMaterialID(m_softCollision, segmentNode);
			int indexCount = NewtonDeformableMeshSegmentGetIndexCount(m_softCollision, segmentNode);
			const short* const indexList = NewtonDeformableMeshSegmentGetIndexList(m_softCollision, segmentNode);

			DemoSubMesh* const segment = mesh->AddSubMesh();
			const char* const textName = FindTextureById(materialID);
			dAssert (textName);
			if (textName) {
				segment->m_textureHandle = materialID;
				//strcpy (segment->m_textureName, textName);
				//segment->m_shiness = material->GetShininess();
				//segment->m_ambient = material->GetAmbientColor();
				//segment->m_diffuse = material->GetDiffuseColor();
				//segment->m_specular = material->GetSpecularColor();
			}
			segment->m_indexCount = indexCount;
			segment->AllocIndexData (indexCount);
			for (int i = 0; i < indexCount; i ++) {
				segment->m_indexes[i] = indexList[i];
			}
		}
	}

	static NewtonCollision* CreateSoftBodyCollisionShape (DemoEntityManager* const scene)
	{
		NewtonWorld* const world = scene->GetNewton();

		// create two auxiliary objects to help with the graphics part
		// make a Box collision as a source to make a mesh 
		dVector size (2.0f, 2.0f, 2.0f, 0.0f);

		NewtonCollision* const box = CreateConvexCollision (world, GetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
		//NewtonCollision* const box = CreateConvexCollision (world, GetIdentityMatrix(), size, _SPHERE_PRIMITIVE, 0);
		

		// now convert the collision into a mesh, with materials and UV
		NewtonMesh* const mesh = NewtonMeshCreateFromCollision(box);
		int material = LoadTexture("smilli.tga");
		//NewtonMeshApplyBoxMapping(mesh, material, material, material);
		NewtonMeshApplySphericalMapping(mesh, material);

		// now create a soft collision mesh
		NewtonCollision* const softCollisionMesh = NewtonCreateDeformableMesh (world, mesh, 0);
		//NewtonDeformableMeshSetSkinThickness (softCollisionMesh, 1.0f);
		NewtonDeformableMeshSetSkinThickness (softCollisionMesh, 0.05f);
		
		// destroy the auxiliary objects
		NewtonMeshDestroy(mesh);
		NewtonDestroyCollision (box);
		return softCollisionMesh;
	}

	void Render(dFloat timeStep) const
	{
		DemoMesh* const mesh = GetMesh();

		// regenerate the soft body mesh for rendering
		NewtonDeformableMeshUpdateRenderNormals(m_softCollision);

		// update the vertex array
		NewtonDeformableMeshGetVertexStreams (m_softCollision, 3 * sizeof (dFloat), (dFloat*) mesh->m_vertex,
			3 * sizeof (dFloat), (dFloat*) mesh->m_normal,
			2 * sizeof (dFloat), (dFloat*) mesh->m_uv, 
			2 * sizeof (dFloat), (dFloat*) mesh->m_uv);

		// proceed with normal rendering
		DemoEntity::Render(timeStep);
	}

	NewtonCollision* m_softCollision;
};

#endif


void SoftBodies(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "playground.ngd", 1);

	dVector location (8.0f, 0.0f, -10.0f, 0.0f) ;

	NewtonCollision* const softBody = SimpleSoftBodyEntity::CreateSoftBodyCollisionShape (scene);
//	new SimpleSoftBodyEntity (scene, softBody, location);
	NewtonDestroyCollision (softBody);

	dQuaternion rot;
	dVector origin (location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
	scene->SetCameraMatrix(rot, origin);

//	scene->SaveScene ("test1.ngd");
//	dScene CreateAlchemediaFromPhysic(); 

}

