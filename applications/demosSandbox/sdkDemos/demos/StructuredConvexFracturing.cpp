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
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

#define MAX_POINT_CLOUD_SIZE		500
#define POINT_DENSITY_PER_METERS	0.5f  
#define POISON_VARIANCE				(0.75f * POINT_DENSITY_PER_METERS)


/*
static void EmitDebriesHitByThisBody (const NewtonBody* breakableBody, const NewtonJoint* contactJoint)
{
	NewtonCollision* collision;
	NewtonCollisionInfoRecord collisionInfo;

	// Get the collision shape for the body and see if this is a compound breakable shape
	collision = NewtonBodyGetCollision(breakableBody);
	NewtonCollisionGetInfo (collision, &collisionInfo);
	if (collisionInfo.m_collisionType == SERIALIZE_ID_COMPOUND_BREAKABLE) {
		// this is a compound breakable, finds the part at the contact and create the debris pieces

		dFloat maxSpeed;
		NewtonMaterial* bestMaterail;

		// find the location of the strongest impact
		maxSpeed = 0.0f;
		bestMaterail = NULL;
		for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
			dFloat speed;
			NewtonMaterial* material;

			material = NewtonContactGetMaterial (contact);
			speed = dAbs (NewtonMaterialGetContactNormalSpeed(material));
			if (speed > maxSpeed) {
				maxSpeed = speed ;
				bestMaterail = material;
			}
		}

		int debriCount;
		dVector contactPoint;
		dVector contactNormal;
		NewtonbreakableComponentMesh* components[32];
		dMatrix matrix;

		_ASSERTE (bestMaterail);
		NewtonMaterialGetContactPositionAndNormal (bestMaterail, &contactPoint.m_x, &contactNormal.m_x);
		NewtonBodyGetMatrix(breakableBody, &matrix[0][0]);

		// Get the origin in the space of the collision mesh
		contactPoint = matrix.UntransformVector(contactPoint);

		// get all the debris pieces on the radios of the impact
		dFloat impactRadius;

		impactRadius = 0.5f;
		debriCount = NewtonBreakableGetComponentsInRadius (collision, &contactPoint.m_x, impactRadius, components, sizeof (components)/sizeof (components[0]));

//xxx = debriCount;

		// delete each loose piece and emit a debris body
		NewtonBreakableBeginDelete (collision);
		for (int i = 0; i < debriCount; i ++) {
			int vertexCount;
			NewtonBody* body;
			ShareVertexMesh* meshInstance;
			NewtonbreakableComponentMesh* component;

			// get the information from this component and emit a visual and physics debris piece
			component = components[i];

			// create a debris rigid body
			body = NewtonBreakableCreateDebrieBody (collision, component);

			// a visual body mesh
			vertexCount = NewtonCompoundBreakableGetVertexCount (collision); 
			meshInstance = new ShareVertexMesh (vertexCount, g_vertex, g_normal, g_uv);
			for (void* segment = NewtonBreakableGetFirstSegment(component); segment; segment = NewtonBreakableGetNextSegment (segment)) {
				int material;
				int indexCount;
				dSubMesh* subMesh;
				subMesh = meshInstance->AddSubMesh();
				material = NewtonBreakableSegmentGetMaterial (segment); 
				indexCount = NewtonBreakableSegmentGetIndexCount (segment); 
				subMesh->m_textureHandle = (GLuint)g_materialMap[material];
				subMesh->AllocIndexData (indexCount);
				subMesh->m_indexCount = NewtonBreakableSegmentGetIndexStream (collision, component, segment, (int*)subMesh->m_indexes); 
			}

			NewtonWorld* world;
			RenderPrimitive* primitive;
			SceneManager* graphicsSystem;

			world = NewtonBodyGetWorld(breakableBody);
			graphicsSystem = (SceneManager*) NewtonWorldGetUserData(world);
			primitive = new RenderPrimitive (matrix, meshInstance);
			graphicsSystem->AddModel(primitive);
			meshInstance->Release();

			// set the callbacks
			NewtonBodySetMatrix(body, &matrix[0][0]);
			NewtonBodySetUserData (body, primitive);
			NewtonBodySetTransformCallback(body, PhysicsSetTransform);
			NewtonBodySetDestructorCallback(body, PhysicsBodyDestructor);
			NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

			// add new body to the debris queue
			g_debrieQueue.Append(body);

			// add this body to the Debris manager to keep the count manageable
			if (g_debrieQueue.GetCount() > 100) {
//			if (g_debrieQueue.GetCount() >= 1) {
				
				NewtonBody* oldBody;
			
				oldBody = g_debrieQueue.GetFirst()->GetInfo();
				
				// get the graphic object form the rigid body
				primitive = (RenderPrimitive*) NewtonBodyGetUserData (oldBody);

				// remove body for world, and graphics system
				graphicsSystem->RemoveModel(primitive);
				NewtonDestroyBody(world, oldBody);

				// remove body form the debris manager
				g_debrieQueue.Remove(g_debrieQueue.GetFirst());
			}


			// finally remove this component for the 
			NewtonBreakableDeleteComponent (collision, component);
		}
		NewtonBreakableEndDelete (collision);


		// now reconstruct the main visual mesh

		// we only need to create the sub mesh that had changed
		RenderPrimitive* primitive;
		NewtonbreakableComponentMesh* meshData;
		primitive = (RenderPrimitive*) NewtonBodyGetUserData(breakableBody);
		meshData = NewtonBreakableGetMainMesh (collision);
		for (void* segment = NewtonBreakableGetFirstSegment(meshData); segment; segment = NewtonBreakableGetNextSegment ( segment)) {
			int material;
			int indexCount;
			OGLMesh* mesh;
			dSubMesh* subMesh;

			mesh = NULL;
			subMesh = NULL;
			material = NewtonBreakableSegmentGetMaterial (segment); 
			indexCount = NewtonBreakableSegmentGetIndexCount (segment); 

			// find a mesh and a submesh by this material 
			for (ModelComponentList<dList<dMesh*> >::dListNode* list = primitive->m_meshList.GetFirst(); !mesh && list; list = list->GetNext()) {
				for (dList<dMesh*>::dListNode* node = list->GetInfo().m_data.GetFirst(); !mesh && node; node = node->GetNext()) { 
					for (dMesh::dListNode* meshSegment = node->GetInfo()->GetFirst(); !mesh && meshSegment; meshSegment = meshSegment->GetNext()) {
						if (int (meshSegment->GetInfo().m_textureHandle) == material) {
							mesh = (OGLMesh*) &list->GetInfo().m_data;
							subMesh = &meshSegment->GetInfo();
						}
					}
				}
			}

			_ASSERTE (mesh);
			if (mesh) {
				// if we did not find this material this is a new segment
				if (!subMesh) {
					subMesh = mesh->AddSubMesh();
					subMesh->m_textureHandle = material;
				}

				// now check if this segment can hold the vertex array;
				if (subMesh->m_indexCount < indexCount) {
					subMesh->AllocIndexData (indexCount);
				}
	//			subMesh->m_indexCount = NewtonBreakableSegmentGetIndexStreamShort (collision, meshData, segment, (short int*)subMesh->m_indexes); 
				subMesh->m_indexCount = NewtonBreakableSegmentGetIndexStream (collision, meshData, segment, (int*)subMesh->m_indexes); 
			}
		}
	}


	PrefFabDebrisDatabase::dTreeNode *debrisPiecesNode;

	// find a the strongest force 
	debrisPiecesNode = PrefFabDebrisDatabase::prefabDebrisDatabase.Find(NewtonBodyGetCollision(body));
	if (debrisPiecesNode) {
		dMatrix matrix;
		NewtonWorld* world;
		SceneManager* system;
		NewtonBody* rigidBody;
		dVector veloc;
		dVector omega;


		// Get the world;
		world = NewtonBodyGetWorld (body);
		system = (SceneManager*) NewtonWorldGetUserData(world);

		NewtonBodyGetOmega(body, &omega[0]);
		NewtonBodyGetVelocity(body, &veloc[0]);
		NewtonBodyGetMatrix (body, &matrix[0][0]);

		veloc = veloc.Scale (0.25f);
		omega = omega.Scale (0.25f);

		for (PrefFabDebriList::dListNode* node = debrisPiecesNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
			OGLMesh* meshInstance;
			NewtonCollision* collision;
			RenderPrimitive* primitive;
			PrefFabDebriElement& debriData = node->GetInfo();
		
			// make a visual object
			meshInstance = debriData.m_mesh;

			// create a visual geometry
			primitive = new RenderPrimitive (matrix, meshInstance);

			// save the graphics system
			system->AddModel (primitive);

			collision = debriData.m_shape;

			// calculate the moment of inertia and the relative center of mass of the solid
			//create the rigid body
			rigidBody = NewtonCreateBody (world, collision);

			// set the correct center of gravity for this body
			NewtonBodySetCentreOfMass (rigidBody, &debriData.m_com[0]);

			// set the mass matrix
			NewtonBodySetMassMatrix (rigidBody, debriData.m_mass, debriData.m_Ixx, debriData.m_Iyy, debriData.m_Izz);

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (rigidBody, primitive);

			// assign the wood id
			//	NewtonBodySetMaterialGroupID (rigidBody, NewtonBodyGetMaterialGroupID(source));

			// set continue collision mode
			NewtonBodySetContinuousCollisionMode (rigidBody, 1);

			// set a destructor for this rigid body
			NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback (rigidBody, PhysicsSetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (rigidBody, &matrix[0][0]);
			PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

			dVector debriOmega (omega);
			dVector debriVeloc (veloc + omega * matrix.RotateVector(debriData.m_com));

			// for now so that I can see the body
//debriVeloc = dVector (0, 0, 0, 0);
//debriVeloc.Scale (0.25f);
//debriVeloc.Omega
//omega = dVector (0, 0, 0, 0);

			NewtonBodySetOmega(rigidBody, &debriOmega[0]);
			NewtonBodySetVelocity(rigidBody, &debriVeloc[0]);
		}

		RenderPrimitive* srcPrimitive;
		srcPrimitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

		// remove the old visual from graphics world
		delete srcPrimitive;
		system->Remove(srcPrimitive);

		// finally destroy this body;
		NewtonDestroyBody(world, body);
	}
}


static FILE* OpenFileAsset(SceneManager& system, const char* name, NewtonBody* landScapeBody)
{
	GLuint tex;
	dFloat width; 
	dFloat breadth; 
	char binName[128];
	dMatrix pallete[512];
	char fullPathName[2048];
	char dependecyPathName[2048];

	FILE* file;
	OGLModel* model;
	NewtonMesh *meshCliper;
	NewtonCollision* compound;

	strcpy (binName, name);
	strtok (binName, ".");
	strcat (binName, ".bin");
	GetWorkingFileName (binName, fullPathName);
	GetWorkingFileName (name, dependecyPathName);

#ifdef _MSC_VER
	HANDLE handle;
	handle = CreateFileA(dependecyPathName, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (handle != INVALID_HANDLE_VALUE)	{
		FILETIME nameCreationTime;
		GetFileTime(handle, NULL, NULL, &nameCreationTime); 
		CloseHandle(handle);

		handle = CreateFileA(fullPathName, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (handle != INVALID_HANDLE_VALUE)	{
			FILETIME nameCreationTime1;
			GetFileTime(handle, NULL, NULL, &nameCreationTime1); 
			CloseHandle(handle);
			if (CompareFileTime(&nameCreationTime1, &nameCreationTime) > 0) {
#ifdef USE_SERILIZED_MESH
				return fopen (fullPathName, "rb");
#endif
			}
		}
	}

#else
	// I do no know how to very file dependencies in Linux and Mac
	file = fopen (fullPathName, "rb");
	if (file) {
		return file;
	}
#endif



	// create a Clip Plane for build the derbies pieces
	dMatrix plane (GetIdentityMatrix());
	dMatrix texMatrix (GetIdentityMatrix());

	width = 20.0f;
	breadth = 20.0f;

	tex = LoadTexture ("destructionDetail.tga");
	texMatrix[1][1] = 4.0f /breadth;
	texMatrix[2][2] = 4.0f /breadth;

	// create a clipper plane (only one for all solid, but there may be a different one for different meshes
	meshCliper = NewtonMeshCreatePlane (system.m_world, &plane[0][0], width, breadth, tex, &texMatrix[0][0], &texMatrix[0][0]);

	// load a model made from solid pieces, each pieces must intersect a nearby piece to make a complex solid large piece 
	model = new OGLModel ();
	OGLLoaderContext context;
	model->Load (dependecyPathName, context);

	// create the matrix pallete that locate each child solid piece, relative to the origin
	model->UpdateMatrixPalette (GetIdentityMatrix(), pallete, sizeof (pallete) / sizeof (dMatrix));

	FractalizedMeshCompounents fractalList;
	// iterate over the mode and searching for each solid piece of geometry
	for (ModelComponentList<dList<dMesh*> >::dListNode* list = model->m_meshList.GetFirst(); list; list = list->GetNext()) {
		for (dList<dMesh*>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			OGLMesh* mesh;
			mesh = (OGLMesh*)node->GetInfo();

			pallete[mesh->m_boneID].TransformTriplex(mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
			fractalList.FractalizeMesh (system.m_world, mesh, 0, meshCliper, tex, 100.0f);
		}
	}

	// we do not need the mode anymore
	delete model;

	// place all the information into a flat array, for passing to funtion NewtonCreateCompoundBreakable
	int debrieCount;
	int* partShapeID;
	int* internalFaceMaterial;
	dFloat* partDensity;
	const NewtonMesh** debrieArray;

	debrieCount = 0;
	partShapeID = (int*) malloc (fractalList.GetCount() * sizeof (int)); 
	internalFaceMaterial = (int*) malloc (fractalList.GetCount() * sizeof (int)); 
	partDensity = (dFloat*) malloc (fractalList.GetCount() * sizeof (dFloat)); 
	debrieArray	= (const NewtonMesh**) malloc (fractalList.GetCount() * sizeof (NewtonMesh*)); 
	for (FractalizedMeshCompounents::dListNode* node = fractalList.GetFirst(); node; node = node->GetNext()) {
		partShapeID[debrieCount] = node->GetInfo().m_partShapoeId;
		partDensity[debrieCount] = node->GetInfo().m_density; 
		debrieArray[debrieCount] = node->GetInfo().m_fractalPiece;
		internalFaceMaterial[debrieCount] = node->GetInfo().m_interiorMaterial;
		debrieCount ++;
	}
	// create ad solid destructible
	compound = NewtonCreateCompoundBreakable (system.m_world, debrieCount, debrieArray, partShapeID, partDensity, internalFaceMaterial, 0, 0, 0.025f);

	free (debrieArray);
	free (partDensity);
	free (partShapeID);
	free (internalFaceMaterial);


	// tell the engine which pieces of this compound are touching this static body and 
	// this call is only need it for compound how are static bodies
//	NewtonCreateCompoundBreakableSetAncheredPieces (compound, 0, NULL, NULL);

	// destroy the mesh because not need anymore
	NewtonMeshDestroy (meshCliper);

	// save the collision file
	file = fopen (fullPathName, "wb");

	// save the Material information Before Serializing the data 
	// Iterate over all components finding Material ID
	dTree<const char*,int>textureMap;
	for (NewtonbreakableComponentMesh* component = NewtonBreakableGetFirstComponent (compound); component; component = NewtonBreakableGetNextComponent (component)) {
		for (void* segment = NewtonBreakableGetFirstSegment(component); segment; segment = NewtonBreakableGetNextSegment (segment)) {
			int material;
			const char* textName; 

			material = NewtonBreakableSegmentGetMaterial (segment); 
			textName = FindTextureById (material);
			if (textName) {
				textureMap.Insert(textName, material);
			}
		}
	}

	// save all the textures
	int texCount;
	texCount = textureMap.GetCount();
	SerializeFile (file, &texCount, sizeof (int));
	dTree<const char*,int>::Iterator iter (textureMap);
	for (iter.Begin(); iter; iter++) {
		int id;
		char buffer[64];
		const char* name;
		const char* fullName;

		id = iter.GetNode()->GetKey();
		fullName = iter.GetNode()->GetInfo();
		name = strrchr (fullName, '/') + 1;
		sprintf (buffer, "%s", name);
		SerializeFile (file, &id, sizeof (int));
		SerializeFile (file, &buffer, sizeof (buffer));
	}


	// tell the engine which pieces of this compound are touching this static body and 
	// this call is only need it for compound how are static bodies
	dMatrix fixMatrix;
	NewtonCollision* fixCollision[1];
	NewtonBodyGetMatrix(landScapeBody, &fixMatrix[0][0]);
	fixCollision[0] = NewtonBodyGetCollision(landScapeBody);

//	NewtonCompoundBreakableResetAnchoredPieces (compound);
//	NewtonCompoundBreakableSetAnchoredPieces (compound, 1, &fixMatrix[0][0], fixCollision);

	NewtonCollisionSerialize (system.m_world, compound, SerializeFile, file);
	fclose (file);
	NewtonReleaseCollision (system.m_world, compound);

	return fopen (fullPathName, "rb");
}
*/




static int MakeRandomPoisonPointCloud(NewtonMesh* const mesh, dVector* const points)
{
	dVector size;
	dMatrix matrix(GetIdentityMatrix()); 
	NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

	dVector minBox (matrix.m_posit - matrix[0].Scale (size.m_x) - matrix[1].Scale (size.m_y) - matrix[2].Scale (size.m_z));
	dVector maxBox (matrix.m_posit + matrix[0].Scale (size.m_x) + matrix[1].Scale (size.m_y) + matrix[2].Scale (size.m_z));

	size = maxBox - minBox;
	int xCount = int (size.m_x / POINT_DENSITY_PER_METERS) + 1;
	int yCount = int (size.m_y / POINT_DENSITY_PER_METERS) + 1;
	int zCount = int (size.m_z / POINT_DENSITY_PER_METERS) + 1;

	int count = 0;
	dFloat z0 = minBox.m_z;
	for (int iz = 0; (iz < zCount) && (count < MAX_POINT_CLOUD_SIZE); iz ++) {
		dFloat y0 = minBox.m_y;
		for (int iy = 0; (iy < yCount) && (count < MAX_POINT_CLOUD_SIZE); iy ++) {
			dFloat x0 = minBox.m_x;
			for (int ix = 0; (ix < xCount) && (count < MAX_POINT_CLOUD_SIZE); ix ++) {

				dFloat x = x0;
				dFloat y = y0;
				dFloat z = z0;
				x += RandomVariable(POISON_VARIANCE);
				y += RandomVariable(POISON_VARIANCE);
				z += RandomVariable(POISON_VARIANCE);
				points[count] = dVector (x, y, z);
				count ++;
				x0 += POINT_DENSITY_PER_METERS;
			}
			y0 += POINT_DENSITY_PER_METERS;
		}
		z0 += POINT_DENSITY_PER_METERS;
	}
	

	return count;
}


static void OnReconstructMainMeshCallBack (NewtonBody* const body, NewtonFracturedCompoundMeshPart* const mainMesh)
{
	DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
	DemoMesh* const visualMesh = entity->GetMesh();
	NewtonCollision* const fracturedCompoundCollision = NewtonBodyGetCollision(body);

	dAssert (NewtonCollisionGetType(fracturedCompoundCollision) == SERIALIZE_ID_FRACTURED_COMPOUND);

	visualMesh->RemoveAll();
	for (void* segment = NewtonFracturedCompoundMeshPartGetFirstSegment(mainMesh); segment; segment = NewtonFracturedCompoundMeshPartGetNextSegment (segment)) {
		DemoSubMesh* const subMesh = visualMesh->AddSubMesh();

		int material = NewtonFracturedCompoundMeshPartGetMaterial (segment); 
		int indexCount = NewtonFracturedCompoundMeshPartGetIndexCount (segment); 

		subMesh->m_textureHandle = AddTextureRef ((GLuint)material);

		subMesh->AllocIndexData (indexCount);
		subMesh->m_indexCount = NewtonFracturedCompoundMeshPartGetIndexStream (fracturedCompoundCollision, mainMesh, segment, (int*)subMesh->m_indexes); 
	}
}

/*
static DemoMesh* CreateVisualMesh (NewtonCollision* const fracturedCompoundCollision)
{
	int vertexCount = NewtonFracturedCompoundCollisionGetVertexCount (fracturedCompoundCollision); 

	const dFloat* const vertex = NewtonFracturedCompoundCollisionGetVertexPositions (fracturedCompoundCollision);
	const dFloat* const normal = NewtonFracturedCompoundCollisionGetVertexNormals(fracturedCompoundCollision);
	const dFloat* const uv = NewtonFracturedCompoundCollisionGetVertexUVs (fracturedCompoundCollision);

	DemoMesh* const mesh = new DemoMesh ("fraturedMainMesh");
	mesh->AllocVertexData (vertexCount);

	dAssert (vertexCount == mesh->m_vertexCount);
	memcpy (mesh->m_vertex, vertex, 3 * vertexCount * sizeof (dFloat));
	memcpy (mesh->m_normal, normal, 3 * vertexCount * sizeof (dFloat));
	memcpy (mesh->m_uv, uv, 2 * vertexCount * sizeof (dFloat));

	NewtonFracturedCompoundMeshPart* const mainMesh = NewtonFracturedCompoundGetMainMesh (fracturedCompoundCollision);
	OnReconstructMainMeshCallBack (fracturedCompoundCollision, mainMesh);
	return mesh;
}
*/
static void CreateVisualEntity (DemoEntityManager* const scene, NewtonBody* const body)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);

	DemoEntity* const visualEntity = new DemoEntity(matrix, NULL);
	scene->Append(visualEntity);

	// set the entiry as teh use data;
	NewtonBodySetUserData (body, visualEntity);

	// create the mesh geometry and attach ot to the entity
	DemoMesh* const visualMesh = new DemoMesh ("fraturedMainMesh");
	visualEntity->SetMesh (visualMesh, GetIdentityMatrix());
	visualMesh->Release();

	// create the mesh and set the vertex array, but not the sub meshes
	NewtonCollision* const fracturedCompoundCollision = NewtonBodyGetCollision(body);
	dAssert (NewtonCollisionGetType(fracturedCompoundCollision) == SERIALIZE_ID_FRACTURED_COMPOUND);

	int vertexCount = NewtonFracturedCompoundCollisionGetVertexCount (fracturedCompoundCollision); 
	const dFloat* const vertex = NewtonFracturedCompoundCollisionGetVertexPositions (fracturedCompoundCollision);
	const dFloat* const normal = NewtonFracturedCompoundCollisionGetVertexNormals(fracturedCompoundCollision);
	const dFloat* const uv = NewtonFracturedCompoundCollisionGetVertexUVs (fracturedCompoundCollision);

	visualMesh->AllocVertexData (vertexCount);
	dAssert (vertexCount == visualMesh->m_vertexCount);
	memcpy (visualMesh->m_vertex, vertex, 3 * vertexCount * sizeof (dFloat));
	memcpy (visualMesh->m_normal, normal, 3 * vertexCount * sizeof (dFloat));
	memcpy (visualMesh->m_uv, uv, 2 * vertexCount * sizeof (dFloat));

	// now add the sub mesh by calling the call back
	NewtonFracturedCompoundMeshPart* const mainMesh = NewtonFracturedCompoundGetMainMesh (fracturedCompoundCollision);
	OnReconstructMainMeshCallBack (body, mainMesh);
}


static void AddStructuredFractured (DemoEntityManager* const scene, const dVector& origin, int materialID, const char* const assetName)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();


#if 0
	// load the mesh asset
	DemoEntity entity(GetIdentityMatrix(), NULL);	
	entity.LoadNGD_mesh (assetName, world);
	DemoMesh* const mesh = entity.GetMesh();
	dAssert (mesh);

	// convert the mesh to a newtonMesh
	NewtonMesh* const solidMesh = mesh->CreateNewtonMesh (world, entity.GetMeshMatrix() * entity.GetCurrentMatrix());
#else
	int externalMaterial = LoadTexture("wood_0.tga");
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), dVector (3.0f, 3.0f, 3.0f, 0.0), _BOX_PRIMITIVE, 0);
	NewtonMesh* const solidMesh = NewtonMeshCreateFromCollision(collision);
	NewtonDestroyCollision(collision);
	//NewtonMeshTriangulate(solidMesh);
	NewtonMeshApplyBoxMapping (solidMesh, externalMaterial, externalMaterial, externalMaterial);
#endif


	// create a random point cloud
	dVector points[MAX_POINT_CLOUD_SIZE];
	int pointCount = MakeRandomPoisonPointCloud (solidMesh, points);

	// create and interiors material for texturing the fractured pieces
	//int internalMaterial = LoadTexture("KAMEN-stup.tga");
	int internalMaterial = LoadTexture("concreteBrick.tga");

	// crate a texture matrix for uv mapping of fractured pieces
	dMatrix textureMatrix (GetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / 2.0f;
	textureMatrix[1][1] = 1.0f / 2.0f;

	/// create the fractured collision and mesh
	int debreePhysMaterial = NewtonMaterialGetDefaultGroupID(world);
	NewtonCollision* const structuredFracturedCollision = NewtonCreateFracturedCompoundCollision (world, solidMesh, 0, debreePhysMaterial, pointCount, &points[0][0], sizeof (dVector), internalMaterial, &textureMatrix[0][0],
																								  OnReconstructMainMeshCallBack);
    dVector com;
    dVector inertia;
    NewtonConvexCollisionCalculateInertialMatrix (structuredFracturedCollision, &inertia[0], &com[0]);	

    //float mass = 10.0f;
    //int materialId = 0;
    //create the rigid body
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_y = 20.0;
	matrix.m_posit.m_w = 1.0f;
    NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, structuredFracturedCollision, &matrix[0][0]);

	// set the mass and center of mass
	dFloat density = 1.0f;
	dFloat mass = density * NewtonConvexCollisionCalculateVolume (structuredFracturedCollision);
	NewtonBodySetMassProperties (rigidBody, mass, structuredFracturedCollision);


	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// create the entity and visual mesh and attach to the body as user data
	CreateVisualEntity (scene, rigidBody);

    // assign the wood id
//    NewtonBodySetMaterialGroupID (rigidBody, materialId);

    // set a destructor for this rigid body
//    NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// release the interior texture
//	ReleaseTexture (internalMaterial);

	// delete the solid mesh since it no longed needed
	NewtonMeshDestroy (solidMesh);

	// destroy the fracture collision
	NewtonDestroyCollision (structuredFracturedCollision);
}



void StructuredConvexFracturing (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	//CreateLevelMesh (scene, "ruinsFloor.ngd", true);
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	AddPrimitiveArray (scene, 0.0f, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f), 1, 1, 0, _BOX_PRIMITIVE, 0, GetIdentityMatrix());



	AddStructuredFractured (scene, dVector (0.0f, 0.0f, 0.0f, 0.0f), 0, "colum.ngd");


	// create a shattered mesh array
	//CreateSimpleVoronoiFracture (scene);

	//int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.75f, 0.75f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());

	// place camera into position
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * 3.141592f / 180.0f); 
	dVector origin (-45.0f, 20.0f, -15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



