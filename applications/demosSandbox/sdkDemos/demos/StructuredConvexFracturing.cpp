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


#if 0
class ShareVertexMesh: public OGLMesh
{
	public: 
	ShareVertexMesh (int vCount, dFloat* vertex, dFloat* normal, dFloat* uv)
		:OGLMesh ()   
	{
		m_vertexCount = vCount;
		m_vertex = vertex;
		m_normal = normal;
		m_uv = uv;
	}

	~ShareVertexMesh ()
	{
		m_vertex = NULL;
		m_normal = NULL;
		m_uv = NULL;
	}

};


static dFloat* g_vertex;
static dFloat* g_normal;
static dFloat* g_uv;
static int g_materialMap[256];

dList<NewtonBody*> g_debrieQueue;

// destroy the clipper Mesh
static void DestroyWorldCallback(const NewtonWorld* newtonWorld)
{
	free (g_vertex);
	free (g_normal);
	free (g_uv);
}


static void EmitDebriesHitByThisBody (const NewtonBody* breakableBody, const NewtonJoint* contactJoint)
{
	NewtonCollision* collision;
	NewtonCollisionInfoRecord collisionInfo;

static int xxx;
if (xxx)
return;

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

/*
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
*/
}


static void SetDemoCallbacks (SceneManager& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}


static NewtonBody* BuildFloorAndSceneRoot (SceneManager& system)
{
	NewtonWorld* world;
	NewtonBody* floorBody;
	LevelPrimitive* level;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	system.AddModel (new SkyBox ());

	// Load a level geometry
	level = new LevelPrimitive ("destructionFloor.mdl", world, 1);
	system.AddModel(level);
	floorBody = level->m_level;

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);

	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

	// save the callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (-1.0f, 0.0f, 0.0f), dVector (60.0f, 20.0f, -20.0f));

	return floorBody;
}


//static void UnstableStruture (SceneManager& system, dVector location, int high)
static void BreakableStruture (SceneManager& system, dVector location, int high)
{
	dFloat plankMass;
	dFloat columnMass;

	// /////////////////////////////////////////////////////////////////////
	//
	// Build a parking lot type structure

	dVector columnBoxSize (3.0f, 1.0f, 1.0f);
	//	dVector columnBoxSize (3.0f, 3.0f, 1.0f);
	dVector plankBoxSize (6.0f, 1.0f, 6.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// get the floor position in from o the camera
	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y += columnBoxSize.m_x;

	// set realistic (extremes in this case for 24 bits precision) masses for the different components
	// note how the newton engine handle different masses ratios without compromising stability, 
	// we recommend the application keep this ration under 100 for contacts and 50 for joints 
	columnMass = 50.0f;
	plankMass = 100.0f;

	// create a material 
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (system.m_world);

	dMatrix columAlignment (dRollMatrix(3.1416f * 0.5f));
	for (int i = 0; i < high; i ++) { 

		NewtonBody* body;
		RenderPrimitive* primitive;

		dMatrix matrix(columAlignment * baseMatrix);


		// add the 4 column
		matrix.m_posit.m_x -=  (columnBoxSize.m_z - plankBoxSize.m_x) * 0.5f;
		matrix.m_posit.m_z -=  (columnBoxSize.m_z - plankBoxSize.m_z) * 0.5f;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
//		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);
/*
		matrix.m_posit.m_x += columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
//		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_z += columnBoxSize.m_z - plankBoxSize.m_z;		
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
//		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x -= columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
//		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		// add a plank
		dVector size (plankBoxSize);
		size.m_x *= 0.85f;
		size.m_z *= 0.85f;
		body = CreateGenericSolid (system.m_world, &system, plankMass, baseMatrix, size, _BOX_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
//		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 80.0f);
		ConvexCastPlacement (body);
*/
		// set up for another level
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y);
	}

#if 0
	dFloat mass;
	NewtonBody* body;
	PrimitiveType type = _BOX_PRIMITIVE;
	dVector size (1.0f, 2.0f, 1.0f);
	dMatrix matrix (GetIdentityMatrix());

	mass = 10.0f;
	matrix.m_posit = location;
	matrix.m_posit.m_y = FindFloor (system.m_world, matrix.m_posit.m_x, matrix.m_posit.m_z) + baseMatrix.m_posit.m_y + 35.0f; 
	body = CreateGenericSolid (system.m_world, &system, mass, matrix, size, type, defaultMaterialID);
#endif
}




class FractalizedPiece
{
	public:
	dFloat m_density;
	int m_partShapoeId;
	int m_interiorMaterial;
	const NewtonMesh* m_fractalPiece;

	~FractalizedPiece ()
	{
		NewtonMeshDestroy(m_fractalPiece);
	}
};

class FractalizedMeshCompounents: public dList<FractalizedPiece>
{
	public: 
	FractalizedMeshCompounents ()
	{
	}

	~FractalizedMeshCompounents ()
	{
	}

	int CalculateClipPlane (const NewtonMesh* mesh, dMatrix& pinAndPivotSplitMatrix) const
	{
		dFloat x;
		dFloat y;
		dFloat z;

		// get the OOBB along for this mesh 
		NewtonMeshCalculateOOBB (mesh, &pinAndPivotSplitMatrix[0][0], &x, &y, &z);

//static int xxx;
//if (xxx > 28)
//return 0;
//xxx ++;

		if (x > 1.0f) {
//		if (x > 0.5f) {
			// if the largest piece is too big, the continue the split process 

			// randomize the clip plane position a little
			dVector posit = pinAndPivotSplitMatrix.m_posit;

			// randomize the plane orientation 
			posit += pinAndPivotSplitMatrix.m_front.Scale (x * RandomVariable(0.5f));
			pinAndPivotSplitMatrix = pinAndPivotSplitMatrix * dYawMatrix(3.1416f * RandomVariable(1.0f));
			pinAndPivotSplitMatrix = pinAndPivotSplitMatrix * dRollMatrix(3.1416f * RandomVariable(1.0f));

			pinAndPivotSplitMatrix.m_posit = posit;

			// return different than zero to indicate that the plane is valid and the process should continue spliting
			return 1;
		} 

		// return zero to indicate that this pieces should not be split anymore.
		return 0;
	}

	void AddPiece (const NewtonMesh* debrieMesh, int interiorFaceMaterial, dFloat density, int partShapeID)
	{
		dListNode* node;
		node = Append ();
		
		node->GetInfo().m_density = density;
		node->GetInfo().m_fractalPiece = debrieMesh;
		node->GetInfo().m_partShapoeId = partShapeID;
		node->GetInfo().m_interiorMaterial = interiorFaceMaterial;
	}


	void AddToHeap (dDownHeap<NewtonMesh*, dFloat>& heap, NewtonMesh* mesh)
	{
		// find all single connected pieces in this mesh and add then to the heap as individual solid pieces
		for (NewtonMesh* segment = NewtonMeshCreateFirstSingleSegment (mesh); segment; segment = NewtonMeshCreateNextSingleSegment (mesh, segment)) {
			dFloat x;
			dFloat y;
			dFloat z;
			dMatrix matrix;

			// get the OOBB along for this mesh 
			NewtonMeshCalculateOOBB (segment, &matrix[0][0], &x, &y, &z);
			heap.Push(segment, x);
		}
		NewtonMeshDestroy(mesh);
	}

	void ClipMesh (const NewtonMesh* mesh, const NewtonMesh* clipper, dMatrix& planeMatrix, NewtonMesh** front, NewtonMesh** back)
	{
		// clip the mesh 
		NewtonMeshClip (mesh, clipper, &planeMatrix[0][0], back, front);

		// if the clipper fail then wiggle the clip plane and try again
		for (int i = 0; (i < 3) && !(*back && *front && !NewtonMeshIsOpenMesh(*back) && !NewtonMeshIsOpenMesh(*front)) ; i ++) {
			dMatrix matrix;
			if (*front) {
				NewtonMeshDestroy(*front);
			}
			if (*back) {
				NewtonMeshDestroy(*back);
			}
			CalculateClipPlane (mesh, matrix);
			NewtonMeshClip (mesh, clipper, &matrix[0][0], back, front);
		}
	}


	void FractalizeMesh (const NewtonWorld* world, const OGLMesh* mesh, int partShapeID, const NewtonMesh* clipper, int clipperMaterialID, dFloat density)
	{
		NewtonMesh *solid;
		dDownHeap<NewtonMesh*, dFloat> heap (512);

		solid = mesh->BuildMesh(world);
		_ASSERTE (!NewtonMeshIsOpenMesh(solid));

		heap.Push(solid, 1.0f);
		while (heap.GetCount()) {
			int foundClipPlane;
			dMatrix matrix;
			const NewtonMesh* mesh;

			mesh = heap[0]; 
			heap.Pop();
			foundClipPlane = CalculateClipPlane (mesh, matrix);
			if (foundClipPlane) {
				NewtonMesh* back;
				NewtonMesh* front;

				// clip this piece but the clipper mesh
				ClipMesh (mesh, clipper, matrix, &back, &front);
				if (front && back) {
					if (!(NewtonMeshIsOpenMesh(front) || NewtonMeshIsOpenMesh(back))) { 
						// this piece was clipped and was the children are also solid pieces
						// we add each solid to the heap an continue the clipping process
						AddToHeap (heap, back);
						AddToHeap (heap, front);

						// delete the parent mesh
						NewtonMeshDestroy(mesh);
					} else {
						// the solid was clipped but it produces degenerated pieces, (pieces with open edges)
						// destroy the clipped pieces and and the parent piece to the list of components.
						NewtonMeshDestroy(front);
						NewtonMeshDestroy(back);
						AddPiece (mesh, clipperMaterialID, density, partShapeID);
					}

				} else {
					// Newton could not clip this solid piece, just added to the component list
					_ASSERTE (back == NULL);
					_ASSERTE (front == NULL);
					AddPiece (mesh, clipperMaterialID, density, partShapeID);
				}
			} else {
				// this is a small piece of debris, add it to the component list
				AddPiece (mesh, clipperMaterialID, density, partShapeID);
			}

			// check is the heap will overflow, 
			if (heap.GetCount() > (heap.GetMaxCount() - 16)) {
				// heap overflowed, remove the smaller pieces to make room to continue clipping the larger ones

				heap.Sort();
				while (heap.GetCount() >= (heap.GetMaxCount()>>1)) {
					mesh = heap[heap.GetCount() - 1]; 
					heap.Remove(heap.GetCount() - 1);
					AddPiece (mesh, clipperMaterialID, density, partShapeID);
				}
			}
		}
	}
};




static FILE* OpenFileAsset(SceneManager& system, const char* name, NewtonBody* landScapeBody)
{
_ASSERTE (0);
return 0;
/*
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
*/
}


static void LoadSolidBuilding (SceneManager& system, NewtonBody* landScapeBody)
{
	int vertexCount;
	int texCount;
	FILE* file;
	NewtonBody* body;
	NewtonWorld* world;
	NewtonCollision* compound;
	ShareVertexMesh* meshInstance;
	NewtonbreakableComponentMesh* meshData;


	world = system.m_world;

	// open the level data
	file = OpenFileAsset(system, "SolidRuins.mdl", landScapeBody);
	// read the textures;

	memset (g_materialMap, 0, sizeof (g_materialMap));
	DeSerializeFile (file, &texCount, sizeof (int));
	for (int i = 0; i < texCount; i ++) {
		int id;
		char buffer[64];
		DeSerializeFile (file, &id, sizeof (int));
		DeSerializeFile (file, &buffer, sizeof (buffer));
		g_materialMap[id] = LoadTexture(buffer);
	}

	compound = NewtonCreateCollisionFromSerialization (system.m_world, DeSerializeFile, file);
	fclose (file);

//dVector minP;
//dVector maxP;
//dMatrix matrix1 (GetIdentityMatrix());
//CalculateAABB (compound, matrix1, minP, maxP);

	// tell the engine which pieces of this compound are touching this static body and 
	// this call is only need it for compound how are static bodies
	dMatrix fixMatrix;
	NewtonCollision* fixCollision[1];
	NewtonBodyGetMatrix(landScapeBody, &fixMatrix[0][0]);
	fixCollision[0] = NewtonBodyGetCollision(landScapeBody);

	NewtonCompoundBreakableResetAnchoredPieces (compound);
	NewtonCompoundBreakableSetAnchoredPieces (compound, 1, &fixMatrix[0][0], fixCollision);

//	NewtonCollisionInfoRecord collisionInfo;
//	NewtonCollisionGetInfo (compound, &collisionInfo);


	// create a visual Mesh for the Compound Breakable
	vertexCount = NewtonCompoundBreakableGetVertexCount (compound); 

	g_vertex = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat)); 
	g_normal = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat)); 
	g_uv = (dFloat*) malloc (2 * vertexCount * sizeof (dFloat)); 
	NewtonCompoundBreakableGetVertexStreams (compound, 3 * sizeof (dFloat), g_vertex, 3 * sizeof (dFloat), g_normal, 2 * sizeof (dFloat), g_uv);


	meshData = NewtonBreakableGetMainMesh (compound);
	meshInstance = new ShareVertexMesh (vertexCount, g_vertex, g_normal, g_uv);
	for (void* segment = NewtonBreakableGetFirstSegment(meshData); segment; segment = NewtonBreakableGetNextSegment ( segment)) {
		int material;
		int indexCount;
		dSubMesh* subMesh;

		subMesh = meshInstance->AddSubMesh();

		material = NewtonBreakableSegmentGetMaterial (segment); 
		indexCount = NewtonBreakableSegmentGetIndexCount (segment); 

		subMesh->m_textureHandle = (GLuint)g_materialMap[material];

		subMesh->AllocIndexData (indexCount);
//		subMesh->m_indexCount = NewtonBreakableSegmentGetIndexStreamShort (compound, meshData, segment, (short int*)subMesh->m_indexes); 
		subMesh->m_indexCount = NewtonBreakableSegmentGetIndexStream (compound, meshData, segment, (int*)subMesh->m_indexes); 
	}


	dMatrix matrix (GetIdentityMatrix());
	RenderPrimitive* primitive;
	primitive = new RenderPrimitive (matrix, meshInstance);
	system.AddModel(primitive);

	meshInstance->Release();

	// create the rigid body
	body = NewtonCreateBody(world, compound);

	// Release the collision
	NewtonReleaseCollision(world, compound);

	//m_matrix = dPitchMatrix (15.0f* 3.141592f / 180.0f);
//	dMatrix matrix (GetIdentityMatrix());

	// set the global position of this body
	NewtonBodySetMatrix (body, &matrix[0][0]); 

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (body, primitive);
}


void BuildingDestruction(SceneManager& system)
{
	NewtonWorld* world;
	NewtonBody* floorBody;

	world = system.m_world;

	// create the sky box and the floor,
	floorBody = BuildFloorAndSceneRoot (system);

	// save the system wit the world
	NewtonWorldSetUserData(system.m_world, &system);

	// Load a Building structure made by solid pieces
	LoadSolidBuilding (system, floorBody);

	// Set a world destruction callback so that we can destroy all assets created for special effects
	NewtonWorldSetDestructorCallBack (system.m_world, DestroyWorldCallback);

	// Set a Function callback for when a Convex Body collision is destroyed if the force exceeded the Max break value
	NewtonSetDestroyBodyByExeciveForce (system.m_world, EmitDebriesHitByThisBody); 

	// this will load a assets to create destruction special effects 
//	PrefFabDebrisDatabase::prefabDebrisDatabase.LoadDestructionCutterMesh ();

//	BreakableStruture (system, dVector (-10.0f, 0.0f, -10.0f, 0.0f), 2);
	BreakableStruture (system, dVector (-10.0f, 0.0f, -12.0f, 0.0f), 1);
//	BreakableStruture (system, dVector (-15.0f, 0.0f,  10.0f, 0.0f), 3);
//	BreakableStruture (system, dVector ( 15.0f, 0.0f,  10.0f, 0.0f), 3);
}

#endif


#define POINT_CLOUD_SIZE	50

static void MakeRandomPointCloud(NewtonMesh* const mesh, dVector* const points)
{
	dVector size;
	dMatrix matrix(GetIdentityMatrix()); 
	NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

	dVector minBox (matrix.m_posit - matrix[0].Scale (size.m_x) - matrix[1].Scale (size.m_y) - matrix[2].Scale (size.m_z));
	dVector maxBox (matrix.m_posit + matrix[0].Scale (size.m_x) + matrix[1].Scale (size.m_y) + matrix[2].Scale (size.m_z));

	int count = 0;		
	size = (maxBox - minBox).Scale (0.5f);
	dVector center ((maxBox + minBox).Scale (0.5f));

	while (count < POINT_CLOUD_SIZE) {			
		dFloat x = RandomVariable(size.m_x);
		dFloat y = RandomVariable(size.m_y);
		dFloat z = RandomVariable(size.m_z);
x = 0;
y = 0;
z = 0;
//		if ((x < maxBox.m_x) && (x > minBox.m_x) && (y < maxBox.m_y) && (y > minBox.m_y) && (z < maxBox.m_z) && (z > minBox.m_z)){
			points[count] = center + dVector (x, y, z);
			count ++;
//		}
	}
}



static void AddStructuredFractured (DemoEntityManager* const scene, const dVector& origin, int materialID, const char* const assetName)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();

	// load the mesh asset
	DemoEntity entity(GetIdentityMatrix(), NULL);	
	entity.LoadNGD_mesh (assetName, world);
	DemoMesh* const mesh = entity.GetMesh();
	dAssert (mesh);

	// convert the mesh to a newtonMesh
	NewtonMesh* const solidMesh = mesh->CreateNewtonMesh (world, entity.GetMeshMatrix() * entity.GetCurrentMatrix());

	// create a random point cloud
	dVector points[POINT_CLOUD_SIZE];
	MakeRandomPointCloud (solidMesh, points);

	// create and interiors material for texturing the fractured pieces
	int internalMaterial = LoadTexture("KAMEN-stup.tga");

	// crate a texture matrix for uv mapping of fractured pieces
	dMatrix textureMatrix (GetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / 2.0f;
	textureMatrix[1][1] = 1.0f / 2.0f;

	/// create the fractured collision and mesh
	int debreePhysMaterial = NewtonMaterialGetDefaultGroupID(world);
	NewtonCollision* const structuredFracturedCollision = NewtonCreateCompoundBreakable (world, solidMesh, 0, debreePhysMaterial, POINT_CLOUD_SIZE, &points[0][0], sizeof (dVector), internalMaterial, &textureMatrix[0][0]);

//    dQuaternion rotation;
//   SimpleFracturedEffectEntity* const entity = new SimpleFracturedEffectEntity (visualMesh, fractureEffect);
    dMatrix matrix (GetIdentityMatrix());
    matrix.m_posit = origin;
    DemoEntity* const visualEntity= new DemoEntity(matrix, NULL);
    scene->Append(visualEntity);

    dVector com;
    dVector inertia;
    NewtonConvexCollisionCalculateInertialMatrix (structuredFracturedCollision, &inertia[0], &com[0]);	

    float mass = 10.0f;
    int materialId = 0;
    //create the rigid body
    NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, structuredFracturedCollision, &matrix[0][0]);
/*
    entity->m_myBody = rigidBody;
    entity->m_myMassInverse = 1.0f / mass;

    // set the correct center of gravity for this body
    //NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

    // set the mass matrix
    NewtonBodySetMassProperties (rigidBody, mass, collision);

    // save the pointer to the graphic object with the body.
    NewtonBodySetUserData (rigidBody, entity);

    // assign the wood id
    NewtonBodySetMaterialGroupID (rigidBody, materialId);

    //  set continue collision mode
    //	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

    // set a destructor for this rigid body
    NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

    // set the transform call back function
    NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);
*/



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



