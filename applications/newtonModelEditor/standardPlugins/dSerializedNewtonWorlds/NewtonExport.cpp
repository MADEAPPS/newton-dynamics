/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonExport.cpp
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "StdAfx.h"
#include "NewtonExport.h"

#include <ctype.h>


NewtonExport::NewtonExport()
	:dExportPlugin()
{
}

NewtonExport::~NewtonExport()
{
}


NewtonExport* NewtonExport::GetPlugin()
{
	static NewtonExport plugin;
	return &plugin;
}


void NewtonExport::Export (const char* const fileName, dPluginInterface* const interface)
{
/*
	FILE* file = fopen (fileName, "rb");
	if (file) {
		ParceMapFile (file, world, NULL);
		
		fclose (file);
	}
*/
}

#if 0
NewtonExport::Property* NewtonExport::PropertyList::GetProperty (const char* name)
{
	_ASSERTE (0);

	unsigned crc = dCRC (name);
	for (NewtonExport::PropertyList::dListNode* node = GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo().m_name == crc) {
			return &node->GetInfo();
		}
	}
*/
	return NULL;
}



/*
void NewtonExport::AddModel (dModel* model, dScene* world)
{
	dScene::dTreeNode* rootNode = world->GetRootNode();

	// add the Mesh all meshes to the root node for latest use
	dTree<dScene::dTreeNode*, int> meshes;
	for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
		dScene::dTreeNode* meshNode = world->CreateMeshNode(rootNode);
		meshes.Insert(meshNode, node->GetInfo().m_boneID);
		// build the mesh geometry here
		CreateMeshGeomtry (meshNode, node->GetInfo().m_mesh, world);
	}
	
	// for each bone add one scene node to the scene
	dTree<dScene::dTreeNode*, int> sceneNodes;
	for (dList<dBone*>::dListNode* boneNode = model->m_skeleton.GetFirst(); boneNode; boneNode = boneNode->GetNext()) {
		AddSkeleton (boneNode->GetInfo(), model, world, rootNode, sceneNodes);
	}

	// bind the meshes to the skeleton of scene nodes
	dTree<dScene::dTreeNode*, int>::Iterator meshIter (meshes);
	for (meshIter.Begin(); meshIter; meshIter ++) {
		dScene::dTreeNode* meshNode = *meshIter;
	
		dTree<dScene::dTreeNode*, int>::dTreeNode* node = sceneNodes.Find(meshIter.GetKey());
		_ASSERTE (node);
		if (node) {
			dScene::dTreeNode* sceneNode = node->GetInfo();	
			world->AddReference(sceneNode, meshNode);
		}
	}

	// detach all mesh from the root node since they are not need it
	dTree<dScene::dTreeNode*, int>::Iterator iter (meshes);
	for (iter.Begin(); iter; iter ++) {
		world->RemoveReference(*iter, rootNode);
	}
}

void NewtonExport::AddSkeleton (dBone* bone, dModel* model, dScene* world, dScene::dTreeNode* parentNode, dTree<dScene::dTreeNode*, int>& sceneNodes)
{
	dScene::dTreeNode* boneNode = world->CreateSceneNode(parentNode);
	sceneNodes.Insert(boneNode, bone->GetBoneID());

	dSceneNodeInfo* meshModel = (dSceneNodeInfo*) world->GetInfoFromNode(boneNode);
	meshModel->SetName(bone->GetName());

	dMatrix matrix (bone->CalcGlobalMatrix ());
	meshModel->SetMatrix(matrix);
	
	for (bone = bone->GetChild(); bone; bone = bone->GetSibling()) {
		AddSkeleton (bone, model, world, boneNode, sceneNodes);
	}
}


void NewtonExport::CreateMeshGeomtry (dScene::dTreeNode* meshNode, dMesh* mesh, dScene* world)
{
	int materialID = 0;
	// iterate over all faces and add them one at a time 
	dMeshNodeInfo::neMeshInfoFlatPoint face[3];
	dMeshNodeInfo* editorMesh = (dMeshNodeInfo*) world->GetInfoFromNode (meshNode);
	editorMesh->BeginBuild();
	for (dMesh::dListNode* segNode = mesh->GetFirst(); segNode; segNode = segNode->GetNext()) {
		dSubMesh& segment = segNode->GetInfo();
		// add the material for these faces
		dScene::dTreeNode* materialNode = world->CreateMaterialNode (meshNode, materialID);
		dMaterialNodeInfo* material = (dMaterialNodeInfo*) world->GetInfoFromNode (materialNode);

		material->SetAmbientColor(segment.m_ambient);
		material->SetDiffuseColor(segment.m_diffuse);
		material->SetSpecularColor(segment.m_specular);
		material->SetEmissiveColor(dVector (0.0f, 0.0f, 0.0f, 1.0f));
		material->SetShininess(segment.m_shiness);
		material->SetOpacity(1.0f);

		// add a Texture to this scene if the mesh has one
		if (segment.m_textureName[0]) {
			dScene::dTreeNode* textureNode = world->CreateTextureNode (segment.m_textureName);

			// link the texture to the Material
			world->AddReference(materialNode, textureNode);

			// collada meshes use the same texture for ambient, diffuse, specular and emissive
			dTextureNodeInfo* texture = (dTextureNodeInfo*) world->GetInfoFromNode (textureNode);
			material->SetAmbientTextId(texture->GetId());
			material->SetDiffuseTextId(texture->GetId());
			material->SetSpecularTextId(texture->GetId());
			material->SetEmissiveTextId(texture->GetId());
		}


		// add all the faces
		for (int i = 0; i < segment.m_indexCount; i += 3) {
			for (int j = 0; j < 3; j ++) {
				int index = segment.m_indexes[i + j];
				face[j].m_x = mesh->m_vertex[index * 3 + 0];
				face[j].m_y = mesh->m_vertex[index * 3 + 1];
				face[j].m_z = mesh->m_vertex[index * 3 + 2];

				face[j].m_nx = mesh->m_normal[index * 3 + 0];
				face[j].m_ny = mesh->m_normal[index * 3 + 1];
				face[j].m_nz = mesh->m_normal[index * 3 + 2];

				face[j].m_u0 = mesh->m_uv[index * 2 + 0];
				face[j].m_v0 = mesh->m_uv[index * 2 + 1];

				// the dModel format only supports one uv channel
				face[j].m_u1 = 0.0f;
				face[j].m_v1 = 0.0f;
			}
			editorMesh->AddPolygon (3, face, materialID);
		}
		// increment material Index for the next material
		materialID ++;
	}
	editorMesh->EndBuild();
	editorMesh->ConvertToPolygons();
}
*/

void NewtonExport::SkipChar (FILE* file, char chr)
{
	while (!feof (file) && getc (file) != chr);	
}

int NewtonExport::NextChar (FILE* file)
{
	_ASSERTE (0);
	return 0;
/*
	int ch;
	for (ch = getc (file); !feof (file) && isspace(ch); ch = getc (file));
	return ch;
*/
}

/*
void NewtonExport::SkipClass (FILE* file)
{
	// skip unknown classes
	int count = 0;
	do {
		int marker = 0;
		for (marker = getc (file); (marker != '{') && (marker != '}'); marker = getc (file));
		if (marker == '{') {
			count ++;
		}
		if (marker == '}') {
			count --;
		}
	} while (count && !feof (file));
}





void NewtonExport::ReadProperties (FILE* file, dList<Property>& list)
{
	int ch;

	do {
		for (ch = getc (file); isspace (ch); ch = getc (file));
		ungetc (ch, file);
		if (ch == '\"') {
			Property& prop = list.Append()->GetInfo();
			char name[64];

			SkipChar (file, '\"');
			fscanf (file,  "%[^\"]", name);
			SkipChar (file, '\"');

			SkipChar (file, '\"');
			fscanf (file, "%[^\"]", prop.m_value);
			SkipChar (file, '\"');

			prop.m_name = dCRC (name);
		}
	} while (ch == '\"');

//	while ()
//	fscanf ("\"%s\"\"%s\"", name, value);
}

void NewtonExport::LoadVersionInfo(FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}

void NewtonExport::LoadVisGroups(FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}



void NewtonExport::LoadViewSettings(FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}



void NewtonExport::LoadEntity (FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}

void NewtonExport::LoadCamera (FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}

void NewtonExport::LoadCordon (FILE* file, dScene* world, void* userData)
{
	SkipClass (file);
}


class BrushList: public dList <NewtonMesh*>
{
	public:
	struct MaterialName
	{
		int m_id;
		char m_name[128];
	};
	class dTree <MaterialName, int> m_material;
};





class SolidPlane
{
	public:
	dBigVector m_plane;
	dVector m_u;
	dVector m_v;
	dFloat m_uScale;
	dFloat m_vScale;
	int m_material;
};




void NewtonExport::LoadSide (FILE* file, dScene* world, void* userData)
{
	SkipChar (file, '{');

	PropertyList list;
	ReadProperties (file, list);

	Property* plane = list.GetProperty ("plane");
//	Property* material = list.GetProperty ("material");
//	Property* uaxis = list.GetProperty ("uaxis");
//	Property* vaxis = list.GetProperty ("vaxis");

	dList<SolidPlane>* planeList = (dList<SolidPlane>*) userData;
	SolidPlane& solidPlane = planeList->Append()->GetInfo();

	dVector pp0;
	dVector pp1;
	dVector pp2;

	sscanf (plane->m_value, "(%f %f %f) (%f %f %f) (%f %f %f)", &pp0.m_x, &pp0.m_y, &pp0.m_z, &pp1.m_x, &pp1.m_y, &pp1.m_z, &pp2.m_x, &pp2.m_y, &pp2.m_z);

	dBigVector p0(pp0);
	dBigVector p1(pp1);
	dBigVector p2(pp2);

	dBigVector p2p0 (p2 - p0);
	dBigVector p1p0 (p1 - p0);
	dBigVector normal (p2p0 * p1p0);
	solidPlane.m_plane = normal.Scale (double (1.0f / sqrt (normal % normal)));
	solidPlane.m_plane.m_w = - (solidPlane.m_plane % p0);

	ParceMapFile (file, world, NULL);

	SkipChar (file, '}');
}


struct AddFaceData
{
	NewtonMesh* m_mesh;
	dList<SolidPlane>* m_planeList;
};

static void AddFaceToBrush (void* userData, int vertexCount, const dFloat* faceArray, int faceId)
{
	AddFaceData* data = (AddFaceData*) userData;

	dBigVector plane (0.0f, 0.0f, 0.0f, 0.0f);
	dBigVector p0 (faceArray[0], faceArray[1], faceArray[2], 0.0f);
	dBigVector p1 (faceArray[3], faceArray[4], faceArray[5], 0.0f);
	dBigVector e0 (p1 - p0);
	for (int i = 2; i < vertexCount; i ++) {
		int index = i * 3;
		dBigVector p2 (faceArray[index], faceArray[index + 1], faceArray[index + 2], 0.0f);
		dBigVector e1 (p2 - p0);
		plane += e0 * e1;
		e0 = e1;
	}
	plane = plane.Scale (double (1.0f / sqrt(plane % plane)));
	plane.m_w = - (plane % p0);

	// find the best matching plane

	SolidPlane* planeInfo = NULL;
	double maxProjection = 0.0;
	for (dList<SolidPlane>::dListNode* node = data->m_planeList->GetFirst(); node; node = node->GetNext()) {
		SolidPlane& info = node->GetInfo();
		double dot = info.m_plane % plane;
		if (dot > maxProjection) {
			maxProjection = dot;
			planeInfo = &info;
		}
	}

	// check if the planes are close enough if there are not some in very wrong with this brush
	_ASSERTE (maxProjection > 0.8);

	// here we must have the best plane from the list 
	_ASSERTE (planeInfo);
	
	if (planeInfo) {
		_ASSERTE (planeInfo);

		dMeshNodeInfo::neMeshInfoFlatPoint face[32];
		for (int i = 0; i < vertexCount; i ++) {
			int index = i * 3;
			face[i].m_x = faceArray[index];
			face[i].m_y = faceArray[index + 1];
			face[i].m_z = faceArray[index + 2];
				
			face[i].m_nx = dFloat(planeInfo->m_plane.m_x);
			face[i].m_ny = dFloat(planeInfo->m_plane.m_y); 
			face[i].m_nz = dFloat(planeInfo->m_plane.m_z); 
			
			face[i].m_u0 = 0.0f;
			face[i].m_v0 = 0.0f; 
			face[i].m_u1 = 0.0f;
			face[i].m_v1 = 0.0f; 
		}

		NewtonMeshAddFace(data->m_mesh, vertexCount, &face[0].m_x, sizeof (dMeshNodeInfo::neMeshInfoFlatPoint), 0);
	}
}




void NewtonExport::LoadSolid (FILE* file, dScene* world, void* userData)
{
	SkipChar (file, '{');

	PropertyList propertyList;
	ReadProperties (file, propertyList);

	BrushList* brushList = (BrushList*)userData;

	BrushList::dListNode* brushNode = brushList->Append(NewtonMeshCreate (world->GetNewtonWorld()));


	dList<SolidPlane> planeList;
	ParceMapFile (file, world, &planeList);

	// build the brush here;
	int count = 0;
	dVector points[128];
	for (dList<SolidPlane>::dListNode* node0 = planeList.GetFirst(); node0; node0 = node0->GetNext()) {
		dBigVector& p0 = node0->GetInfo().m_plane;
		for (dList<SolidPlane>::dListNode* node1 = node0->GetNext(); node1; node1 = node1->GetNext()) {
			dBigVector& p1 = node1->GetInfo().m_plane;
			for (dList<SolidPlane>::dListNode* node2 = node1->GetNext(); node2; node2 = node2->GetNext()) {
				dBigVector& p2 = node2->GetInfo().m_plane;
						
				double den = p0 % (p1 * p2);
				if (abs (den) > 1.0e-8f) {
					dBigVector q0 ((p1 * p2).Scale (p0.m_w));
					dBigVector q1 ((p2 * p0).Scale (p1.m_w));
					dBigVector q2 ((p0 * p1).Scale (p2.m_w));
					
					dBigVector q ((q0 + q1 + q2).Scale (-1.0f/den));
					bool isHullPoint = true;
					for (dList<SolidPlane>::dListNode* node = planeList.GetFirst(); node; node = node->GetNext()) {
						if ((node != node0) && (node != node1) && (node != node2)) {
							dBigVector& plane =  node->GetInfo().m_plane;
							double side = q % plane + plane.m_w;
							if (side >= 0.0f) {
								isHullPoint = false;
								break;
							}
						}
					}
					if (isHullPoint) {
						points[count] = dVector (dFloat(q.m_x), dFloat(q.m_y), dFloat(q.m_z), 0.0f);
						count ++;

					}
				}
			}
		}
	}


	_ASSERTE (count >= 4);

	AddFaceData faceDatebase;
	faceDatebase.m_mesh = brushNode->GetInfo();
	faceDatebase.m_planeList = &planeList; 
	dMatrix matrix (GetIdentityMatrix());


	NewtonWorld* newton = world->GetNewtonWorld();
	NewtonCollision* convexHull = NewtonCreateConvexHull(newton, count, &points[0].m_x, sizeof (dVector), 0.0f, 0, NULL);

	// make sure we do no read really bad data that some tme is in some of maps (ex flat planes passed as brushes)
	if (convexHull) {
		NewtonMeshBeginFace(faceDatebase.m_mesh);
		NewtonCollisionForEachPolygonDo (convexHull, &matrix[0][0], AddFaceToBrush, &faceDatebase);
		NewtonMeshEndFace(faceDatebase.m_mesh);
		NewtonMeshPolygonize (faceDatebase.m_mesh);
		NewtonReleaseCollision(newton, convexHull);
	}

	SkipChar (file, '}');
}


void NewtonExport::LoadWorld (FILE* file, dScene* world, void* userData)
{
	SkipChar (file, '{');
	PropertyList propertyList;
	ReadProperties (file, propertyList);

	BrushList brushList;
	ParceMapFile (file, world, &brushList);


	dScene::dTreeNode* rootNode = world->GetRootNode();

	
	
	
	// add the world scene node
	dScene::dTreeNode* worldSceneNode = world->CreateSceneNode(rootNode);
	dSceneNodeInfo* worldNodeInfo = (dSceneNodeInfo*) world->GetInfoFromNode(worldSceneNode);
	Property* worldName = propertyList.GetProperty ("classname");
	worldNodeInfo->SetName(worldName->m_value);
	//worldNodeInfo->SetMatrix(GetIdentityMatrix());
	worldNodeInfo->SetTransform(GetIdentityMatrix());

	// add the world mesh
	dScene::dTreeNode* worldMeshNode = world->CreateMeshNode(rootNode);
	dMeshNodeInfo* worldMeshNodeInfo = (dMeshNodeInfo*) world->GetInfoFromNode(worldMeshNode);
	worldMeshNodeInfo->SetName ("worldBrushesMesh");
	worldMeshNodeInfo->SetPivotMatrix(GetIdentityMatrix());
	
	// link the mesh to the scene node
	world->AddReference(worldSceneNode, worldMeshNode);

	_ASSERTE (0);

	worldMeshNodeInfo->BeginBuild();
	// append all brushes top the world mesh info	
	for (BrushList::dListNode* node = brushList.GetFirst(); node; node = node->GetNext()) {
		NewtonMesh* brush = node->GetInfo();

		// clip all bushes before adding the to the world
		for (BrushList::dListNode* node1 = node->GetNext(); node1; node1 = node1->GetNext()) {
//			_ASSERTE (0);
		}

		int stride = NewtonMeshGetPointStrideInByte (brush) / sizeof (dFloat); 
		dFloat* brushPoints = NewtonMeshGetPointArray (brush); 

		for (void* faceHandle = NewtonMeshGetFirstFace(brush); faceHandle; faceHandle = NewtonMeshGetNextFace(brush, faceHandle)) {
			if (!NewtonMeshIsFaceOpen(brush, faceHandle)) {
				int indices[32];
				dMeshNodeInfo::neMeshInfoFlatPoint face[32];
				int indexCount = NewtonMeshGetFaceIndexCount (brush, faceHandle);

				_ASSERTE (indexCount < sizeof (indices)/sizeof (indices[0]));

				NewtonMeshGetFacePointIndices (brush, faceHandle, indices);
				for (int i = 0; i < indexCount; i ++) {
					int index = indices[i];
					memcpy (&face[i].m_x, &brushPoints[index * stride], sizeof (dFloat) * stride);
				}
				worldMeshNodeInfo->AddPolygon(indexCount, face, 0);
			}
		}
	}
	worldMeshNodeInfo->EndBuild();
	worldMeshNodeInfo->ConvertToPolygons();

	// apply the rotation and scale
	dMatrix scale (GetIdentityMatrix());
	scale[0][0] = 1.0f / 100.0f; 
	scale[1][1] = 1.0f / 100.0f; 
	scale[2][2] = 1.0f / 100.0f; 
	dMatrix rotation (scale * dPitchMatrix (-90.0f * 3.141592f / 180.0f));
//	world->BakeSceneNodeTransformRecursive (worldSceneNode, rotation);
	world->BakeTransform (rotation);

	// destroy all brushes
	for (BrushList::dListNode* node = brushList.GetFirst(); node; node = node->GetNext()) {
		NewtonMeshDestroy (node->GetInfo());
	}
	SkipChar (file, '}');

}
*/

void NewtonExport::ParceMapFile (FILE* file, dScene* world, void* userData)
{
	_ASSERTE (0);
/*
	char className[64];
	int ch;
	while (((ch = NextChar(file)) != '}') && !feof(file)) {
		ungetc (ch, file);
		fscanf (file, "%s", className);

		if (!strcmp (className, "versioninfo")) {
			LoadVersionInfo(file, world, NULL);
		} else if (!strcmp (className, "visgroups")) {
			LoadVisGroups(file, world, NULL);
		} else if (!strcmp (className, "viewsettings")) {
			LoadViewSettings(file, world, NULL);
		} else if (!strcmp (className, "world")) {
			LoadWorld(file, world, NULL);
		} else if (!strcmp (className, "solid")) {
			LoadSolid(file, world, userData);
		} else if (!strcmp (className, "side")) {
			LoadSide(file, world, userData);
		} else if (!strcmp (className, "entity")) {
			LoadEntity(file, world, NULL);
		} else if (!strcmp (className, "cameras")) {
			LoadCamera(file, world, NULL);
		} else if (!strcmp (className, "cordon")) {
			LoadCordon(file, world, NULL);
		} else {
			SkipClass (file);
		}
	}
	ungetc (ch, file);
*/
}

#endif