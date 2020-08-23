
/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "Common.h"
#include "import.h"
#include "ImportDesc.h"



class MaxNodeChache: public dTree<INode*, const dScene::dTreeNode* >
{

};


class GeometryCache: public dTree<GeomObject*, const dScene::dTreeNode* >
{
	public:
	void AddMesh (GeomObject* geometry, const dScene::dTreeNode* mesh)
	{
		Insert (geometry, mesh);
	}
};

class MaterialProxi
{
	public: 
	Mtl* m_mtl;
	int m_matID;
};


class MaterialCache: public dTree<MaterialProxi, int>
{
	public:
	MaterialCache(MultiMtl* const multiMat)
		:dTree<MaterialProxi, int>()
	{
		m_matID = 0;
		m_multiMat = multiMat;
		m_multiMat->SetMtlFlag(MTL_DISPLAY_ENABLE_FLAGS);
		m_multiMat->SetName("default");
	}

	//	void SetSubMaterialCount(int count)
	//	{
	//		m_multiMat->SetNumSubMtls(count);
	//	}

	const MaterialProxi* FindMaterial (int id) const
	{
		dTreeNode* const node = Find (id);
		if (node) {
			return &node->GetInfo();
		} else {
			return NULL;
		}
	}

	void AddMaterial (const MaterialProxi& material, int id)
	{
		MaterialProxi tmp (material);

		tmp.m_matID = m_matID; 
		Insert (tmp, id);
		m_multiMat->SetSubMtl(m_matID, tmp.m_mtl);
		tmp.m_mtl->SetMtlFlag(MTL_DISPLAY_ENABLE_FLAGS);
		m_matID ++;
	}


	int m_matID;
	MultiMtl* m_multiMat;
};


void Import::SetSmoothingGroups (Mesh& maxMesh)
{
	struct FaceAdjecency
	{
		int m_count;
		int m_adjacentFace[9];
	};


	int edgeCount;
	int faceIndexPool[1024 * 8];
	int triangleCount = maxMesh.getNumFaces(); 

	maxMesh.InvalidateTopologyCache();
	maxMesh.InvalidateGeomCache();
	Edge* const edgeList = maxMesh.MakeEdgeList(&edgeCount);

	FaceAdjecency* const adjacency = new FaceAdjecency [triangleCount];
	dVector* const faceNormals = new dVector[triangleCount];

	memset (adjacency, 0, triangleCount * sizeof (FaceAdjecency));
	for (int i = 0; i < edgeCount; i ++) {
		int face0 = edgeList[i].f[0];
		int face1 = edgeList[i].f[1];
		if ((face0 != -1) && (face1 != -1)) {
			_ASSERTE (face0 < triangleCount);
			_ASSERTE (face1 < triangleCount);

			adjacency[face0].m_adjacentFace[adjacency[face0].m_count] = face1;
			adjacency[face1].m_adjacentFace[adjacency[face1].m_count] = face0;

			adjacency[face0].m_count += 1;
			adjacency[face1].m_count += 1;

			_ASSERTE (adjacency[face0].m_count <= sizeof (adjacency[0].m_adjacentFace) / sizeof (adjacency[0].m_adjacentFace[0]));
			_ASSERTE (adjacency[face1].m_count <= sizeof (adjacency[0].m_adjacentFace) / sizeof (adjacency[0].m_adjacentFace[0]));
		}
	}

	for (int i = 0; i < triangleCount; i ++) {
		Face* face;
		face = &maxMesh.faces[i];
		dVector p0 (maxMesh.verts[face->v[0]].x, maxMesh.verts[face->v[0]].y, maxMesh.verts[face->v[0]].z, 0.0f);
		dVector p1 (maxMesh.verts[face->v[1]].x, maxMesh.verts[face->v[1]].y, maxMesh.verts[face->v[1]].z, 0.0f);
		dVector p2 (maxMesh.verts[face->v[2]].x, maxMesh.verts[face->v[2]].y, maxMesh.verts[face->v[2]].z, 0.0f);

		dVector normal ((p1 - p0) * (p2 - p0));
		faceNormals[i] = normal.Scale (1.0f / dSqrt (normal % normal));
	}


	unsigned group = 1;
	for (int i = 0; i < triangleCount; i ++) {
		Face* const face = &maxMesh.faces[i];
		if (!face->smGroup) {

			face->setSmGroup(group);
			faceIndexPool[0] = i;
			int stack = 1;

			while (stack) {
				stack --;
				int index = faceIndexPool[stack];

				dVector& n0 = faceNormals[index];
				for (int j = 0; j < adjacency[index].m_count; j ++) {
					int adjacentFaceIndex = adjacency[index].m_adjacentFace[j];
					Face* const adjacentFace = &maxMesh.faces[adjacentFaceIndex];

					if (!adjacentFace->smGroup) {
						dVector& n1 = faceNormals[adjacentFaceIndex];

						float dot = n0 % n1;
						if (dot > 0.86f) {
							if (stack < sizeof (faceIndexPool) / sizeof (faceIndexPool[0])) {
								adjacentFace->setSmGroup(group);
								faceIndexPool[stack] = adjacentFaceIndex;
								stack ++;
							}
						}
					}
				}
			}

			group = group * 2;
			if (!group) {
				group = 1;
			}
		}
	}


	delete[] faceNormals;
	delete[] adjacency;

	maxMesh.buildNormals();
}



void Import::LoadMaterials (dScene& scene, MaterialCache& materialCache)
{
	dScene::dTreeNode* const cacheNode = scene.GetMaterialCacheNode ();
	for (void* link = scene.GetFirstChildLink(cacheNode); link; link = scene.GetNextChildLink(cacheNode, link)) {
		dScene::dTreeNode* const materialNode = scene.GetNodeFromLink(link);
		dNodeInfo* const info = scene.GetInfoFromNode(materialNode);
		if (info->IsType(dMaterialNodeInfo::GetRttiType())) {
			MaterialProxi material;
			material.m_mtl = NewDefaultStdMat();
			StdMat* const stdMtl = (StdMat*)material.m_mtl;

			dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*) info;
			//stdMtl->SetName("default");
			// max do not let plug ins to set a sub material name
			// in the future I will change this to be simple materials
			stdMtl->SetName(materialInfo->GetName());

			dVector ambient (materialInfo->GetAmbientColor());
			dVector difusse (materialInfo->GetDiffuseColor());
			dVector specular (materialInfo->GetSpecularColor());
			float shininess (materialInfo->GetShininess());
			//float shininessStr (materialInfo->GetShinStr());      
			float transparency (materialInfo->GetOpacity());

			stdMtl->SetAmbient(*((Point3*)&ambient), 0);
			stdMtl->SetDiffuse(*((Point3*)&difusse), 0);
			stdMtl->SetSpecular(*((Point3*)&specular), 0);
			stdMtl->SetShinStr(shininess / 100.0f, 0);      
			stdMtl->SetOpacity(transparency, 0);

			if (materialInfo->GetDiffuseTextId() != -1) {
				dScene::dTreeNode* const textNode = scene.FindTextureByTextId(materialNode, materialInfo->GetDiffuseTextId());
				if (textNode) {
					_ASSERTE (textNode);

					dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*) scene.GetInfoFromNode(textNode);
					TCHAR txtNameBuffer[256];
					sprintf (txtNameBuffer, "%s/%s", m_path, textureInfo->GetPathName());

					const TCHAR* txtName = txtNameBuffer;
					BitmapTex* const bmtex = (BitmapTex*)NewDefaultBitmapTex();
					bmtex->SetMapName((TCHAR*)txtName);

					txtName = textureInfo->GetPathName();
					bmtex->SetName (txtName);
					bmtex->GetUVGen()->SetMapChannel(1);

					stdMtl->SetSubTexmap(ID_DI, bmtex);
					stdMtl->SetTexmapAmt(ID_DI, 1.0f, 0);
					stdMtl->EnableMap(ID_DI, TRUE);

	//					const char* materialOpanacity = segment.m_opacityTextureName;
	//					if (materialOpanacity[0]) {
	//						BitmapTex* bmtex;
	//						const TCHAR* txtName;
	//
	//						txtName = segment.m_opacityPathName;
	//						bmtex = (BitmapTex*)NewDefaultBitmapTex();
	//						bmtex->SetMapName((TCHAR*)txtName);
	//
	//						txtName = materialName;
	//						bmtex->SetName (txtName);
	//						bmtex->GetUVGen()->SetMapChannel(2);
	//
	//						stdMtl->SetSubTexmap(ID_OP, bmtex);
	//						stdMtl->SetTexmapAmt(ID_OP, 1.0f, 0);
	//						stdMtl->EnableMap(ID_OP, TRUE);
	//					}
	//				materialCache.AddMaterial(material, segment.m_textureName);
				}
			}
			materialCache.AddMaterial(material, int (materialInfo->GetId()));
		}
	}
}


void Import::LoadGeometries (dScene& scene, GeometryCache& meshCache, const MaterialCache& materialCache)
{
	dScene::dTreeNode* const geometryCache = scene.GetGeometryCacheNode ();
	for (void* link = scene.GetFirstChildLink(geometryCache); link; link = scene.GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const geometryNode = scene.GetNodeFromLink(link);
		dNodeInfo* const info = scene.GetInfoFromNode(geometryNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {

				// add the vertices
				//PolyObject* const geometry = (PolyObject*) CreateInstance (GEOMOBJECT_CLASS_ID, Class_ID(POLYOBJ_CLASS_ID, 0));
				PolyObject* const geometry = (PolyObject*) CreateInstance (GEOMOBJECT_CLASS_ID, EPOLYOBJ_CLASS_ID);
				
				meshCache.AddMesh(geometry, geometryNode);
				MNMesh& maxMesh = geometry->GetMesh();

				dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) scene.GetInfoFromNode(geometryNode);
				NewtonMesh* const mesh = meshInfo->GetMesh();

				//NewtonMeshTriangulate (mesh);
				//NewtonMeshPolygonize (mesh);

				int faceCount = 0;
				int vertexCount = NewtonMeshGetVertexCount(mesh);
				for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
					if (!NewtonMeshIsFaceOpen(mesh, face)) {
						faceCount ++;
					}
				}

				maxMesh.Clear();
				maxMesh.setNumVerts(vertexCount);
				maxMesh.setNumFaces(faceCount);

				// add all vertex
				int vertexStride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat64);
				dFloat64* const vertex = NewtonMeshGetVertexArray (mesh); 
				for (int j = 0; j < vertexCount; j ++) {
					maxMesh.P(j) = Point3 (vertex[vertexStride * j + 0], vertex[vertexStride * j + 1], vertex[vertexStride * j + 2]);
				}


				// count the number of face and make a face map
				int faceIndex = 0;
				for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
					if (!NewtonMeshIsFaceOpen(mesh, face)) {
						int faceIndices[256];

						int indexCount = NewtonMeshGetFaceIndexCount (mesh, face);
						int matId = NewtonMeshGetFaceMaterial (mesh, face);

						MaterialProxi material;
						material.m_mtl = 0;
						material.m_matID = 0;
						MaterialCache::dTreeNode* const materialNode = materialCache.Find(matId);
						if (materialNode) {
							material = materialNode->GetInfo();
						}

						NewtonMeshGetFaceIndices (mesh, face, faceIndices);
						MNFace* const face = maxMesh.F(faceIndex);
						face->MakePoly(indexCount, faceIndices, NULL, NULL);
						face->material = material.m_matID;

						faceIndex ++;
					}
				}



				int pointCount = NewtonMeshGetPointCount(mesh);
				int texChannels = 2;
				maxMesh.SetMapNum (texChannels);
				maxMesh.M(texChannels - 1)->ClearFlag (MN_DEAD);
				maxMesh.M(texChannels - 1)->setNumFaces (faceCount);
				maxMesh.M(texChannels - 1)->setNumVerts (pointCount);

				UVVert* const tv = maxMesh.M(texChannels - 1)->v;
				MNMapFace* const tf = maxMesh.M(texChannels - 1)->f;


				// add uvs
				dFloat64* const uv0 = NewtonMeshGetUV0Array(mesh); 
				int pointStride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat64);
				for (int j = 0; j < pointCount; j ++) {
					tv[j] = Point3 (uv0[pointStride * j + 0], uv0[pointStride * j + 1], 0.0);
				}

				faceIndex = 0;
				for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
					if (!NewtonMeshIsFaceOpen(mesh, face)) {
						int faceIndices[256];

						int indexCount = NewtonMeshGetFaceIndexCount (mesh, face);

						NewtonMeshGetFacePointIndices (mesh, face, faceIndices);
						MNMapFace* const textFace = &tf[faceIndex];
						textFace->MakePoly (indexCount, faceIndices);
						faceIndex ++;
					}
				}

				maxMesh.InvalidateGeomCache();
				maxMesh.InvalidateTopoCache();
				maxMesh.FillInMesh();
				maxMesh.AutoSmooth(45.0f * 3.1416f / 180.0f, false, false);

			} else {
				_ASSERTE (0);
			}
		}
	}
}


INode* Import::CreateMaxHelperNode ()
{
	HelperObject* const obj = (HelperObject*)CreateInstance(HELPER_CLASS_ID, Class_ID(DUMMY_CLASS_ID, 0));
	INode* const maxNode = m_ip->CreateObjectNode(obj);

//maxNode->SetBoneNodeOnOff(TRUE, 0);
//maxNode->BoneAsLine(TRUE);
//maxNode->ShowBone(1);
	return maxNode;
}


INode* Import::CreateMaxMeshNode (dScene& scene, Mtl *mtl, dScene::dTreeNode* meshNode, const GeometryCache& meshCache)
{
	_ASSERTE (meshCache.Find(meshNode));
	GeomObject* const maxObj = (GeomObject*) meshCache.Find(meshNode)->GetInfo();

//	maxObj->mesh.InvalidateTopologyCache();
//	maxObj->mesh.InvalidateGeomCache();
//	maxObj->mesh.buildNormals();

	INode* const maxNode = m_ip->CreateObjectNode(maxObj);

	
	dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) scene.GetInfoFromNode(meshNode);
	_ASSERTE (meshInfo->GetTypeId() == dMeshNodeInfo::GetRttiType());

	const dMatrix& matrix = meshInfo->GetPivotMatrix();
	Point3 posit (matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z);
	dQuaternion quat (matrix);
	Quat rot (-quat.m_q1, -quat.m_q2, -quat.m_q3, quat.m_q0);
	maxNode->SetObjOffsetRot(rot);
	maxNode->SetObjOffsetPos(posit);

	maxNode->SetMtl(mtl);
	return maxNode;
}


void Import::LoadNode (dScene& scene, INode* maxParent, dScene::dTreeNode* node, const dMatrix& parentMatrix, const GeometryCache& meshCache, Mtl *mtl, MaxNodeChache& maxNodeCache)
{
	dScene::dTreeNode* geometryNode = NULL;
	for (void* ptr = scene.GetFirstChildLink(node); ptr; ptr = scene.GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* const node = scene.GetNodeFromLink(ptr);
		dNodeInfo* info = scene.GetInfoFromNode(node);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			geometryNode = node;
			break;
		}
	}


	INode* maxNode = NULL;
	if (geometryNode) {
		maxNode = CreateMaxMeshNode (scene, mtl, geometryNode, meshCache);
		maxNodeCache.Insert (maxNode, geometryNode);
	} else {
		maxNode = CreateMaxHelperNode ();
	}
	
	_ASSERTE (maxNode);
	maxNodeCache.Insert (maxNode, node);

	if (maxParent) {
		maxParent->AttachChild(maxNode, 1);
	}

	dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*)scene.GetInfoFromNode(node);

	TCHAR name[128];
	TCHAR tmp[128];
	strcpy (tmp, sceneInfo->GetName());
	for (int i = 0; tmp[i]; i ++) {
		if (isspace(tmp[i])) {
			tmp[i] = '_';
		}
	}
	strcpy (name, tmp);
	for (int i = 1; m_ip->GetINodeByName(name); i ++) {
		sprintf (name, "%s_%02d", tmp, i);
//		node->SetNameID(name);
	}
	maxNode->SetName(name);

//	dMatrix transform (sceneInfo->GetTransform());
	dMatrix transform (sceneInfo->GetTransform() * parentMatrix);
//	dMatrix matrix;
//	for (int i = 0; i < 4; i ++) {
//		for (int j = 0; j < 4; j ++) {
//			matrix[i][j] = transform[i][j];
//		}
//	}
//	Matrix3 maxMatrix (GetMatrixFromdMatrix (matrix));
	Matrix3 maxMatrix (GetMatrixFromdMatrix (transform));
	maxNode->SetNodeTM(0, maxMatrix);

	for (void* ptr = scene.GetFirstChildLink(node); ptr; ptr = scene.GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
		dNodeInfo* info = scene.GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (scene, maxNode, node, transform, meshCache, mtl, maxNodeCache);
		}
	}
}

void Import::LoadNodes (dScene& scene, const GeometryCache& meshCache, Mtl *mtl, MaxNodeChache& maxNodeCache)
{
	dScene::dTreeNode* const root = scene.GetRootNode();

	for (void* ptr = scene.GetFirstChildLink(root); ptr; ptr = scene.GetNextChildLink(root, ptr) ) {
		dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
		dNodeInfo* info = scene.GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (scene, NULL, node, GetIdentityMatrix(), meshCache, mtl, maxNodeCache);
		}
	}
}


void Import::ApplyModifiers (dScene& scene, const MaxNodeChache& maxNodeCache)
{
	dScene::Iterator iter (scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			dScene::dTreeNode* skinModifierNode = NULL;	
			for (void* ptr = scene.GetFirstChildLink(meshNode); ptr; ptr = scene.GetNextChildLink(meshNode, ptr)) {
				dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(node);
				if (info->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
					skinModifierNode = node;
					break;
				}
			}

			if (skinModifierNode) {
				//create a skin modifier and add it
				Modifier* skinMod = (Modifier*) CreateInstance(OSM_CLASS_ID, SKIN_CLASSID);
				ISkinImportData* iskinImport = (ISkinImportData*) skinMod->GetInterface(I_SKINIMPORTDATA);
				INode* maxNode = maxNodeCache.Find(meshNode)->GetInfo(); 
				_ASSERTE (maxNode);

				IDerivedObject *derob = NULL;
				Object* obj = maxNode->GetObjectRef();	
				if(obj->SuperClassID() != GEN_DERIVOB_CLASS_ID)
				{
					derob = CreateDerivedObject(obj);
					maxNode->SetObjectRef(derob);
				} else {
					derob = (IDerivedObject*) obj;
				}
				derob->AddModifier(skinMod);

				dGeometryNodeSkinModifierInfo* skinModifier = (dGeometryNodeSkinModifierInfo*) scene.GetInfoFromNode(skinModifierNode);

				dMatrix matrix (skinModifier->m_shapeBindMatrix);
				Matrix3 bindPoseMatrix;
				bindPoseMatrix.SetRow (0, *((Point3*) &matrix[0]));
				bindPoseMatrix.SetRow (1, *((Point3*) &matrix[1]));
				bindPoseMatrix.SetRow (2, *((Point3*) &matrix[2]));
				bindPoseMatrix.SetRow (3, *((Point3*) &matrix[3]));
				iskinImport->SetSkinTm(maxNode, bindPoseMatrix, bindPoseMatrix);

				int maxNodeCount = 0;
				INode* maxNodes[1024];
			
				for (void* ptr = scene.GetFirstChildLink(skinModifierNode); ptr; ptr = scene.GetNextChildLink(skinModifierNode, ptr)) {
					dScene::dTreeNode* boneNode = scene.GetNodeFromLink(ptr);
					INode* skelBone = maxNodeCache.Find(boneNode)->GetInfo(); 
					maxNodes[maxNodeCount] = skelBone;
					maxNodeCount ++;
					skelBone->SetBoneNodeOnOff(TRUE, 0);
					skelBone->BoneAsLine(TRUE);
					skelBone->ShowBone(1);
					if (iskinImport->AddBoneEx(skelBone, TRUE)) {
						dSceneNodeInfo* sceneNode = (dSceneNodeInfo*) scene.GetInfoFromNode(boneNode);
						dMatrix matrix (sceneNode->GetTransform());
						Matrix3 bindPoseMatrix;
						bindPoseMatrix.SetRow (0, *((Point3*) &matrix[0]));
						bindPoseMatrix.SetRow (1, *((Point3*) &matrix[1]));
						bindPoseMatrix.SetRow (2, *((Point3*) &matrix[2]));
						bindPoseMatrix.SetRow (3, *((Point3*) &matrix[3]));
						iskinImport->SetBoneTm(skelBone, bindPoseMatrix, bindPoseMatrix);
					}
				}

				// must evaluate the node after adding bones
				maxNode->EvalWorldState(0);

				for (int i = 0; i < skinModifier->m_vertexCount; i ++) {
					Tab<float> weightList;
					Tab<INode*> boneNodeList;
					for (int j = 0; j < 4; j ++) {
						if (skinModifier->m_vertexWeights[i][j] > 1.0e-5f) {
							int boneIndex = skinModifier->m_boneWeightIndex[i].m_index[j];
							INode *skelBone = maxNodes[boneIndex];
							_ASSERTE (skelBone);		
							boneNodeList.Append (1, &skelBone);
							weightList.Append (1, &skinModifier->m_vertexWeights[i][j]);
						}
					}
					iskinImport->AddWeights(maxNode, i, boneNodeList, weightList);
				}
			}
		}
	}
}



void Import::SetSceneParameters()
{
	Interval range;

	range.SetStart(0);
	range.SetEnd(30);
	SetFrameRate(30);

	SetTicksPerFrame(160);

	range.SetStart(range.Start() * GetTicksPerFrame());
	range.SetEnd(range.End() * GetTicksPerFrame());

	m_ip->SetAnimRange(range);

	Point3 bkColor (0.0f, 0.0f, 0.0f);
	m_impip->SetBackGround(0, bkColor);

	Point3 amColor (0.3f, 0.3f, 0.3f);
	m_impip->SetAmbient(0, amColor);
}



Import::Import(const char* pathName, Interface* ip, ImpInterface* impip)
{
	// set the path for textures
	char* ptr = NULL;
	sprintf (m_path, "%s", pathName);
	for (int i = 0; m_path[i]; i ++) {
		if ((m_path[i] == '\\') || (m_path[i] == '/') ) {
			ptr = &m_path[i];
		}
	}
	*ptr = 0;


	m_ip = ip;
	m_impip = impip;
	m_succes = TRUE;
	MaterialCache materialCache (NewDefaultMultiMtl());

	SetSceneParameters();

	dFloat scale;
	scale = float (GetMasterScale(UNITS_METERS));
	dMatrix scaleMatrix (GetIdentityMatrix());
	scaleMatrix[0][0] = 1.0f / scale;
	scaleMatrix[1][1] = 1.0f / scale;
	scaleMatrix[2][2] = 1.0f / scale;
	dMatrix globalRotation (scaleMatrix * dPitchMatrix(3.14159265f * 0.5f) * dRollMatrix(3.14159265f * 0.5f));

	NewtonWorld* const newton = NewtonCreate();
	dScene scene (newton);

	scene.Deserialize (pathName);
	scene.DeleteDuplicateGeometries();

//	dScene::Iterator iter (scene);	
//	for (iter.Begin(); iter; iter ++) {
//		dScene::dTreeNode* node = iter.GetNode();
//		dNodeInfo* info = scene.GetInfoFromNode(node);
//		if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
//			dMeshNodeInfo* mesh = (dMeshNodeInfo*) scene.GetInfoFromNode(node);
//			mesh->ConvertToTriangles();
//		}
//	}
	scene.BakeTransform (globalRotation);
	
	GeometryCache geometryCache;
	MaxNodeChache maxNodeCache;

	LoadMaterials (scene, materialCache);
	LoadGeometries (scene, geometryCache, materialCache);
	LoadNodes (scene, geometryCache, materialCache.m_multiMat, maxNodeCache);
	ApplyModifiers (scene, maxNodeCache);

	scene.CleanUp();
	NewtonDestroy(newton);
}



Import::~Import(void)
{
}



