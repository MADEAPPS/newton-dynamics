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

#include "stdafx.h"
#include "dTree.h"
#include "dMatrix.h"
#include "Export.h"

#define SCALE 100.0f

struct SortVertex
{
	float m_x;
	float m_y;
	float m_z;
	float m_nx;
	float m_ny;
	float m_nz;
	float m_u;
	float m_v;
	float m_attribute;
};

struct VertexAttribute
{
	int m_material;
	float m_weight[4];
	int	m_boneIndex[4];
};


Exporter::Exporter ()
{
}

Exporter::~Exporter ()
{
}

int Exporter::GetType ()
{
	return cMsPlugIn::eTypeExport;
}

const char* Exporter::GetTitle ()
{
	return "Newton Alchemedia Exporter...";
}

/*
void Exporter::AddSkeleton (msModel* pModel, dModel& model)
{
	int boneCount;
	
	boneCount = 0;
	for (ModelComponentList<dList<dBone*> >::dListNode* list = model->m_skeleton.GetFirst(); list; list = list->GetNext()) {
		for (dList<dBone*>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			for (dBone* bone = node->GetInfo()->GetFirst(); bone; bone = bone->GetNext()) {
				boneCount ++;
			}
		}
	}

	for (int i = 0; i < boneCount; i ++) {
		int boneIndex;
		msBone* pBone;
		dBone* srcBone;

		srcBone = model->FindBone(i);
		_ASSERTE (srcBone);

		boneIndex = msModel_AddBone (pModel);
		pBone = msModel_GetBoneAt (pModel, boneIndex);

		msBone_SetName (pBone, srcBone->GetName());
		if (srcBone->GetParent()) {
			msBone_SetParentName (pBone, srcBone->GetParent()->GetName());
		}

		msVec3 position, rotation;
		dMatrix matrix (srcBone->GetMatrix());

		position[0] = matrix.m_posit.m_x * SCALE;
		position[1] = matrix.m_posit.m_y * SCALE;
		position[2] = matrix.m_posit.m_z * SCALE;

		dVector angle (matrix.GetXYZ_EulerAngles());
		rotation[0] = angle.m_x; 
		rotation[1] = angle.m_y; 
		rotation[2] = angle.m_z; 

		msBone_SetPosition (pBone, position);
		msBone_SetRotation (pBone, rotation);
	}
}

void Exporter::ConvertMeshToSkins(dModel& model)
{
	int meshCount;

	int vertexCount;
	dMesh* meshes[256];

	meshCount = 0;
	vertexCount = 0;
	for (ModelComponentList<dList<dMesh*> >::dListNode* list = model->m_meshList.GetFirst(); list; list = list->GetNext()) {
		for (dList<dMesh*>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			if (node->GetInfo()->GetType() == dMesh::D_STATIC_MESH) {
				meshes[meshCount] = node->GetInfo();
				vertexCount += meshes[meshCount]->m_vertexCount;
				meshCount ++;
			}
		}
	}
	if (meshCount) {
		dMesh* skin;
		int vertexIndex;
		
		vertexIndex = 0;
		skin = new dMesh (dMesh::D_SKIN_MESH);

		strcpy (skin->m_name, model->FindBone(0)->GetName());

		skin->AllocVertexData (vertexCount);
		for (int i = 0; i < meshCount; i ++) {
			dMesh* mesh;
			dBone* bone;

			mesh = meshes[i];
			memcpy (&skin->m_vertex[vertexIndex * 3], mesh->m_vertex, 3 * mesh->m_vertexCount * sizeof (dFloat));
			memcpy (&skin->m_normal[vertexIndex * 3], mesh->m_normal, 3 * mesh->m_vertexCount * sizeof (dFloat));
			memcpy (&skin->m_uv[vertexIndex * 2], mesh->m_uv, 2 * mesh->m_vertexCount * sizeof (dFloat));

			bone = model->FindBone (mesh->m_boneID);
			dMatrix matrix (bone->CalcGlobalMatrix());

			matrix.TransformTriplex (&skin->m_vertex[vertexIndex * 3], 3 * sizeof (dFloat), &skin->m_vertex[vertexIndex * 3], 3 * sizeof (dFloat), mesh->m_vertexCount);

			matrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
			matrix.TransformTriplex (&skin->m_normal[vertexIndex * 3], 3 * sizeof (dFloat), &skin->m_normal[vertexIndex * 3], 3 * sizeof (dFloat), mesh->m_vertexCount);

			for (int j = 0; j < mesh->m_vertexCount; j ++) {
				skin->m_weighList->m_vertexWeight[j + vertexIndex][0] = 1.0f;
				skin->m_weighList->m_boneWeightIndex[j + vertexIndex].m_index[0] = mesh->m_boneID;
			}

			for (dMesh::dListNode* node = mesh->GetFirst(); node; node = node = node->GetNext()) {
				dSubMesh& srcMesh = node->GetInfo();
				dSubMesh& subMesh = *skin->AddSubMesh();

				subMesh.m_ambient = srcMesh.m_ambient;
				subMesh.m_diffuse = srcMesh.m_diffuse;
				subMesh.m_specular = srcMesh.m_specular;
				subMesh.m_shiness = srcMesh.m_shiness;
				strcpy (subMesh.m_textureName, srcMesh.m_textureName);
				subMesh.AllocIndexData (srcMesh.m_indexCount);

				for (int j = 0; j < srcMesh.m_indexCount; j ++) {
					subMesh.m_indexes[j] = unsigned short (srcMesh.m_indexes[j] + vertexIndex); 
				}
			}

			vertexIndex += mesh->m_vertexCount;

			model->RemoveMesh (mesh);
		}

		model->AddMesh (skin);
		skin->Release();
	}
}


void Exporter::MergeEqualMaterials(dModel& model)
{
	for (ModelComponentList<dList<dMesh*> >::dListNode* list = model->m_meshList.GetFirst(); list; list = list->GetNext()) {
		for (dList<dMesh*>::dListNode* meshNode = list->GetInfo().m_data.GetFirst(); meshNode; meshNode = meshNode->GetNext()) { 
			dMesh* skin;

			skin = meshNode->GetInfo();
			for (dMesh::dListNode* node = skin->GetFirst(); node; node = node->GetNext()) {
				int indexCount = node->GetInfo().m_indexCount;
				for (dMesh::dListNode* node1 = node->GetNext(); node1; node1 = node1->GetNext()) {
					if (!strcmp (node->GetInfo().m_textureName, node1->GetInfo().m_textureName)) {
						dVector error (node->GetInfo().m_ambient - node1->GetInfo().m_ambient);
						if (error % error < 1.0e-4f) {
							dVector error (node->GetInfo().m_diffuse - node1->GetInfo().m_diffuse);
							if (error % error < 1.0e-4f) {
								dVector error (node->GetInfo().m_specular - node1->GetInfo().m_specular);
								if (error % error < 1.0e-4f) {
									indexCount += node1->GetInfo().m_indexCount;
									node1->GetInfo().m_textureHandle = 1;
								}
							}
						}
					}
				}

				if (indexCount > node->GetInfo().m_indexCount) {
					dMesh::dListNode* newNode;
					node->GetInfo().m_textureHandle = 1;
					
					newNode = skin->Addtop();
					dSubMesh& srcMesh = node->GetInfo();
					dSubMesh& subMesh = newNode->GetInfo();

					subMesh.m_ambient = srcMesh.m_ambient;
					subMesh.m_diffuse = srcMesh.m_diffuse;
					subMesh.m_specular = srcMesh.m_specular;
					subMesh.m_shiness = srcMesh.m_shiness;
					strcpy (subMesh.m_textureName, srcMesh.m_textureName);
					subMesh.AllocIndexData (indexCount);

					indexCount = 0;
					node = node->GetPrev();
					dMesh::dListNode* nextNode;
					for (dMesh::dListNode* node1 = node; node1; node1 = nextNode) {
						nextNode = node1->GetNext();
						if (node1->GetInfo().m_textureHandle == 1) {
							memcpy (&subMesh.m_indexes[indexCount], node1->GetInfo().m_indexes, sizeof (unsigned short) * node1->GetInfo().m_indexCount);
							indexCount += node1->GetInfo().m_indexCount;
							skin->Remove (node1);
						}
					}
				}
			}
		}
	}
}
*/

BOOL CALLBACK ToolDlgProc(HWND hwnd, UINT Message, WPARAM wParam, LPARAM lParam)
{
    switch(Message)
    {
        case WM_COMMAND:
            switch(LOWORD(wParam))
            {
                case IDOK:
                    MessageBox(hwnd, "Hi!", "This is a message", 
                        MB_OK | MB_ICONEXCLAMATION);
                break;
                case IDCANCEL:
                    MessageBox(hwnd, "Bye!", "This is also a message", 
                        MB_OK | MB_ICONEXCLAMATION);
                break;
            }
        break;
        default:
            return FALSE;
    }
    return TRUE;
}


int Exporter::Execute (msModel *pModel)
{
	if (!pModel)
		return -1;
	//
	// choose filename
	//
	OPENFILENAME ofn;
	memset (&ofn, 0, sizeof (OPENFILENAME));

	char szFile[MS_MAX_PATH];
	char szFileTitle[MS_MAX_PATH];
	char szDefExt[32] = "xml";
	char szFilter[128] = "Newton Alchemedia (*.xml)\0*.xml\0\0";
	szFile[0] = '\0';
	szFileTitle[0] = '\0';

	ofn.lStructSize = sizeof (OPENFILENAME);
	ofn.lpstrDefExt = szDefExt;
	ofn.lpstrFilter = szFilter;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = MS_MAX_PATH;
	ofn.lpstrFileTitle = szFileTitle;
	ofn.nMaxFileTitle = MS_MAX_PATH;
	ofn.Flags = OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
	ofn.lpstrTitle = "Export Newton Alchemedia models";

	if (!::GetSaveFileName (&ofn))
		return 0;

	_ASSERTE (0);
#if 0

	dModel model;

	int rootBoneID = 0;
	// find out if any milkshape mesh is reference the root bone.
	int meshCount = msModel_GetMeshCount(pModel);
	for (int i = 0; (i < meshCount) && !rootBoneID; i ++) {
		msMesh* srcMesh = msModel_GetMeshAt (pModel, i);
		int vertexCount = msMesh_GetVertexCount (srcMesh);
		for (int j = 0; j < vertexCount; j ++) {
			msVertex* p = msMesh_GetVertexAt (srcMesh, j);
			int bondeIndex = msVertex_GetBoneIndex (p);
			if (bondeIndex == 0) {
				rootBoneID = 1;
				break;
			}
		}
	}


	// build the skeleton from all of the bones
	dBone* root = NULL;
	int boneCount = msModel_GetBoneCount(pModel);
	for (int i = 0; i < boneCount; i ++) {
		dBone* bone;
		msBone* msBone;

		bone = NULL;
		msBone = msModel_GetBoneAt (pModel, i);

		char name[256];
		
		if (!root) {
			root = new dBone (NULL);
			bone = root;
		} else {
			dBone* parent;
			msBone_GetParentName (msBone, name, sizeof (name) - 1);
			parent = root->Find(name);
			bone = new dBone (parent);
		}

		msBone_GetName (msBone, name, sizeof (name) - 1);
		bone->SetNameID (name);
		bone->m_boneID = i + rootBoneID;

		msVec3 position;
		msVec3 rotation;
		msBone_GetPosition (msBone, position);
		msBone_GetRotation (msBone, rotation);

		dMatrix matrix (dPitchMatrix (rotation[0]) * dYawMatrix (rotation[1]) * dRollMatrix (rotation[2]));
		matrix.m_posit = dVector (position[0], position[1], position[2], 1.0f);
		bone->SetMatrix (matrix);
	}

	// some modeler do no permit bone to be the part o fa mesh and also part a skinning bone of the same mesh
	// therefore if a mesh uses the root node, we will promote to a dummy root bone
	if (rootBoneID) {
		dBone* bone = new dBone (NULL);
		root->Attach(bone);
		root = bone;

		root->m_boneID = 0;
		root->SetNameID ("addedRoot");
	}

	model.AddSkeleton(root);
	root->Release();

	// find the material array, also make sure we are not using duplicated materials
	int uniqueArray[256];
	int indirectMaterial[256];
	int uniqueMaterials = 0;
	for (int i = 0; i < msModel_GetMaterialCount(pModel); i ++) {
		indirectMaterial[i] = i;
		uniqueArray[uniqueMaterials] = i;
		uniqueMaterials ++;

		msMaterial* material;
		material = msModel_GetMaterialAt (pModel, i);

		for (int j = 0; j < i; j ++) {
			msMaterial* material1;
			material1 = msModel_GetMaterialAt (pModel, j);

			if ((material1->Ambient[0] == material->Ambient[0]) && 
				(material1->Ambient[1] == material->Ambient[1]) && 
				(material1->Ambient[2] == material->Ambient[2]) && 
				(material1->Diffuse[0] == material->Diffuse[0]) && 
				(material1->Diffuse[0] == material->Diffuse[0]) && 
				(material1->Diffuse[1] == material->Diffuse[1]) && 
				(material1->Diffuse[2] == material->Diffuse[2]) && 
				(material1->Specular[0] == material->Specular[0]) && 
				(material1->Specular[1] == material->Specular[1]) && 
				(material1->Specular[2] == material->Specular[2])) {
				if (!strcmp (material1->szDiffuseTexture, material->szDiffuseTexture))  {
					indirectMaterial[i] = j;
					uniqueMaterials --;
					break;
				}
			}
		}
	}


	// create single mesh by concatenating all the sub part in the model
	int trianglesCount = 0;
	int *indices = new int [65536 * 3];
	SortVertex* vertex = new SortVertex[65536 * 3];
	VertexAttribute* attrib = new VertexAttribute[65536 * 3];
	memset (attrib, 0, 65536 * 3 * sizeof (VertexAttribute));

	// flatten all meshes
	for (int i = 0; i < meshCount; i ++) {
		msMesh* srcMesh = msModel_GetMeshAt (pModel, i);
		int triangles = msMesh_GetTriangleCount (srcMesh);
		int materialIndex = indirectMaterial[msMesh_GetMaterialIndex (srcMesh)];

		for (int j = 0; j < triangles; j ++) {
			msTriangle* face = msMesh_GetTriangleAt (srcMesh, j);

			word faceIndices[3];
			word normalIndices[3];
			msTriangle_GetVertexIndices (face, faceIndices);
			msTriangle_GetNormalIndices (face, normalIndices);
			for (int k = 0; k < 3; k ++) {
				msVec3 normal;

				msVertex* p = msMesh_GetVertexAt (srcMesh, faceIndices[k]);
				msMesh_GetVertexNormalAt (srcMesh, normalIndices[k], normal);

				vertex[(trianglesCount + j) * 3 + k].m_x = p->Vertex[0];
				vertex[(trianglesCount + j) * 3 + k].m_y = p->Vertex[1];
				vertex[(trianglesCount + j) * 3 + k].m_z = p->Vertex[2];
				vertex[(trianglesCount + j) * 3 + k].m_nx = normal[0];
				vertex[(trianglesCount + j) * 3 + k].m_ny = normal[1];
				vertex[(trianglesCount + j) * 3 + k].m_nz = normal[2];
				vertex[(trianglesCount + j) * 3 + k].m_u = p->u;
				vertex[(trianglesCount + j) * 3 + k].m_v = 1.0f - p->v;
				vertex[(trianglesCount + j) * 3 + k].m_attribute = dFloat ((trianglesCount + j) * 3 + k);

				attrib[(trianglesCount + j) * 3 + k].m_material = materialIndex;
				int bondeIndex = msVertex_GetBoneIndex (p);
				attrib[(trianglesCount + j) * 3 + k].m_weight[0] = 1.0f;
				attrib[(trianglesCount + j) * 3 + k].m_boneIndex[0] = bondeIndex + rootBoneID;

/*
				// note in milk shape msMesh_GetVertexExAt does really works so only 100 weigh is possible
				// at leas I do no know who to read and write that informations
				msVertexEx* w = msMesh_GetVertexExAt (srcMesh, faceIndices[k]);
				for (int i = 0; i < 3; i ++) {
					int weight = msVertexEx_GetBoneWeights (w, index);
					if ((weight > 0) && (weight < 100)) {
						attrib[(trianglesCount + j) * 3 + k].m_weight[index] = float (weight) / 100.0f;
						attrib[(trianglesCount + j) * 3 + k].m_boneIndex[index] = msVertexEx_GetBoneIndices(w, index);
						index ++;
					}
				} 
*/
			}
		}
		trianglesCount += triangles;
	}


	// now add the skinned mesh to the model, 
	dMesh* mesh = new dMesh (root->GetName());
	
	mesh->m_boneID = 0;
	mesh->m_hasBone = 1;
	model.AddMesh(mesh);
	mesh->Release();

	dMeshInstance& instance = model.m_meshList.GetLast()->GetInfo();
	instance.m_boneID = 0;

	// since the mesh is a child of teh root node we nee to remove the root node transform
	int vertexCount = dModel::dPackVertexArray (&vertex[0].m_x, sizeof (SortVertex) / sizeof (float), sizeof (SortVertex), trianglesCount * 3, indices);
	// move the mesh to to the first bone
	dMatrix rootMatrix (root->CalcGlobalMatrix().Inverse());
	rootMatrix.TransformTriplex(&vertex[0].m_x, sizeof (SortVertex), &vertex[0].m_x, sizeof (SortVertex), vertexCount);
	rootMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	rootMatrix.TransformTriplex(&vertex[0].m_nx, sizeof (SortVertex), &vertex[0].m_nx, sizeof (SortVertex), vertexCount);
	mesh->AllocVertexData (vertexCount);

	// add vertex, normals, uv and bones data to the mesh
	int weightsCount = 0;
	dSkinModifier* modifier = new dSkinModifier (mesh);
	dSkinModifier::dBoneVertexWeightData* weights = new dSkinModifier::dBoneVertexWeightData[vertexCount * 4];
	for (int i = 0; i < vertexCount; i ++) {
		mesh->m_vertex[i * 3 + 0] = vertex[i].m_x;
		mesh->m_vertex[i * 3 + 1] = vertex[i].m_y;
		mesh->m_vertex[i * 3 + 2] = vertex[i].m_z;

		mesh->m_normal[i * 3 + 0] = vertex[i].m_nx;
		mesh->m_normal[i * 3 + 1] = vertex[i].m_ny;
		mesh->m_normal[i * 3 + 2] = vertex[i].m_nz;

		mesh->m_uv[i * 2 + 0] = vertex[i].m_u;
		mesh->m_uv[i * 2 + 1] = vertex[i].m_v;

//		mesh->m_weighList->m_boneWeightIndex[i].m_index[0] = vertex[i].weightIndex[0];
//		mesh->m_weighList->m_vertexWeight[i][0] = vertex[i].weight[0];

		int index = int (vertex[i].m_attribute);
		weights[weightsCount].m_boneId = attrib[index].m_boneIndex[0];
		weights[weightsCount].m_weight = attrib[index].m_weight[0];
		weights[weightsCount].m_vertexIndex = i;
		weightsCount ++;
	}

	// add the skin modifier 
	modifier->SetBindingPose(instance, model, weights, weightsCount);
	instance.SetModifier(modifier);

	// add the triangles according to teh material they reference
	for (int i = 0; i < uniqueMaterials; i ++) {
		int faceCount;
		faceCount = 0;

		for (int j = 0; j < trianglesCount; j ++) {
			_ASSERTE (attrib[j * 3 + 0].m_material == attrib[j * 3 + 1].m_material);
			_ASSERTE (attrib[j * 3 + 0].m_material == attrib[j * 3 + 2].m_material);
			if (int (attrib[j * 3 + 0].m_material) == uniqueArray[i]) {
				faceCount ++;
			}
		}

		int base;
		dSubMesh* subMesh;
		subMesh = mesh->AddSubMesh();

		base = 0;
		subMesh->AllocIndexData (faceCount * 3);

		msMaterial* material;
		material = msModel_GetMaterialAt (pModel, uniqueArray[i]);
		subMesh->m_ambient = dVector (material->Ambient[0], material->Ambient[1], material->Ambient[2], 1.0f);
		subMesh->m_diffuse = dVector (material->Diffuse[0], material->Diffuse[1], material->Diffuse[2], 1.0f);
		subMesh->m_specular = dVector (material->Specular[0], material->Specular[1], material->Specular[2], 1.0f);
		subMesh->m_shiness = material->fShininess;

		subMesh->m_textureName[0] = 0;
		if (material->szDiffuseTexture[0]) {
			strcpy (subMesh->m_textureName, material->szDiffuseTexture);
		}

		for (int j = 0; j < trianglesCount; j ++) {
			if (int (attrib[j * 3 + 0].m_material) == uniqueArray[i]) {
				subMesh->m_indexes[base * 3 + 0] =  indices[j * 3 + 0];
				subMesh->m_indexes[base * 3 + 1] =  indices[j * 3 + 1];
				subMesh->m_indexes[base * 3 + 2] =  indices[j * 3 + 2];
				base ++;
			}
		}
	}

	// delete all intermediate data
	delete[] attrib; 
	delete[] vertex;
	delete[] indices;
	delete[] weights;

	// save the model
//	dMatrix matrix (GetIdentityMatrix());
	dMatrix matrix (dYawMatrix (-3.141592f));
	model.SaveCollada (szFile, matrix, 1.0f / SCALE);
#endif
	return 0;
}

