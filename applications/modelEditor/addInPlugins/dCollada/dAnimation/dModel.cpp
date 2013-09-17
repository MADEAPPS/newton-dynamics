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

#include <dAnimationStdAfx.h>
#include <dBone.h>
#include <dMesh.h>
#include <dModel.h>
#include <dAnimationClip.h>

#ifdef D_LOAD_SAVE_XML
#include <tinyxml.h>
#endif


dInitRtti(dModel);


dModel* dLoaderContext::CreateModel ()
{
	return new dModel();
}


dMesh* dLoaderContext::CreateMesh (const char* name)
{
	return new dMesh (name);
}

dBone* dLoaderContext::CreateBone (dBone* parent)
{
	return new dBone (parent);
}

void dLoaderContext::LoaderFixup (dModel* model)
{
}

void dLoaderContext::SetUserData (const NewtonBody* body, dModel* model, const char* bindkeyword)
{

}

void dLoaderContext::SetTransformCallback (const NewtonBody* body, const char* bindkeyword)
{

}

void dLoaderContext::SetForceAndTorqueCallback (const NewtonBody* body, const char* bindkeyword)
{

}

void dLoaderContext::SetDestructorCallback (const NewtonBody* body, const char* bindkeyword)
{

}



dModel::dModel(void)
	:dClassInfo()
{
//	int xxx = dModel::GetRttiType();
//	xxx = dClassInfo::GetRttiType();
//	xxx = dClassInfo::GetTypeId();
//	xxx = GetTypeId();
//	bool sss = IsType (dModel::GetRttiType());
//	sss = IsType (dClassInfo::GetRttiType());
	strcpy (m_name, "scene_model");;
}


dModel::~dModel(void)
{
	for (dList<dBone*>::dListNode* list = m_skeleton.GetFirst(); list; list = list->GetNext()) {
		list->GetInfo()->Release();
	}

	for (dList<dAnimationClip*>::dListNode* node = m_animations.GetFirst(); node; node = node->GetNext()) {
		node->GetInfo()->Release();
	}
}


void dModel::InitFromModel (const dModel& source)
{
	while (m_skeleton.GetCount()) {
		RemoveSkeleton(m_skeleton.GetFirst()->GetInfo());
	}
	while (m_meshList.GetCount()) {
		RemoveMesh(m_meshList.GetFirst()->GetInfo().m_mesh);
	}

	while (m_animations.GetCount()) {
		RemoveAnimation(m_animations.GetFirst()->GetInfo());
	}


	for (dList<dBone*>::dListNode* node = source.m_skeleton.GetFirst(); node; node = node->GetNext()) {
		int stack = 1;
		dBone* pool[64];
		dBone* parentBones[64];

		parentBones[0] = NULL;
		dBone* rootBone = NULL;
		pool[0] = node->GetInfo();
		
		while (stack) {
			stack --;
			dBone* parent = parentBones[stack];
			dBone* sourceBone = pool[stack];
			dBone* bone = new dBone (*sourceBone, parent);
			if (!rootBone) {
				rootBone = bone;
			}

			for (sourceBone = sourceBone->GetChild(); sourceBone; sourceBone = sourceBone->GetSibling()) {
				pool[stack] = sourceBone;
				parentBones[stack] = bone;
				stack ++;
			}
		}
	
		AddSkeleton(rootBone);
		rootBone->Release();
	}



	for (dList<dAnimationClip*>::dListNode* node = source.m_animations.GetFirst(); node; node = node->GetNext()) {
		AddAnimation(node->GetInfo());
	}


	for (dList<dMeshInstance>::dListNode* node = source.m_meshList.GetFirst(); node; node = node->GetNext()) { 
		AddMesh(node->GetInfo().m_mesh);
		dMeshInstance& instance = m_meshList.GetLast()->GetInfo();
		instance.m_boneID = node->GetInfo().m_boneID;
		if (node->GetInfo().GetModifier()) {
			instance.SetModifier(node->GetInfo().GetModifier()->CreateCopy (instance, *this));
		}
	}
}

//void dModel::SetMatrix (const dMatrix& matrix)
//{
//	m_matrix = matrix;
//}


void dModel::AddMesh(dMesh* mesh)
{	
	m_meshList.Append(mesh);
}


void dModel::RemoveMesh(dMesh* mesh)
{
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		if (mesh == node->GetInfo().m_mesh) {
			m_meshList.Remove(node);
			return;
		}
	}
}

void dModel::AddAnimation(dAnimationClip* anim)
{
//	dList<dAnimationClip*>& clip = m_animations.Append()->GetInfo();
//	strncpy (clip.m_name, name, MODEL_COMPONENT_NAME_SIZE - 1);
//	clip.m_data = anim;
	m_animations.Append(anim);
	anim->AddRef();
}

void dModel::RemoveAnimation(dAnimationClip* anim)
{
	for (dList<dAnimationClip*>::dListNode* node = m_animations.GetFirst(); node; node = node->GetNext()) {
		if (anim == node->GetInfo()) {
			node->GetInfo()->Release();
			m_animations.Remove(node);
			break;
		}
	}
}

void dModel::AddSkeleton (dBone* rootBone)
{
//	ModelComponent<dList<dBone*> >& boneList = m_skeleton.Append()->GetInfo();
//	boneList.m_data.Append (rootBone);
	m_skeleton.Append(rootBone);
	rootBone->AddRef();
}


void dModel::RemoveSkeleton(dBone* rootBone)
{
	for (dList<dBone*>::dListNode* list = m_skeleton.GetFirst(); list; list = list->GetNext()) {
		if (list->GetInfo() == rootBone) {
			rootBone->Release();
			m_skeleton.Remove(list);
			break;
		}
	}
}



dAnimationClip* dModel::FindAnimationClip (const char* name) const
{
	_ASSERTE (0);
	return NULL;
}


void dModel::BindMeshToBonesByName () const
{
	_ASSERTE (0);
/*
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		
		dMesh* mesh;
		dBone* bone;
		mesh = node->GetInfo().m_mesh;
		bone = FindBone (mesh->m_name);
		if (bone) {
			bone->m_boneID = bone->m_boneID;
		} else {
			bone->m_boneID = 0;
		}
	}
*/
}

int dModel::GetBoneCount() const
{
	int count = 0;
	for (dList<dBone*>::dListNode* list = m_skeleton.GetFirst(); list; list = list->GetNext()) {
			count += list->GetInfo()->GetBonesCount();
	}
	return count;
}

void dModel::UpdateMatrixPalette (const dMatrix& parentMatrix, dMatrix* const matrixOut, int maxCount) const
{
	for (dList<dBone*>::dListNode* list = m_skeleton.GetFirst(); list; list = list->GetNext()) {
			list->GetInfo()->UpdateMatrixPalette (parentMatrix, matrixOut, maxCount);
	}
}

void dModel::GetFilePath (const char* pathNamenode, char* name)
{
	for (int i = int (strlen (pathNamenode)); i >= 0; i --) {
		if ((pathNamenode[i] == '/') || (pathNamenode[i] == '\\')) {
			strcpy (name, pathNamenode); 
			name[i + 1] = 0;
			return ;
		}
	}
	name[0] = 0;
}

void dModel::GetFileName (const char* pathNamenode, char* name)
{
	for (int i = int (strlen (pathNamenode)); i >= 0; i --) {
		if ((pathNamenode[i] == '/') || (pathNamenode[i] == '\\')) {
			strcpy (name, &pathNamenode[i + 1]); 
			return ;
		}
	}
	strcpy (name, pathNamenode);
}





static int SortVertexArray (const void *A, const void *B) 
{
	const dFloat* vertexA = (dFloat*) A;
	const dFloat* vertexB = (dFloat*) B;

	if (vertexA[0] < vertexB[0]) {
		return -1;
	} else if (vertexA[0] > vertexB[0]) {
		return 1;
	} else {
		return 0;
	}
}


int dModel::dPackVertexArray (dFloat* vertexList, int compareElements, int strideInBytes, int vertexCount, int* indexListOut)
{
	int stride;
	int indexCount;
	dFloat errorTol;

	stride = strideInBytes / sizeof (dFloat);
	errorTol = dFloat (1.0e-4f);

	dFloat* array;
	array = new dFloat[(stride + 2) * vertexCount];

	for (int i = 0; i < vertexCount; i ++) {
		memcpy (&array[i * (stride + 2)], &vertexList[i * stride], strideInBytes);
		array[i * (stride + 2) + stride + 0] = dFloat(i);
		array[i * (stride + 2) + stride + 1] = 0.0f;
	}

	qsort(array, vertexCount, (stride + 2) * sizeof (dFloat), SortVertexArray);
	indexCount = 0;
	for (int i = 0; i < vertexCount; i ++) {
		int index;
		index = i * (stride + 2);
		if (array[index + stride + 1] == 0.0f) {
			dFloat window;
			window = array[index] + errorTol; 
			for (int j = i + 1; j < vertexCount; j ++) {
				int index2;
				index2 = j * (stride + 2);
				if (array[index2] >= window) {
					break;
				}
				if (array[index2 + stride + 1] == 0.0f) {
					int k;
					for (k = 0; k < compareElements; k ++) {
						dFloat error;
						error = array[index + k] - array[index2+ k];
						if (dAbs (error) >= errorTol) {
							break;
						}
					}
					if (k >= compareElements) {
						int m;
						m = int (array[index2 + stride]);
						memcpy (&array[indexCount * (stride + 2)], &array[index2], sizeof (dFloat) * stride);
						indexListOut[m] = indexCount;
						array[index2 + stride + 1] = 1.0f;
					}
				}
			}
			int m;
			m = int (array[index + stride]);
			memcpy (&array[indexCount * (stride + 2)], &array[index], sizeof (dFloat) * stride);
			indexListOut[m] = indexCount;
			array[indexCount * (stride + 2) + stride + 1] = 1.0f;
			indexCount ++;
		}
	}

	for (int i = 0; i < indexCount; i ++) {
		memcpy (&vertexList[i * stride], &array[i * (stride + 2)], sizeof (dFloat) * stride);
	}

	delete[] array;
	return indexCount;
}

dBone* dModel::FindBone (int index) const
{
	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		for (dBone* bone = node->GetInfo()->GetFirst(); bone; bone = bone->GetNext()) {
			if (bone->GetBoneID() == index) {
				return bone;
			}
		}
	}

	return NULL;
}

dBone* dModel::FindBone (const char* name) const
{
	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		dBone* bone;
		bone = node->GetInfo()->Find(name);
		if (bone) {
			return bone;
		}
	}
	return NULL;
}

void dModel::GetBoneList (dList<dBone*>& boneList) const
{
	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		dBone* rootbone = node->GetInfo();
		for (dBone* bone = rootbone->GetFirst(); bone; bone = (dBone*) bone->GetNext()) {
			 boneList.Append(bone);
		}
	}
}

dMesh* dModel::FindMesh (const char* name) const
{
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		if (!strcmp (node->GetInfo().m_mesh->m_name, name)) {
			return node->GetInfo().m_mesh;
		}
	}
	return NULL;
}

dMeshInstance* dModel::FindMeshInstance (int boneID) const
{
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		if (node->GetInfo().m_boneID == boneID) {
			return &node->GetInfo();
		}
	}
	return NULL;
}

dMeshInstance* dModel::FindMeshInstance (dMesh* mesh) const
{
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		if (node->GetInfo().m_mesh == mesh) {
			return &node->GetInfo();
		}
	}
	return NULL;
}


void dModel::ApplyGlobalScale (dFloat scale)
{
	_ASSERTE (0);
/*
	dMatrix scaleMatrix (GetIdentityMatrix());
	scaleMatrix[0][0] = scale;
	scaleMatrix[1][1] = scale;
	scaleMatrix[2][2] = scale;
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		dMesh* geom = node->GetInfo().m_mesh;
		if (node->GetInfo().m_boneID == geom->m_boneID) {
			scaleMatrix.TransformTriplex (geom->m_vertex, 3 * sizeof (dFloat), geom->m_vertex, 3 * sizeof (dFloat), geom->m_vertexCount);
		}
		if (node->GetInfo().GetModifier()) {
			node->GetInfo().GetModifier()->ApplyGlobalScale(scale);
		}
	}


	int stackIndex;
	dBone* stack[1024];

	stackIndex = 0;
	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		for (dList<dBone*>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			stack[stackIndex] = node->GetInfo();
			stackIndex ++;
		}
	}

	dMatrix transformInv (scaleMatrix);
	transformInv[0][0] = 1.0f/transformInv[0][0];
	transformInv[1][1] = 1.0f/transformInv[1][1];
	transformInv[2][2] = 1.0f/transformInv[2][2];

	while (stackIndex) {
		dBone* bone;
		stackIndex --;
		bone = stack[stackIndex];

		dMatrix matrix (transformInv * bone->GetMatrix() * scaleMatrix);
		bone->SetMatrix (matrix);

		for (bone = bone->GetChild(); bone; bone = bone->GetSibling()) {
			stack[stackIndex] = bone;
			stackIndex ++;
		}
	}
*/
}





void dModel::ApplyGlobalTransform (const dMatrix& transform)
{
	_ASSERTE (0);
/*
	dMatrix rotation (transform);
	rotation.m_posit =dVector (0.0f, 0.0f, 0.0f, 1.0f);
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		dMesh* geom = node->GetInfo().m_mesh;

		if (node->GetInfo().m_boneID == geom->m_boneID) {
			transform.TransformTriplex (geom->m_vertex, 3 * sizeof (dFloat), geom->m_vertex, 3 * sizeof (dFloat), geom->m_vertexCount);
			rotation.TransformTriplex (geom->m_normal, 3 * sizeof (dFloat), geom->m_normal, 3 * sizeof (dFloat), geom->m_vertexCount);
		}
		if (node->GetInfo().GetModifier()) {
			node->GetInfo().GetModifier()->ApplyGlobalTransform(transform);
		}
	}

	int stackIndex;
	dBone* stack[1024];

	stackIndex = 0;
	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		for (dList<dBone*>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			stack[stackIndex] = node->GetInfo();
			stackIndex ++;
		}
	}

	dMatrix transformInv (transform.Inverse());
	while (stackIndex) {
		dBone* bone;
		stackIndex --;
		bone = stack[stackIndex];

		dMatrix matrix (transformInv * bone->GetMatrix() * transform);
		bone->SetMatrix (matrix);

		for (bone = bone->GetChild(); bone; bone = bone->GetSibling()) {
			stack[stackIndex] = bone;
			stackIndex ++;
		}
	}
*/
}


#ifdef D_LOAD_SAVE_XML

void dModel::IntsToString (char* string, const int* ints, int count)
{
	for (int i =0; i < (count - 1); i ++) {
		sprintf (string, "%d ", ints[i]);
		string += strlen (string);
	}
	sprintf (string, "%d", ints[count - 1]);

}

void dModel::FloatsToString (char* string, const dFloat* floats, int count)
{
	for (int i =0; i < (count - 1); i ++) {
		sprintf (string, "%f ", floats[i]);
		string += strlen (string);
	}
	sprintf (string, "%f", floats[count - 1]);
}

int dModel::StringToInts (const char* string, int* ints)
{
	int count;
	count = 0;

	do {
		ints[count] = atoi (string);
		count ++;
		while (string[0] && !isspace(string[0])) string ++;
		while (string[0] && isspace(string[0])) string ++;
	} while (string[0]);

	return count;
}

int dModel::StringToFloats (const char* string, dFloat* floats)
{
	int count;
	count = 0;

	do {
		floats[count] = dFloat (atof (string));
		count ++;
		while (string[0] && !isspace(string[0])) string ++;
		while (string[0] && isspace(string[0])) string ++;
	} while (string[0]);

	return count;
}

void dModel::SaveXML (const char* modelName, bool exportSkeleton, bool exportMesh, bool exportAnimations)
{
	TiXmlText* header;
	TiXmlElement *root;
	TiXmlDeclaration* decl;

	char mdlName[256];
	strcpy (mdlName, modelName);
	*strrchr (mdlName, '.') = 0;
	strcat (mdlName, ".mdl");

	TiXmlDocument out (mdlName);
	decl = new TiXmlDeclaration( "1.0", "", "" );
	out.LinkEndChild( decl );

	root = new TiXmlElement( "root" );
	out.LinkEndChild(root);

	header = new TiXmlText (XML_HEADER);
	root->LinkEndChild(header);


	for (ModelComponentList<dList<dMeshInstance> >::dListNode* list = m_meshList.GetFirst(); list; list = list->GetNext()) {
		char name[256];
		TiXmlElement *mesh;
		strcpy (name, list->GetInfo().m_name);

		mesh = new TiXmlElement( "mesh" );
		root->LinkEndChild(mesh);

		header = new TiXmlText (list->GetInfo().m_name);
		mesh->LinkEndChild(header);
	}

	for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
		TiXmlElement *skeleton;
		skeleton = new TiXmlElement( "skeleton" );
		root->LinkEndChild(skeleton);

		header = new TiXmlText (list->GetInfo().m_name);
		skeleton->LinkEndChild(header);
	}

	for (ModelComponentList<dAnimationClip*>::dListNode* node = m_animations.GetFirst(); node; node = node->GetNext()) {
		TiXmlElement *anim;
		anim = new TiXmlElement( "animation" );
		root->LinkEndChild(anim);
		header = new TiXmlText (node->GetInfo().m_name);
		anim->LinkEndChild(header);
	}


	if (exportSkeleton && m_skeleton.GetCount()) {
		char pathName[256];
		dModel::GetFilePath (modelName, pathName);
		for (dList<dBone*>::dListNode* node = m_skeleton.GetFirst(); node; node = node->GetNext()) {
			char name[256];
			strcpy (name, pathName);
			strcat (name, list->GetInfo().m_name);
			dBone::Save (name, list->GetInfo().m_data);
		}
	}

	if (exportMesh && m_meshList.GetCount()) {

		char pathName[256];
		dModel::GetFilePath (modelName, pathName);
		for (ModelComponentList<dList<dMeshInstance> >::dListNode* list = m_meshList.GetFirst(); list; list = list->GetNext()) {
			char name[256];
			strcpy (name, pathName);
			strcat (name, list->GetInfo().m_name);
			dMesh::SaveXML (name, list->GetInfo().m_data);
		}
	}

	if (exportAnimations && m_animations.GetCount()) {

		char pathName[256];
		dModel::GetFilePath (modelName, pathName);
		for (ModelComponentList<dAnimationClip*>::dListNode* node = m_animations.GetFirst(); node; node = node->GetNext()) {
			char name[256];
			strcpy (name, pathName);
			strcat (name, node->GetInfo().m_name);
			node->GetInfo().m_data->SaveXML (name);
		}

	}
	out.SaveFile (mdlName);
}


void dModel::LoadXML (const char* name, dLoaderContext& context)
{
	_ASSERTE (0);

	const TiXmlElement* root;
	TiXmlDocument doc (name);
	doc.LoadFile();

	char path[256];
	char* tmpName;
	strcpy (path, name);
	tmpName = strrchr (path, '/');
	if (!tmpName) {
		tmpName = strrchr (path, '\\');
	} 
	if (tmpName) {
		tmpName[0] = 0;
	}

	root = doc.RootElement();
	
	for (TiXmlElement* skeleton = (TiXmlElement*)root->FirstChildElement ("skeleton"); skeleton; skeleton = (TiXmlElement*)root->NextSibling ("skeleton")) {
		char pathName[256];
		ModelComponent<dList<dBone*> >& data = m_skeleton.Append()->GetInfo();
		strncpy (data.m_name, skeleton->GetText(), sizeof (data.m_name) - 1);

		strcpy (pathName, path);
		strcat (pathName, "/");
		strcat (pathName, data.m_name);
		dBone::Load (pathName, data.m_data, context);
	}

	for (TiXmlElement* meshes = (TiXmlElement*)root->FirstChildElement ("mesh"); meshes; meshes = (TiXmlElement*)root->NextSibling ("mesh")) {
		char pathName[256];
		ModelComponent<dList<dMeshInstance> >& data = m_meshList.Append()->GetInfo();

		strncpy (data.m_name, meshes->GetText(), sizeof (data.m_name) - 1);
		strcpy (pathName, path);
		strcat (pathName, "/");
		strcat (pathName, data.m_name);
		dMesh::LoadXML (pathName, data.m_data, context);
	}

	for (ModelComponentList<dList<dMeshInstance> >::dListNode* list = m_meshList.GetFirst(); list; list = list->GetNext()) {
		for (dList<dMeshInstance>::dListNode* node = list->GetInfo().m_data.GetFirst(); node; node = node->GetNext()) { 
			_ASSERTE (0);
//			if (node->GetInfo()->GetType() == dMesh::D_SKIN_MESH) {
//				node->GetInfo()->m_weighList->SetBindingPose(node->GetInfo(), *this); 
//			}
		}
	}

	for (TiXmlElement* anim = (TiXmlElement*)root->FirstChildElement("animation"); anim; anim = (TiXmlElement*)anim->NextSibling("animation")) {
		char pathName[256];
		ModelComponent<dAnimationClip*>& data = m_animations.Append()->GetInfo();

		strncpy (data.m_name, anim->GetText(), sizeof (data.m_name) - 1);
		strcpy (pathName, path);
		strcat (pathName, "/");
		strcat (pathName, data.m_name);
		
		data.m_data = new dAnimationClip ();
		data.m_data->LoadXML (pathName);
	}

	context.LoaderFixup (this);
}

#endif

