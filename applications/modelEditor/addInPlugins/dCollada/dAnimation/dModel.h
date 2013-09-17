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


#ifndef _D_MODEL_H_
#define _D_MODEL_H_

#include <dAnimationStdAfx.h>
#include <dClassInfo.h>
#include <dMesh.h>


class dBone;
class dModel;
class dAnimationClip;
class dColladaSceneExportContext;


#define XML_HEADER "newton 2.0 file format"


class dLoaderContext
{
	public:
	dLoaderContext() {}
	virtual ~dLoaderContext() {}

	virtual dModel* CreateModel ();
	virtual dMesh* CreateMesh (const char* name);
	virtual dBone* CreateBone (dBone* parent);
	virtual void LoaderFixup (dModel* model);

	virtual void SetUserData (const NewtonBody* body, dModel* model, const char* bindkeyword);
	virtual void SetTransformCallback (const NewtonBody* body, const char* bindkeyword);
	virtual void SetForceAndTorqueCallback (const NewtonBody* body, const char* bindkeyword);
	virtual void SetDestructorCallback (const NewtonBody* body, const char* bindkeyword);
};

#define MODEL_COMPONENT_NAME_SIZE 32


//class dModel: public dRefCounter  
class dModel: public dClassInfo
{
	public:
	dModel();
	virtual ~dModel();

	virtual void InitFromModel (const dModel& source);

	void AddMesh(dMesh* mesh);
	void RemoveMesh(dMesh* mesh);

	void AddAnimation(dAnimationClip* clip);
	void RemoveAnimation(dAnimationClip* mesh);

	void AddSkeleton (dBone* rootBone);
	void RemoveSkeleton(dBone* rootBone);

	void UpdateMatrixPalette (const dMatrix& parentMatrix, dMatrix* const matrixOut, int maxCount) const;
	void BindMeshToBonesByName () const;

	int GetBoneCount() const;
	dBone* FindBone (int index) const;
	dBone* FindBone (const char* name) const;
	dMesh* FindMesh (const char* name) const;
	void GetBoneList (dList<dBone*>& bodeList) const;

	dMeshInstance* FindMeshInstance (dMesh* mesh) const;
	dMeshInstance* FindMeshInstance (int boneIndex) const;
	dAnimationClip* FindAnimationClip (const char* name) const;

	void ApplyGlobalScale (dFloat scale);
	void ApplyGlobalTransform (const dMatrix& matrix);


	static void GetFileName (const char* pathNamenode, char* name);
	static void GetFilePath (const char* pathNamenode, char* name);
	static int dPackVertexArray (dFloat* vertexList, int compareElements, int strideInBytes, int vertexCount, int* indexListOut);

#ifdef D_LOAD_SAVE_XML
	static int StringToInts (const char* string, int* ints);
	static int StringToFloats (const char* string, dFloat* floats);
	static void IntsToString (char* string, const int* ints, int count);
	static void FloatsToString (char* string, const dFloat* floats, int count);
	virtual void LoadXML (const char* name, dLoaderContext& context); 
	virtual void SaveXML (const char* name, bool exportSkeleton = true, bool exportMesh = true, bool exportAnimations = true); 
#endif
//	virtual void SaveCollada (const char *fileName, const dMatrix& globalRotation, dFloat scale) const;
//	virtual void LoadCollada (const char* fileName, dLoaderContext& context, const dMatrix& globalRotation, dFloat scale); 

//	dMatrix m_matrix;
	char m_name[D_NAME_STRING_LENGTH];
	dList<dBone*> m_skeleton;
	dList<dMeshInstance> m_meshList;
	dList<dAnimationClip*> m_animations;

	dAddRtti(dClassInfo);
};


#endif
