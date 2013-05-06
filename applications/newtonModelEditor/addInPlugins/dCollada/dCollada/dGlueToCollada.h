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

#ifndef  __SAVE_COLLADA_SCENE_H__
#define  __SAVE_COLLADA_SCENE_H__



#define D_PLANE_COLLISON_ID 100

class dMesh;
class dModel;
class dSubMesh;
class dLoaderContext;


class dSceneModelList: public dList<dModel*>
{
	public:
	dSceneModelList();
	~dSceneModelList();

	void AddModel (dModel* model);
	void RemoveModel (dModel* model);

	void ExportVisualScene (const char* fileName, const dMatrix& globalRotation, dFloat scale);
	void ImportVisualScene (const char* fileName, dLoaderContext& context, const dMatrix& globalRotation, dFloat scale);

	void ExportPhysicsScene (const char* fileName, NewtonWorld* world, const dMatrix& globalRotation, dFloat scale);
	void ImportPhysicsScene (const char* fileName, NewtonWorld* world, dLoaderContext& context, const dMatrix& globalRotation, dFloat scale);
};




#endif