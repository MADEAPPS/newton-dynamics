/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dCustomBallAndSocket.h: interface for the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _D_CUSTOM_MODEL_LOAD_SAVE_H_
#define _D_CUSTOM_MODEL_LOAD_SAVE_H_

#include "dCustomJoint.h"

class dCustomJointSaveLoad: public dCustomAlloc
{
	public:
	dCustomJointSaveLoad(NewtonWorld* const world, FILE* const file)
		:m_file(file)
		,m_world(world)
	{
	}

	virtual ~dCustomJointSaveLoad() 
	{
	}

	virtual const char* GetUserDataName(const NewtonBody* const body) const = 0;
	virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const = 0;

	CUSTOM_JOINTS_API virtual const char* NextToken () const;
	CUSTOM_JOINTS_API virtual int LoadInt () const;
	CUSTOM_JOINTS_API virtual dFloat LoadFloat () const;
	CUSTOM_JOINTS_API virtual dVector LoadVector () const;
	CUSTOM_JOINTS_API virtual void LoadName (char* const name) const;
	
	CUSTOM_JOINTS_API virtual void Newline () const;
	CUSTOM_JOINTS_API virtual void SaveInt (const char* const token, int val) const;
	CUSTOM_JOINTS_API virtual void SaveFloat (const char* const token, dFloat val) const;
	CUSTOM_JOINTS_API virtual void SaveVector (const char* const token, const dVector& v) const;
	CUSTOM_JOINTS_API virtual void SaveName (const char* const token, const char* const name) const;

	CUSTOM_JOINTS_API virtual NewtonBody* Load();
	CUSTOM_JOINTS_API virtual void Save(NewtonBody* const rootbody);

	CUSTOM_JOINTS_API int FindBodyId(NewtonBody* const body) const;

	private:
	void LoadRigidBodyList(dTree<const NewtonBody*, int>& bodyList);

	void SaveRigidBodyList(dList<const NewtonBody*>& bodyList);
	void SaveRigidJointList(dList<const dCustomJoint*>& jointList);
	void ParseJoint(const dTree<NewtonBody*, const dString>& bodyMap);
	void GetBodiesAndJointsList (dList<const NewtonBody*>& bodylist, dList<const dCustomJoint*>& jointlist, NewtonBody* const rootbody);
	
	FILE* m_file;
	NewtonWorld* m_world;
	dTree<int, const NewtonBody*> m_bodyFilter;
	mutable  char m_token[128];
};



#endif 

