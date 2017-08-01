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



class dSaveLoad : public dCustomAlloc
{
	public:
	class BodyJointPair;

	dSaveLoad(NewtonWorld* const world)
		:m_world(world) 
	{
	}

	virtual ~dSaveLoad() 
	{
	}

	virtual const char* GetBodyUniqueName(const NewtonBody* const body) const = 0;
	virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const = 0;

	CUSTOM_JOINTS_API virtual void Load(const char* const name);
	CUSTOM_JOINTS_API virtual void Save(NewtonBody* const rootbody, const char* const name);

	private:
	NewtonCollision* ParseCollisonShape(FILE* const file);
	void GetBodyList (dList<BodyJointPair>& list, NewtonBody* const rootbody);
	void ParseRigidBody(FILE* const file, dTree<NewtonBody*, const dString>& bodyMap);
	void ParseJoint(FILE* const file, const dTree<NewtonBody*, const dString>& bodyMap);

	NewtonWorld* m_world;
};



#endif 

