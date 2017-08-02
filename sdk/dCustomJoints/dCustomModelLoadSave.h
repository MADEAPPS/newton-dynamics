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

	const char* NextToken (char* const buffer) const;
	int LoadInt () const;
	dFloat LoadFloat () const;
	dVector LoadVector () const;
	void LoadName (char* const name) const;
	
	void SaveInt (const char* const token, int val) const;
	void SaveFloat (const char* const token, dFloat val) const;
	void SaveVector (const char* const token, const dVector& v) const;



	virtual const char* GetBodyUniqueName(const NewtonBody* const body) const = 0;
	virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const = 0;

	CUSTOM_JOINTS_API virtual NewtonBody* Load();
	CUSTOM_JOINTS_API virtual void Save(NewtonBody* const rootbody);

	private:
	NewtonCollision* ParseCollisonShape();
	void ParseRigidBody(dTree<NewtonBody*, const dString>& bodyMap);
	void ParseJoint(const dTree<NewtonBody*, const dString>& bodyMap);
	void GetBodiesAndJointsList (dList<const NewtonBody*>& bodylist, dList<const dCustomJoint*>& jointlist, NewtonBody* const rootbody);
	

	FILE* m_file;
	NewtonWorld* m_world;
};



#endif 

