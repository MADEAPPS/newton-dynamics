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

#ifndef _D_SKELETON_H_
#define _D_SKELETON_H_

#include <dAnimationStdAfx.h>
#include <dClassInfo.h>
#include <dModel.h>



class TiXmlElement;


class dBone: public dHierarchy<dBone>, virtual public dClassInfo  
{
	public:
	enum NodeType
	{
		m_sceneNode,
		m_bone,
	};

	dBone(dBone* parent);
	dBone(const dBone& initFrom, dBone* parent);
	

	const dMatrix& GetMatrix () const;
	void SetMatrix (const dMatrix& matrix);
	dMatrix CalcGlobalMatrix (const dBone* root = NULL) const;

	int GetBonesCount() const;
	void UpdateMatrixPalette (const dMatrix& parentMatrix, dMatrix* const matrixOut, int maxCount) const;

	void SetType (NodeType type);
	NodeType GetType () const;
	
	int GetBoneID() const;
	void SetBoneId(int id); 

	static void Load(const char* fileName, dList<dBone*>& list, dLoaderContext& context);
	static void Save(const char* fileName, const dList<dBone*>& list);

	dMatrix m_localMatrix;
	int m_boneID;
	int m_type;

	protected:
	~dBone(void);

	dAddRtti(dClassInfo);
};

#endif

