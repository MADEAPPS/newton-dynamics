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


#ifndef _D_MESH_H_
#define _D_MESH_H_

#include <dAnimationStdAfx.h>
#include <dClassInfo.h>

class TiXmlElement;
class dModel;


class dSubMesh
{
	public:
	dSubMesh ();
	~dSubMesh ();

	void AllocIndexData (int indexCount);
	int m_indexCount;
	unsigned *m_indexes;
	unsigned m_textureHandle;

	dFloat m_shiness;
	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	char m_textureName[D_NAME_STRING_LENGTH];
};


class dMesh: public dList<dSubMesh>, virtual public dClassInfo  
{
	public:
	dMesh(const char* name);

	void AllocVertexData (int vertexCount);
	void CalculateAABB (dVector& min, dVector& max) const;

#ifdef D_LOAD_SAVE_XML
	static void LoadXML(const char* fileName, dList<dMesh*>& list, dLoaderContext& context);
	static void SaveXML(const char* fileName, const dList<dMesh*>& list);
	TiXmlElement* ConvertToXMLNode () const;
#endif

	dSubMesh* AddSubMesh();

	virtual void GetName (char* nameOut) const;
	virtual void GetTextureName (const dSubMesh* subMesh, char* nameOut) const;

	virtual void ApplyGlobalScale (dFloat scale);
	virtual void ApplyGlobalTransform (const dMatrix& transform);

	protected:
	virtual ~dMesh();

	dAddRtti(dClassInfo);

	public:
	int m_hasBone;
	int m_boneID;
	int m_vertexCount;
	dFloat *m_uv;
	dFloat *m_vertex;
	dFloat *m_normal;
	char m_name[D_NAME_STRING_LENGTH];
};


class dMeshInstance
{
	public:
	class dModifier
	{
		public:
		dModifier() {};
		virtual ~dModifier() {}
		virtual dModifier* CreateCopy (const dMeshInstance& instance, const dModel& model) const {return NULL;}

		virtual void ApplyGlobalScale(dFloat scale) {}
		virtual void ApplyGlobalTransform (const dMatrix& transform) {}
	};

	dMeshInstance (dMesh* mesh)
		: m_boneID (0), m_mesh (mesh),m_modifier(NULL)
	{
		m_mesh->AddRef();
	}

	dMeshInstance (const dMeshInstance& meshIntansce)
		: m_boneID (0), m_mesh (meshIntansce.m_mesh),m_modifier(NULL)
	{
		m_mesh->AddRef();
	}

	~dMeshInstance ()
	{
		m_mesh->Release();
		if (m_modifier) {
			delete m_modifier;
		}
	}

	void SetModifier(dModifier* modifier)
	{
		m_modifier = modifier;
	}

	dModifier* GetModifier() const
	{
		return m_modifier;
	}

	bool IsIntance () const
	{
		return (m_boneID != m_mesh->m_boneID);
	}

	void SetBoneId(int id) {m_boneID = id;}
	int GetBoneId() const {return m_boneID;}

	int m_boneID;
	dMesh* m_mesh;
	dModifier* m_modifier;
};




#endif 

