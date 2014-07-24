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

class DemoMesh;
class DemoEntityManager;

class DemoSubMesh
{
	public:
	DemoSubMesh ();
	~DemoSubMesh ();

	void Render() const;
	void AllocIndexData (int indexCount);
	void OptimizeForRender(const DemoMesh* const mesh) const;
	

	int m_indexCount;
	unsigned *m_indexes;
	unsigned m_textureHandle;

	dFloat m_shiness;
	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	dFloat m_opacity;
	//char m_textureName[D_NAME_STRING_LENGTH];
	dString  m_textureName;
};


class DemoMesh: public dList<DemoSubMesh>, virtual public dClassInfo  
{
	public:
	DemoMesh(const DemoMesh& mesh);
	DemoMesh(const char* const name);
	DemoMesh(NewtonMesh* const mesh);
	DemoMesh(const dScene* const scene, dScene::dTreeNode* const meshNode);
	DemoMesh(const char* const name, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2);
	DemoMesh(const char* const name, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize);

	using dClassInfo::operator new;
	using dClassInfo::operator delete;

	DemoSubMesh* AddSubMesh();
	void AllocVertexData (int vertexCount);

	virtual const dString& GetName () const;
	virtual const dString& GetTextureName (const DemoSubMesh* const subMesh) const;

    virtual void RenderTransparency () const;
	virtual void Render (DemoEntityManager* const scene);
	virtual void RenderNormals ();

	NewtonMesh* CreateNewtonMesh(NewtonWorld* const workd, const dMatrix& meshMatrix);

	protected:
	virtual ~DemoMesh();

	dAddRtti(dClassInfo,DOMMY_API);
	void  OptimizeForRender();
	void  ResetOptimization();
	void  SpliteSegment(dListNode* const node, int maxIndexCount);


	public:
	int m_vertexCount;
	dFloat* m_uv;
	dFloat* m_vertex;
	dFloat* m_normal;
	unsigned m_optimizedOpaqueDiplayList;
	unsigned m_optimizedTransparentDiplayList;		
	dString m_name;
};






#endif 

