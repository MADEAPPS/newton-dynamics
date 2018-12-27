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


#ifndef _D_MESH_H_
#define _D_MESH_H_

class DemoMesh;
class DemoEntityManager;

class DemoMeshInterface: public dClassInfo  
{
	public:
	DemoMeshInterface();
	~DemoMeshInterface();
	const dString& GetName () const;

	bool GetVisible () const;
	void SetVisible (bool visibilityFlag);

	virtual void RenderTransparency () const = 0;
	virtual void Render (DemoEntityManager* const scene) = 0;
	virtual void RenderNormals () = 0;
	virtual NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix) = 0;

	dAddRtti(dClassInfo,DOMMY_API);

	dString m_name;
	bool m_isVisible;
};

class DemoSubMesh
{
	public:
	DemoSubMesh ();
	~DemoSubMesh ();

	void Render() const;
	void AllocIndexData (int indexCount);
	void OptimizeForRender(const DemoMesh* const mesh) const;
	
	void SetOpacity(dFloat opacity);

	int m_indexCount;
	unsigned *m_indexes;
	unsigned m_textureHandle;

	dFloat m_shiness;
	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	dFloat m_opacity;
	dString  m_textureName;
};

class DemoMesh: public DemoMeshInterface, public dList<DemoSubMesh>
{
	public:
	DemoMesh(const DemoMesh& mesh);
	DemoMesh(const char* const name);
	DemoMesh(NewtonMesh* const mesh);
	DemoMesh(const dScene* const scene, dScene::dTreeNode* const meshNode);
	DemoMesh(const char* const name, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat opacity = 1.0f, const dMatrix& uvMatrix = dGetIdentityMatrix());
	DemoMesh(const char* const name, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize);

	using dClassInfo::operator new;
	using dClassInfo::operator delete;

	DemoSubMesh* AddSubMesh();
	void AllocVertexData (int vertexCount);

	virtual const dString& GetTextureName (const DemoSubMesh* const subMesh) const;

    virtual void RenderTransparency () const;
	virtual void Render (DemoEntityManager* const scene);
	virtual void RenderNormals ();

	void OptimizeForRender();
	virtual NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	protected:
	virtual ~DemoMesh();

	dAddRtti (DemoMeshInterface, DOMMY_API);
	
	void  ResetOptimization();
	void  SpliteSegment(dListNode* const node, int maxIndexCount);

	public:
	int m_vertexCount;
	dFloat* m_uv;
	dFloat* m_vertex;
	dFloat* m_normal;
	unsigned m_optimizedOpaqueDiplayList;
	unsigned m_optimizedTransparentDiplayList;		
};

class DemoBezierCurve: public DemoMeshInterface
{
	public:
	DemoBezierCurve (const dBezierSpline& curve);
	DemoBezierCurve(const dScene* const scene, dScene::dTreeNode* const meshNode);

	int GetRenderResolution () const;
	void SetRenderResolution (int breaks);

	virtual void RenderTransparency () const;
	virtual void Render (DemoEntityManager* const scene);
	virtual void RenderNormals ();

	virtual NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	dBezierSpline m_curve;
	int m_renderResolution;

	dAddRtti (DemoMeshInterface, DOMMY_API);
};

class DemoSkinMesh: public DemoMeshInterface
{
	public:
	DemoSkinMesh(DemoMesh* const mesh);
	~DemoSkinMesh();

	void Render (DemoEntityManager* const scene);
	void RenderNormals ();
	void RenderTransparency () const;
	NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	protected: 
	DemoMesh* m_mesh;
};

#endif 


