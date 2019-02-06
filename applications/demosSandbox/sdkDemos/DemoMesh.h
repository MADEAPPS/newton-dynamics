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
class DemoEntity;
class ShaderPrograms;
class DemoEntityManager;

class DemoSubMesh
{
	public:
	DemoSubMesh();
	~DemoSubMesh();

	void Render() const;
	void AllocIndexData(int indexCount);
	void OptimizeForRender(const DemoMesh* const mesh) const;

	void SetOpacity(dFloat opacity);

	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	dString  m_textureName;
	dFloat m_opacity;
	dFloat m_shiness;
	int m_indexCount;
	unsigned m_shader;
	unsigned m_textureHandle;
	unsigned *m_indexes;
};

class DemoMeshInterface: public dClassInfo  
{
	public:
	DemoMeshInterface();
	~DemoMeshInterface();
	const dString& GetName () const;

	bool GetVisible () const;
	void SetVisible (bool visibilityFlag);

	virtual DemoMeshInterface* Clone(DemoEntity* const owner) { dAssert(0); return NULL; }

	virtual void RenderTransparency () const = 0;
	virtual void Render (DemoEntityManager* const scene) = 0;
	virtual void RenderNormals () = 0;
	virtual NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix) = 0;

	dAddRtti(dClassInfo,DOMMY_API);

	dString m_name;
	bool m_isVisible;
};

class DemoMesh: public DemoMeshInterface, public dList<DemoSubMesh>
{
	public:
	DemoMesh(const DemoMesh& mesh, const ShaderPrograms& shaderCache);
	DemoMesh(const char* const name, const ShaderPrograms& shaderCache);
	DemoMesh(NewtonMesh* const mesh, const ShaderPrograms& shaderCache);
	DemoMesh(const dScene* const scene, dScene::dTreeNode* const meshNode, const ShaderPrograms& shaderCache);
	DemoMesh(const char* const name, const ShaderPrograms& shaderCache, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat opacity = 1.0f, const dMatrix& uvMatrix = dGetIdentityMatrix());
	DemoMesh(const char* const name, const ShaderPrograms& shaderCache, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize);

	virtual DemoMeshInterface* Clone(DemoEntity* const owner) { AddRef(); return this;}

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

class DemoSkinMesh: public DemoMeshInterface
{
	public:
	struct dWeightBoneIndex
	{
		int m_boneIndex[4];
	};

	DemoSkinMesh(const DemoSkinMesh& clone, DemoEntity* const owner);
	DemoSkinMesh(dScene* const scene, DemoEntity* const owner, dScene::dTreeNode* const meshNode, const dTree<DemoEntity*, dScene::dTreeNode*>& boneMap, const ShaderPrograms& shaderCache);
	~DemoSkinMesh();

	void Render (DemoEntityManager* const scene);
	void RenderNormals ();
	void RenderTransparency () const;
	NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	protected: 
	virtual DemoMeshInterface* Clone(DemoEntity* const owner);
	int CalculateMatrixPalette(dMatrix* const bindMatrix) const;
	void ConvertToGlMatrix(int count, const dMatrix* const bindMatrix, GLfloat* const glMatrices) const;
	dGeometryNodeSkinClusterInfo* FindSkinModifier(dScene* const scene, dScene::dTreeNode* const meshNode) const;
	void OptimizeForRender(const DemoSubMesh& segment, const dVector* const pointWeights, const dWeightBoneIndex* const pointSkinBone) const;

	DemoMesh* m_mesh;
	DemoEntity* m_entity; 
	dMatrix* m_bindingMatrixArray;
	int m_nodeCount; 
	int m_shader;
};

class DemoBezierCurve: public DemoMeshInterface
{
	public:
	DemoBezierCurve(const dBezierSpline& curve);
	DemoBezierCurve(const dScene* const scene, dScene::dTreeNode* const meshNode);

	int GetRenderResolution() const;
	void SetRenderResolution(int breaks);

	virtual void RenderTransparency() const;
	virtual void Render(DemoEntityManager* const scene);
	virtual void RenderNormals();

	virtual NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	dBezierSpline m_curve;
	int m_renderResolution;

	dAddRtti(DemoMeshInterface, DOMMY_API);
};


#endif 


