/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgMeshEffect_H__
#define __dgMeshEffect_H__

#include <dgRefCounter.h>

class dgWorld;
class dgMeshEffect;
class dgCollisionInstance;

#define DG_MESH_EFFECT_PRECISION_BITS		48
#define DG_MESH_EFFECT_PRECISION_SCALE		dgFloat64(dgInt64(1)<<DG_MESH_EFFECT_PRECISION_BITS)
#define DG_MESH_EFFECT_PRECISION_SCALE_INV	(dgFloat64 (1.0f) / DG_MESH_EFFECT_PRECISION_SCALE)


#define DG_MESH_EFFECT_INITIAL_VERTEX_SIZE	8
#define DG_VERTEXLIST_INDEXLIST_TOL			(dgFloat64 (0.0f))
#define DG_MESH_EFFECT_POINT_SPLITED		512



#define DG_MESH_EFFECT_BVH_STACK_DEPTH		256


class dgMeshEffect: public dgPolyhedra, public dgRefCounter
{
	public:

	class dgVertexAtribute 
	{
		public:
		dgBigVector m_vertex;
		dgFloat64 m_normal_x;
		dgFloat64 m_normal_y;
		dgFloat64 m_normal_z;
		dgFloat64 m_u0;
		dgFloat64 m_v0;
		dgFloat64 m_u1;
		dgFloat64 m_v1;
		dgFloat64 m_material;
	};

	class dgIndexArray 
	{
		public:
		dgInt32 m_materialCount;
		dgInt32 m_indexCount;
		dgInt32 m_materials[256];
		dgInt32 m_materialsIndexCount[256];
		dgInt32* m_indexList;
	};



	public:
	class dgMeshBVH
	{
		public:
		class dgMeshBVHNode
		{
			public:
			dgMeshBVHNode (const dgMeshEffect* const mesh, dgEdge* const face, void* const userData);
			dgMeshBVHNode (dgMeshBVHNode* const left, dgMeshBVHNode* const right);
			~dgMeshBVHNode ();
			void SetBox (const dgVector& p0, const dgVector& p1);

			DG_CLASS_ALLOCATOR(allocator)
			dgVector m_p0;
			dgVector m_p1;

			dgFloat32 m_area;
			dgEdge* m_face;
			void* m_userData;
			dgMeshBVHNode* m_left;
			dgMeshBVHNode* m_right;
			dgMeshBVHNode* m_parent;
		};

		class dgFitnessList: public dgTree <dgMeshBVHNode*, dgMeshBVHNode*>
		{
			public:
			dgFitnessList (dgMemoryAllocator* const allocator);
			dgFloat64 TotalCost () const;
		};

		
		dgMeshBVH (dgMeshEffect* const mesh);
		virtual ~dgMeshBVH();

		virtual void Build ();
		virtual void Cleanup ();

		void GetOverlapNodes (dgList<dgMeshBVHNode*>& overlapNodes, const dgBigVector& p0, const dgBigVector& p1) const;
		dgMeshBVHNode* FaceRayCast (const dgBigVector& l0, const dgBigVector& l1, dgFloat64& paramOut, bool doubleSidedFaces) const;

		protected:
		dgMeshBVHNode* AddFaceNode (dgEdge* const face, void* const userData);
		void RemoveNode (dgMeshBVHNode* const treeNode);
		void ImproveNodeFitness ();
		void ImproveNodeFitness (dgMeshBVHNode* const node);
		dgFloat32 CalculateSurfaceArea (dgMeshBVHNode* const node0, dgMeshBVHNode* const node1, dgVector& minBox, dgVector& maxBox) const;
		virtual bool SanityCheck() const;

		virtual dgFloat64 VertexRayCast (const dgBigVector& l0, const dgBigVector& l1) const;
		virtual dgFloat64 RayFaceIntersect (const dgMeshBVHNode* const face, const dgBigVector& p0, const dgBigVector& p1, bool dobleSidedFaces) const;
		virtual bool RayRayIntersect (dgEdge* const edge, const dgMeshEffect* const otherMesh, dgEdge* const otherEdge, dgFloat64& param, dgFloat64& otherParam) const;
		
		dgMeshEffect* m_mesh;
		dgMeshBVHNode* m_rootNode;
		dgFitnessList m_fitness;
		friend class dgMeshEffect;
	};



	dgMeshEffect(dgMemoryAllocator* const allocator);
	dgMeshEffect(dgCollisionInstance* const collision);
	dgMeshEffect(const dgMeshEffect& source);
	dgMeshEffect(dgPolyhedra& mesh, const dgMeshEffect& source);
	dgMeshEffect (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData);

	// create from OFF or PLY file format
	dgMeshEffect(dgMemoryAllocator* const allocator, const char* const fileName);

	// Create a convex hull Mesh form point cloud
	dgMeshEffect (dgMemoryAllocator* const allocator, const dgFloat64* const vertexCloud, dgInt32 count, dgInt32 strideInByte, dgFloat64 distTol);

	// create a planar Mesh
	dgMeshEffect(dgMemoryAllocator* const allocator, const dgMatrix& planeMatrix, dgFloat32 witdth, dgFloat32 breadth, dgInt32 material, const dgMatrix& textureMatrix0, const dgMatrix& textureMatrix1);
	virtual ~dgMeshEffect(void);

	void Trace () const;
	void SaveOFF (const char* const fileName) const;

	void ApplyTransform (const dgMatrix& matrix);
	dgMatrix CalculateOOBB (dgBigVector& size) const;
	void CalculateAABB (dgBigVector& min, dgBigVector& max) const;

	void CalculateNormals (dgFloat64 angleInRadians);
	void SphericalMapping (dgInt32 material);
	void BoxMapping (dgInt32 front, dgInt32 side, dgInt32 top);
	void UniformBoxMapping (dgInt32 material, const dgMatrix& textruMatrix);
	void CylindricalMapping (dgInt32 cylinderMaterial, dgInt32 capMaterial);
	void AngleBaseFlatteningMapping (dgInt32 cylinderMaterial, dgReportProgress progressReportCallback, void* const userData);

	dgEdge* InsertEdgeVertex (dgEdge* const edge, dgFloat64 param);

	dgMeshEffect* Union (const dgMatrix& matrix, const dgMeshEffect* const clipper) const;
	dgMeshEffect* Difference (const dgMatrix& matrix, const dgMeshEffect* const clipper) const;
	dgMeshEffect* Intersection (const dgMatrix& matrix, const dgMeshEffect* const clipper) const;
	void ClipMesh (const dgMatrix& matrix, const dgMeshEffect* const clipper, dgMeshEffect** const top, dgMeshEffect** const bottom) const;

	//bool PlaneClip (const dgBigPlane& plane);
	
	dgMeshEffect* ConvexMeshIntersection (const dgMeshEffect* const convexMesh) const;

	dgMeshEffect* GetFirstLayer ();
	dgMeshEffect* GetNextLayer (dgMeshEffect* const layer);

	void Triangulate ();
	void ConvertToPolygons ();
	void RemoveUnusedVertices(dgInt32* const vertexRemapTable);
	
	void BeginPolygon ();
	void AddPolygon (dgInt32 count, const dgFloat32* const vertexList, dgInt32 stride, dgInt32 material);
#ifndef _NEWTON_USE_DOUBLE
	void AddPolygon (dgInt32 count, const dgFloat64* const vertexList, dgInt32 stride, dgInt32 material);
#endif
	void EndPolygon (dgFloat64 tol, bool fixTjoint = true);

	void PackVertexArrays ();

	void BuildFromVertexListIndexList(dgInt32 faceCount, const dgInt32 * const faceIndexCount, const dgInt32 * const faceMaterialIndex, 
		const dgFloat32* const vertex, dgInt32  vertexStrideInBytes, const dgInt32 * const vertexIndex,
		const dgFloat32* const normal, dgInt32  normalStrideInBytes, const dgInt32 * const normalIndex,
		const dgFloat32* const uv0, dgInt32  uv0StrideInBytes, const dgInt32 * const uv0Index,
		const dgFloat32* const uv1, dgInt32  uv1StrideInBytes, const dgInt32 * const uv1Index);


	dgInt32 GetVertexCount() const;
	dgInt32 GetVertexStrideInByte() const;
	dgFloat64* GetVertexPool () const;

	dgInt32 GetPropertiesCount() const;
	dgInt32 GetPropertiesStrideInByte() const;
	dgFloat64* GetAttributePool() const;
	dgFloat64* GetNormalPool() const;
	dgFloat64* GetUV0Pool() const;
	dgFloat64* GetUV1Pool() const;

	dgEdge* SpliteFace (dgInt32 v0, dgInt32 v1);

	dgInt32 GetTotalFaceCount() const;
	dgInt32 GetTotalIndexCount() const;
	void GetFaces (dgInt32* const faceCount, dgInt32* const materials, void** const faceNodeList) const;

	void RepairTJoints ();
	bool SeparateDuplicateLoops (dgEdge* const face);

	bool HasOpenEdges () const;

	dgFloat64 CalculateVolume () const;

	void GetVertexStreams (dgInt32 vetexStrideInByte, dgFloat32* const vertex, 
						   dgInt32 normalStrideInByte, dgFloat32* const normal, 
						   dgInt32 uvStrideInByte0, dgFloat32* const uv0, 
						   dgInt32 uvStrideInByte1, dgFloat32* const uv1);

	void GetIndirectVertexStreams(dgInt32 vetexStrideInByte, dgFloat64* const vertex, dgInt32* const vertexIndices, dgInt32* const vertexCount,
								  dgInt32 normalStrideInByte, dgFloat64* const normal, dgInt32* const normalIndices, dgInt32* const normalCount,
								  dgInt32 uvStrideInByte0, dgFloat64* const uv0, dgInt32* const uvIndices0, dgInt32* const uvCount0,
								  dgInt32 uvStrideInByte1, dgFloat64* const uv1, dgInt32* const uvIndices1, dgInt32* const uvCount1);

	

	dgIndexArray* MaterialGeometryBegin();
	void MaterialGeomteryEnd(dgIndexArray* const handle);
	dgInt32 GetFirstMaterial (dgIndexArray* const handle) const;
	dgInt32 GetNextMaterial (dgIndexArray* const handle, dgInt32 materialHandle) const;
	dgInt32 GetMaterialID (dgIndexArray* const handle, dgInt32 materialHandle) const;
	dgInt32 GetMaterialIndexCount (dgIndexArray* const handle, dgInt32 materialHandle) const;
	void GetMaterialGetIndexStream (dgIndexArray* const handle, dgInt32 materialHandle, dgInt32* const index) const;
	void GetMaterialGetIndexStreamShort (dgIndexArray* const handle, dgInt32 materialHandle, dgInt16* const index) const;
	
	dgCollisionInstance* CreateCollisionTree(dgWorld* const world, dgInt32 shapeID) const;
	dgCollisionInstance* CreateConvexCollision(dgWorld* const world, dgFloat64 tolerance, dgInt32 shapeID, const dgMatrix& matrix = dgGetIdentityMatrix()) const;

	dgMeshEffect* CreateSimplification (dgInt32 maxVertexCount, dgReportProgress reportProgressCallback, void* const userData) const;
	dgMeshEffect* CreateConvexApproximation (dgFloat32 maxConcavity, dgFloat32 backFaceDistanceFactor, dgInt32 maxHullOuputCount, dgInt32 maxVertexPerHull, dgReportProgress reportProgressCallback, void* const userData) const;

	static dgMeshEffect* CreateDelaunayTetrahedralization (dgMemoryAllocator* const allocator, dgInt32 pointCount, dgInt32 pointStrideInBytes, const dgFloat32* const pointCloud, dgInt32 materialId, const dgMatrix& textureProjectionMatrix);
	static dgMeshEffect* CreateVoronoiConvexDecomposition (dgMemoryAllocator* const allocator, dgInt32 pointCount, dgInt32 pointStrideInBytes, const dgFloat32* const pointCloud, dgInt32 materialId, const dgMatrix& textureProjectionMatrix);
	static dgMeshEffect* CreateFromSerialization (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData);

	void Serialize (dgSerialize callback, void* const userData) const;

	dgBigVector& GetVertex (dgInt32 index) const;
	dgVertexAtribute& GetAttribute (dgInt32 index) const;
	void TransformMesh (const dgMatrix& matrix);


	void* GetFirstVertex () const;
	void* GetNextVertex (const void* const vertex) const;
	int GetVertexIndex (const void* const vertex) const;

	void* GetFirstPoint () const;
	void* GetNextPoint (const void* const point) const;
	int GetPointIndex (const void* const point) const;
	int GetVertexIndexFromPoint (const void* const point) const;


	void* GetFirstEdge () const;
	void* GetNextEdge (const void* const edge) const;
	void GetEdgeIndex (const void* const edge, dgInt32& v0, dgInt32& v1) const;
//	void GetEdgeAttributeIndex (const void* edge, dgInt32& v0, dgInt32& v1) const;

	void* GetFirstFace () const;
	void* GetNextFace (const void* const face) const;
	int IsFaceOpen (const void* const face) const;
	int GetFaceMaterial (const void* const face) const;
	int GetFaceIndexCount (const void* const face) const;
	void GetFaceIndex (const void* const face, dgInt32* const indices) const;
	void GetFaceAttributeIndex (const void* const face, dgInt32* const indices) const;
	dgBigVector CalculateFaceNormal (const void* const face) const;

	void SetFaceMaterial (const void* const face, int materialID) const;

	
	dgVertexAtribute InterpolateVertex (const dgBigVector& point, const dgEdge* const face) const;

	bool Sanity () const;
	void AddVertex(const dgBigVector& vertex);
	void AddAtribute (const dgVertexAtribute& attib);
	void AddPoint(const dgFloat64* vertexList, dgInt32 material);

	protected:
	virtual void BeginFace();
	virtual void EndFace ();

	void Init ();
	dgBigVector GetOrigin ()const;
	dgInt32 CalculateMaxAttributes () const;
	dgFloat64 QuantizeCordinade(dgFloat64 val) const;

	void ClearAttributeArray ();
	dgInt32 EnumerateAttributeArray (dgVertexAtribute* const attib);
	void ApplyAttributeArray (dgVertexAtribute* const attib, dgInt32 maxCount);
	
	void MergeFaces (const dgMeshEffect* const source);
//	void ReverseMergeFaces (dgMeshEffect* const source);
	dgVertexAtribute InterpolateEdge (dgEdge* const edge, dgFloat64 param) const;

	bool PlaneClip (const dgMeshEffect& convexMesh, const dgEdge* const face);

	dgMeshEffect* GetNextLayer (dgInt32 mark);
	dgMeshEffect* CreateVoronoiConvex (const dgBigVector* const conevexPointCloud, dgInt32 count, dgInt32 materialId, const dgMatrix& textureProjectionMatrix, dgFloat32 normalAngleInRadians) const;


	dgInt32 m_pointCount;
	dgInt32 m_maxPointCount;

	dgInt32 m_atribCount;
	dgInt32 m_maxAtribCount;

	dgBigVector* m_points;
	dgVertexAtribute* m_attrib;
	
	friend class dgConvexHull3d;
	friend class dgConvexHull4d;
	friend class dgBooleanMeshBVH;
	friend class dgTriangleAnglesToUV;
	friend class dgCollisionCompoundFractured;
};



DG_INLINE dgInt32 dgMeshEffect::GetVertexCount() const
{
	return m_pointCount;
}

DG_INLINE dgInt32 dgMeshEffect::GetPropertiesCount() const
{
	return m_atribCount;
}

DG_INLINE dgInt32 dgMeshEffect::GetMaterialID (dgIndexArray* const handle, dgInt32 materialHandle) const
{
	return handle->m_materials[materialHandle];
}

DG_INLINE dgInt32 dgMeshEffect::GetMaterialIndexCount (dgIndexArray* const handle, dgInt32 materialHandle) const
{
	return handle->m_materialsIndexCount[materialHandle];
}

DG_INLINE dgMeshEffect::dgVertexAtribute& dgMeshEffect::GetAttribute (dgInt32 index) const 
{
	return m_attrib[index];
}

DG_INLINE dgBigVector& dgMeshEffect::GetVertex (dgInt32 index) const
{
	return m_points[index];
}

DG_INLINE dgInt32 dgMeshEffect::GetPropertiesStrideInByte() const 
{
	return sizeof (dgVertexAtribute);
}

DG_INLINE dgFloat64* dgMeshEffect::GetAttributePool() const 
{
	return &m_attrib->m_vertex.m_x;
}

DG_INLINE dgFloat64* dgMeshEffect::GetNormalPool() const 
{
	return &m_attrib->m_normal_x;
}

DG_INLINE dgFloat64* dgMeshEffect::GetUV0Pool() const 
{
	return &m_attrib->m_u0;
}

DG_INLINE dgFloat64* dgMeshEffect::GetUV1Pool() const 
{
	return &m_attrib->m_u1;
}

DG_INLINE dgInt32 dgMeshEffect::GetVertexStrideInByte() const 
{
	return sizeof (dgBigVector);
}

DG_INLINE dgFloat64* dgMeshEffect::GetVertexPool () const 
{
	return &m_points[0].m_x;
}


DG_INLINE dgMeshEffect* dgMeshEffect::GetFirstLayer ()
{
	return GetNextLayer (IncLRU());
}

DG_INLINE dgMeshEffect* dgMeshEffect::GetNextLayer (dgMeshEffect* const layerSegment)
{
	if (!layerSegment) {
		return NULL;
	}
	return GetNextLayer (layerSegment->IncLRU() - 1);
}


DG_INLINE dgFloat64 dgMeshEffect::QuantizeCordinade(dgFloat64 x) const
{
	dgInt32 exp;
	dgFloat64 mantissa = frexp(x, &exp);
	mantissa = DG_MESH_EFFECT_PRECISION_SCALE_INV * floor (mantissa * DG_MESH_EFFECT_PRECISION_SCALE);

	dgFloat64 x1 = ldexp(mantissa, exp);
	return x1;
}

#endif
