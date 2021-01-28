/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_MESH_EFFECT_H__
#define __D_MESH_EFFECT_H__

#include "ndCollisionStdafx.h"

#define DG_MESH_EFFECT_PRECISION_BITS		48
#define DG_MESH_EFFECT_PRECISION_SCALE		dFloat64(dInt64(1)<<DG_MESH_EFFECT_PRECISION_BITS)
#define DG_MESH_EFFECT_PRECISION_SCALE_INV	(dFloat64 (1.0f) / DG_MESH_EFFECT_PRECISION_SCALE)

#define DG_VERTEXLIST_INDEXLIST_TOL			(dFloat64 (0.0f))
#define DG_MESH_EFFECT_POINT_SPLITED		512
#define DG_MESH_EFFECT_BVH_STACK_DEPTH		256

class ndIndexArray
{
	public:
	dInt32 m_materialCount;
	dInt32 m_indexCount;
	dInt32 m_materials[256];
	dInt32 m_materialsIndexCount[256];
	dInt32* m_indexList;
};

class ndMeshEffect: public dPolyhedra
{
#if 0
	public:
	class dMeshBVH
	{
		public:
		class dgMeshBVHNode
		{
			public:
			dgMeshBVHNode (const ndMeshEffect* const mesh, dEdge* const face, void* const userData);
			dgMeshBVHNode (dgMeshBVHNode* const left, dgMeshBVHNode* const right);
			~dgMeshBVHNode ();
			void SetBox (const dVector& p0, const dVector& p1);

			DG_CLASS_ALLOCATOR(allocator)
			dVector m_p0;
			dVector m_p1;

			dFloat32 m_area;
			dEdge* m_face;
			void* m_userData;
			dgMeshBVHNode* m_left;
			dgMeshBVHNode* m_right;
			dgMeshBVHNode* m_parent;
		};

		class dgFitnessList: public dTree <dgMeshBVHNode*, dgMeshBVHNode*>
		{
			public:
			dgFitnessList (dMemoryAllocator___* const allocator);
			dFloat64 TotalCost () const;
		};

		
		dMeshBVH (const ndMeshEffect* const mesh);
		virtual ~dMeshBVH();

		virtual void Build ();
		virtual void Cleanup ();
		
		void FaceRayCast (const dBigVector& l0, const dBigVector& l1, void* const userData) const;
		void GetOverlapNodes (dList<dgMeshBVHNode*>& overlapNodes, const dBigVector& p0, const dBigVector& p1) const;

		protected:
		virtual dgMeshBVHNode* CreateLeafNode (dEdge* const face, void* const userData) = 0;

		dgMeshBVHNode* AddFaceNode (dEdge* const face, void* const userData);
		void RemoveNode (dgMeshBVHNode* const treeNode);
		void ImproveNodeFitness ();
		void ImproveNodeFitness (dgMeshBVHNode* const node);
		dFloat32 CalculateSurfaceArea (dgMeshBVHNode* const node0, dgMeshBVHNode* const node1, dVector& minBox, dVector& maxBox) const;
		virtual bool SanityCheck() const;

		virtual dFloat64 RayFaceIntersect (const dgMeshBVHNode* const face, const dBigVector& p0, const dBigVector& p1, void* const userData) const;
//		virtual dFloat64 VertexRayCast (const dBigVector& l0, const dBigVector& l1) const;
//		virtual bool RayRayIntersect (dEdge* const edge, const ndMeshEffect* const otherMesh, dEdge* const otherEdge, dFloat64& param, dFloat64& otherParam) const;
		
		const ndMeshEffect* m_mesh;
		dgMeshBVHNode* m_rootNode;
		dgFitnessList m_fitness;
		friend class ndMeshEffect;
	};

	
	ndMeshEffect(dMemoryAllocator___* const allocator);
	ndMeshEffect(dgCollisionInstance* const collision);
	ndMeshEffect(const ndMeshEffect& source);
	ndMeshEffect(dPolyhedra& mesh, const ndMeshEffect& source);
	ndMeshEffect (dMemoryAllocator___* const allocator, dgDeserialize deserialization, void* const userData);

	// create from OFF or PLY file format
	ndMeshEffect(dMemoryAllocator___* const allocator, const char* const fileName);

	// Create a convex hull Mesh form point cloud
	ndMeshEffect (dMemoryAllocator___* const allocator, const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol);

	// create a planar Mesh
	ndMeshEffect(dMemoryAllocator___* const allocator, const dMatrix& planeMatrix, dFloat32 witdth, dFloat32 breadth, dInt32 material, const dMatrix& textureMatrix0, const dMatrix& textureMatrix1);

	void Trace () const;

	
	dMatrix CalculateOOBB (dBigVector& size) const;
	void CalculateAABB (dBigVector& min, dBigVector& max) const;

	void FlipWinding(); 
	void CylindricalMapping (dInt32 cylinderMaterial, dInt32 capMaterial, const dMatrix& uvAligment);
	void AngleBaseFlatteningMapping (dInt32 cylinderMaterial, dgReportProgress progressReportCallback, void* const userData);

	dEdge* InsertEdgeVertex (dEdge* const edge, dFloat64 param);

	ndMeshEffect* Union (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	ndMeshEffect* Difference (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	ndMeshEffect* Intersection (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	void ClipMesh (const dMatrix& matrix, const ndMeshEffect* const clipper, ndMeshEffect** const top, ndMeshEffect** const bottom) const;

	//bool PlaneClip (const dBigPlane& plane);
	
	ndMeshEffect* ConvexMeshIntersection (const ndMeshEffect* const convexMesh) const;

	ndMeshEffect* GetFirstLayer ();
	ndMeshEffect* GetNextLayer (ndMeshEffect* const layer);

	void Triangulate ();
	void ConvertToPolygons ();
	void RemoveUnusedVertices(dInt32* const vertexRemapTable);

	dInt32 GetVertexBaseCount() const;
	void SetVertexBaseCount(dInt32 count);
	
	dEdge* SpliteFace (dInt32 v0, dInt32 v1);

	dInt32 GetTotalFaceCount() const;
	dInt32 GetTotalIndexCount() const;
	void GetFaces (dInt32* const faceCount, dInt32* const materials, void** const faceNodeList) const;

	bool HasOpenEdges () const;

	dFloat64 CalculateVolume () const;

	void OptimizePoints();
	void OptimizeAttibutes();
	const dInt32* GetIndexToVertexMap() const;

	bool HasLayersChannel() const;
	bool HasNormalChannel() const;
	bool HasBinormalChannel() const;
	bool HasUV0Channel() const;
	bool HasUV1Channel() const;
	bool HasVertexColorChannel() const;
	
	dgCollisionInstance* CreateCollisionTree(dgWorld* const world, dInt32 shapeID) const;
	dgCollisionInstance* CreateConvexCollision(dgWorld* const world, dFloat64 tolerance, dInt32 shapeID, const dMatrix& matrix = dGetIdentityMatrix()) const;

	ndMeshEffect* CreateSimplification (dInt32 maxVertexCount, dgReportProgress reportProgressCallback, void* const userData) const;
	ndMeshEffect* CreateConvexApproximation (dFloat32 maxConcavity, dFloat32 backFaceDistanceFactor, dInt32 maxHullOuputCount, dInt32 maxVertexPerHull, dgReportProgress reportProgressCallback, void* const userData) const;

	ndMeshEffect* CreateTetrahedraIsoSurface() const;
	void CreateTetrahedraLinearBlendSkinWeightsChannel (const ndMeshEffect* const tetrahedraMesh);

	
	static ndMeshEffect* CreateFromSerialization (dMemoryAllocator___* const allocator, dgDeserialize deserialization, void* const userData);

	void LoadOffMesh (const char* const filename);
	void LoadTetraMesh (const char* const filename);
	void Serialize (dgSerialize callback, void* const userData) const;

	dBigVector GetVertex (dInt32 index) const;
	dInt32 GetVertexLayer (dInt32 index) const;

	void TransformMesh (const dMatrix& matrix);

	void* GetFirstVertex () const;
	void* GetNextVertex (const void* const vertex) const;
	dInt32 GetVertexIndex (const void* const vertex) const;

	void* GetFirstPoint () const;
	void* GetNextPoint (const void* const point) const;
	dInt32 GetPointIndex (const void* const point) const;
	dInt32 GetVertexIndexFromPoint (const void* const point) const;

	void* GetFirstEdge () const;
	void* GetNextEdge (const void* const edge) const;
//	void* FindEdge (dInt32 v0, dInt32 v1) const;
	void GetEdgeIndex (const void* const edge, dInt32& v0, dInt32& v1) const;
//	void GetEdgeAttributeIndex (const void* edge, dInt32& v0, dInt32& v1) const;

	const dEdge* GetPolyhedraEdgeFromNode(const void* const edge) const;

	void* GetFirstFace () const;
	void* GetNextFace (const void* const face) const;
	dInt32 IsFaceOpen (const void* const face) const;
	dInt32 GetFaceIndexCount (const void* const face) const;
	void GetFaceIndex (const void* const face, dInt32* const indices) const;
	void GetFaceAttributeIndex (const void* const face, dInt32* const indices) const;
	dBigVector CalculateFaceNormal (const void* const face) const;

	void SetFaceMaterial (const void* const face, dInt32 materialID);
	void AddInterpolatedEdgeAttribute (dEdge* const edge, dFloat64 param);
	dInt32 InterpolateVertex (const dBigVector& point, const dEdge* const face) const;

	protected:

	dBigVector GetOrigin ()const;
	dInt32 CalculateMaxAttributes () const;
	dFloat64 QuantizeCordinade(dFloat64 val) const;
//	void ReverseMergeFaces (ndMeshEffect* const source);

	bool PlaneClip (const ndMeshEffect& convexMesh, const dEdge* const face);

	ndMeshEffect* GetNextLayer (dInt32 mark);
	ndMeshEffect* CreateVoronoiConvex (const dBigVector* const conevexPointCloud, dInt32 count, dInt32 materialId, const dMatrix& textureProjectionMatrix, dFloat32 normalAngleInRadians) const;
	
	void PackPoints (dFloat64 tol);
	void UnpackPoints();
	
	friend class dConvexHull3d;
	friend class dgConvexHull4d;
	friend class dgBooleanMeshBVH;
	friend class dgHACDClusterGraph;
	friend class dgTriangleAnglesToUV;
	friend class dgTetraIsoSufaceStuffing;
	friend class dgCollisionCompoundFractured;
#endif

	enum dChannelType
	{
		m_vertex,
		m_normal,
		m_binormal,
		m_uv0,
		m_uv1,
		m_color,
		m_material,
		m_layer,
		m_point,
	};

	template<class T, dChannelType type>
	class dChannel: public dArray<T>
	{
		public:
		dChannel();
		dChannel(const dChannel& source);
		~dChannel();

		void Clear();
		void PushBack(const T& element);
		
		dChannelType m_type;
		bool m_isValid;
	};

	class dFormat
	{
		public:
		class dSortKey
		{
			public:
			dInt32 m_mask;
			dInt32 m_ordinal;
			dInt32 m_vertexIndex;
			dInt32 m_attibuteIndex;
		};
		class dVertexSortData
		{
			public:
			const dChannel<dBigVector, m_point>* m_points;
			dInt32 m_vertexSortIndex;
		};

		dInt32 GetSortIndex(const dChannel<dBigVector, m_point>& points, dFloat64& dist) const;
		static dInt32 CompareVertex(const dSortKey* const ptr0, const dSortKey* const ptr1, void* const context);
	};

	class dPointFormat: public dFormat
	{
		public:
		dPointFormat();
		dPointFormat(const dPointFormat& source);
		~dPointFormat();

		void Clear();
		void SetCount(dInt32 count);
		void CompressData(dInt32* const indexList);
		
		dChannel<dInt32, m_layer> m_layers;
		dChannel<dBigVector, m_point> m_vertex;
	};

	class dAttibutFormat: public dFormat
	{
		public:
		class dgUV
		{
			public:
			dFloat32 m_u;
			dFloat32 m_v;
		};

		dAttibutFormat();
		dAttibutFormat(const dAttibutFormat& source);
		~dAttibutFormat();

		void Clear();
		void SetCount(dInt32 count);
		void CopyFrom(const dAttibutFormat& source);
		void CopyEntryFrom(dInt32 index, const dAttibutFormat& source, dInt32 sourceIndex);
		void CompressData(const dPointFormat& points, dInt32* const indexList);

		dChannel<dInt32, m_vertex> m_pointChannel;
		dChannel<dInt32, m_material> m_materialChannel;
		dChannel<dTriplex, m_normal> m_normalChannel;
		dChannel<dTriplex, m_binormal> m_binormalChannel;
		dChannel<dVector, m_color> m_colorChannel;
		dChannel<dgUV, m_uv0> m_uv0Channel;
		dChannel<dgUV, m_uv1> m_uv1Channel;
	};

	public:
	D_MSV_NEWTON_ALIGN_16
	class dMaterial
	{
		public:
		dMaterial()
			:m_ambient(dFloat32(0.8f), dFloat32(0.8f), dFloat32(0.8f), dFloat32(1.0f))
			,m_diffuse(dFloat32(0.8f), dFloat32(0.8f), dFloat32(0.8f), dFloat32(1.0f))
			,m_specular(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f))
			,m_opacity(dFloat32(1.0f))
			,m_shiness(dFloat32 (60.0f))
		{
			m_textureName[0] = 0;
		}

		dVector m_ambient;
		dVector m_diffuse;
		dVector m_specular;
		dFloat32 m_opacity;
		dFloat32 m_shiness;
		char m_textureName[32];
	}D_GCC_NEWTON_ALIGN_16;

	class dMeshVertexFormat
	{
		public:
		class dDoubleData
		{
			public:
			const dFloat64* m_data;
			const dInt32* m_indexList;
			dInt32 m_strideInBytes;
		};

		class dFloatData
		{
			public:
			const dFloat32* m_data;
			const dInt32* m_indexList;
			dInt32 m_strideInBytes;
		};

		dMeshVertexFormat()
		{
			Clear();
		}

		void Clear()
		{
			memset(this, 0, sizeof(dMeshVertexFormat));
		}

		dInt32 m_faceCount;
		const dInt32* m_faceIndexCount;
		const dInt32* m_faceMaterial;
		dDoubleData m_vertex;
		dFloatData m_normal;
		dFloatData m_binormal;
		dFloatData m_uv0;
		dFloatData m_uv1;
		dFloatData m_vertexColor;
	};
	
	D_COLLISION_API ndMeshEffect();
	//D_COLLISION_API ndMeshEffect(dgCollisionInstance* const collision);
	
	// Create a convex hull Mesh form point cloud
	D_COLLISION_API ndMeshEffect(const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol);

	D_COLLISION_API virtual ~ndMeshEffect();

	void SetName (const dString& name);
	const dString& GetName() const;

	dArray<dMaterial>& GetMaterials();
	dInt32 GetPropertiesCount() const;

	dInt32 GetVertexCount() const;
	dInt32 GetVertexStrideInByte() const;
	const dFloat64* GetVertexPool() const;

	dInt32 GetFaceMaterial(dEdge* const faceEdge) const;

	D_COLLISION_API void ApplyTransform(const dMatrix& matrix);
	D_COLLISION_API void CalculateNormals(dFloat64 angleInRadians);
	D_COLLISION_API void BuildFromIndexList(const dMeshVertexFormat* const format);
	
	D_COLLISION_API void GetVertexChannel64(dInt32 strideInByte, dFloat64* const bufferOut) const;
	D_COLLISION_API void GetVertexChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_COLLISION_API void GetNormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_COLLISION_API void GetBinormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_COLLISION_API void GetUV0Channel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_COLLISION_API void GetUV1Channel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_COLLISION_API void GetVertexColorChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	//	void GetWeightBlendChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	//	void GetWeightIndexChannel(dInt32 strideInByte, dInt32* const bufferOut) const;

	D_COLLISION_API ndIndexArray* MaterialGeometryBegin();
		D_COLLISION_API dInt32 GetFirstMaterial(ndIndexArray* const handle) const;
		D_COLLISION_API dInt32 GetNextMaterial(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_COLLISION_API dInt32 GetMaterialID(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_COLLISION_API dInt32 GetMaterialIndexCount(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_COLLISION_API void GetMaterialGetIndexStream(ndIndexArray* const handle, dInt32 materialHandle, dInt32* const index) const;
		D_COLLISION_API void GetMaterialGetIndexStreamShort(ndIndexArray* const handle, dInt32 materialHandle, dInt16* const index) const;
	D_COLLISION_API void MaterialGeomteryEnd(ndIndexArray* const handle);

	D_COLLISION_API void BeginBuild();
	//	D_COLLISION_API void BeginBuildFace();
	//		D_COLLISION_API void AddPoint(dFloat64 x, dFloat64 y, dFloat64 z);
	//		D_COLLISION_API void AddLayer(dInt32 layer);
	//		D_COLLISION_API void AddMaterial(dInt32 materialIndex);
	//		D_COLLISION_API void AddNormal(dFloat32 x, dFloat32 y, dFloat32 z);
	//		D_COLLISION_API void AddBinormal(dFloat32 x, dFloat32 y, dFloat32 z);
	//		D_COLLISION_API void AddVertexColor(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w);
	//		D_COLLISION_API void AddUV0(dFloat32 u, dFloat32 v);
	//		D_COLLISION_API void AddUV1(dFloat32 u, dFloat32 v);
	//	D_COLLISION_API void EndBuildFace();
	D_COLLISION_API void EndBuild(dFloat64 tol, bool fixTjoint = true);

	D_COLLISION_API dBigVector GetOrigin()const;
	D_COLLISION_API void SphericalMapping(dInt32 material, const dMatrix& uvAligment);
	D_COLLISION_API void UniformBoxMapping(dInt32 material, const dMatrix& textureMatrix);
	D_COLLISION_API void BoxMapping(dInt32 front, dInt32 side, dInt32 top, const dMatrix& uvAligment);
	D_COLLISION_API void RepairTJoints();

	D_COLLISION_API static ndMeshEffect* CreateVoronoiConvexDecomposition(dInt32 pointCount, dInt32 pointStrideInBytes, const dFloat32* const pointCloud, dInt32 materialId, const dMatrix& textureProjectionMatrix);

	protected:
	D_COLLISION_API void Init();
	D_COLLISION_API virtual void BeginFace();
	D_COLLISION_API virtual bool EndFace();

	bool Sanity() const;
	void PackAttibuteData();
	void UnpackAttibuteData();
	bool SeparateDuplicateLoops(dEdge* const face);
	dInt32 AddInterpolatedHalfAttribute(dEdge* const edge, dInt32 midPoint);

	void MergeFaces(const ndMeshEffect* const source);

	dString m_name;
	dPointFormat m_points;
	dAttibutFormat m_attrib;
	dArray<dMaterial> m_materials;
	dInt32 m_vertexBaseCount;
	dInt32 m_constructionIndex;
};

#if 0

inline dInt32 ndMeshEffect::GetVertexBaseCount() const
{
	return m_vertexBaseCount;
}

inline void ndMeshEffect::SetVertexBaseCount(dInt32 count)
{
	m_vertexBaseCount = count;
}


inline const dInt32* ndMeshEffect::GetIndexToVertexMap() const
{
	return &m_attrib.m_pointChannel[0];
}

inline dBigVector ndMeshEffect::GetVertex (dInt32 index) const
{
	dAssert(index >= 0);
	dAssert(index < m_points.m_vertex.m_count);
	return m_points.m_vertex[index];
}

inline bool ndMeshEffect::HasLayersChannel() const
{
	return m_points.m_layers.m_count != 0;
}

inline dInt32 ndMeshEffect::GetVertexLayer(dInt32 index) const
{
	dAssert(index >= 0);
	dAssert(index < m_points.m_vertex.m_count);
	return (m_points.m_layers.m_count) ? m_points.m_layers[index] : 0;
}

inline ndMeshEffect* ndMeshEffect::GetFirstLayer ()
{
	return GetNextLayer (IncLRU());
}

inline ndMeshEffect* ndMeshEffect::GetNextLayer (ndMeshEffect* const layerSegment)
{
	if (!layerSegment) {
		return nullptr;
	}
	return GetNextLayer (layerSegment->IncLRU() - 1);
}


inline dFloat64 ndMeshEffect::QuantizeCordinade(dFloat64 x) const
{
	dInt32 exp;
	dFloat64 mantissa = frexp(x, &exp);
	mantissa = DG_MESH_EFFECT_PRECISION_SCALE_INV * floor (mantissa * DG_MESH_EFFECT_PRECISION_SCALE);

	dFloat64 x1 = ldexp(mantissa, exp);
	return x1;
}
#endif

template<class T, ndMeshEffect::dChannelType type>
ndMeshEffect::dChannel<T, type>::dChannel()
	:dArray<T>()
	,m_type(type)
	,m_isValid(false)
{
}

template<class T, ndMeshEffect::dChannelType type>
ndMeshEffect::dChannel<T, type>::dChannel(const dChannel& source)
	:dArray<T>(source)
	,m_type(source.m_type)
	,m_isValid(source.m_isValid)
{
}

template<class T, ndMeshEffect::dChannelType type>
ndMeshEffect::dChannel<T, type>::~dChannel()
{
}

template<class T, ndMeshEffect::dChannelType type>
void ndMeshEffect::dChannel<T, type>::Clear()
{
	m_isValid = false;
	dArray<T>::Clear();
}

template<class T, ndMeshEffect::dChannelType type>
void ndMeshEffect::dChannel<T, type>::PushBack(const T& element)
{
	T tmp(element);
	m_isValid = true;
	dArray<T>::PushBack(tmp);
}

inline ndMeshEffect::dPointFormat::dPointFormat()
	:m_layers()
	,m_vertex()
{
}

inline ndMeshEffect::dPointFormat::dPointFormat(const dPointFormat& source)
	:m_layers(source.m_layers)
	, m_vertex(source.m_vertex)
{
}

inline ndMeshEffect::dPointFormat::~dPointFormat()
{
}

inline void ndMeshEffect::dPointFormat::Clear()
{
	m_layers.Clear();
	m_vertex.Clear();
}

inline void ndMeshEffect::dPointFormat::SetCount(dInt32 count)
{
	m_layers.Resize(count);
	m_vertex.Resize(count);
	m_layers.SetCount(count);
	m_vertex.SetCount(count);
}

inline ndMeshEffect::dAttibutFormat::dAttibutFormat()
	:m_pointChannel()
	,m_materialChannel()
	,m_normalChannel()
	,m_binormalChannel()
	,m_colorChannel()
	,m_uv0Channel()
	,m_uv1Channel()
{
}

inline ndMeshEffect::dAttibutFormat::dAttibutFormat(const dAttibutFormat& source)
	:m_pointChannel(source.m_pointChannel)
	,m_materialChannel(source.m_materialChannel)
	,m_normalChannel(source.m_normalChannel)
	,m_binormalChannel(source.m_binormalChannel)
	,m_colorChannel(source.m_colorChannel)
	,m_uv0Channel(source.m_uv0Channel)
	,m_uv1Channel(source.m_uv1Channel)
{
}

inline ndMeshEffect::dAttibutFormat::~dAttibutFormat()
{
}

inline void ndMeshEffect::dAttibutFormat::Clear()
{
	m_pointChannel.Clear();
	m_materialChannel.Clear();
	m_normalChannel.Clear();
	m_binormalChannel.Clear();
	m_colorChannel.Clear();
	m_uv0Channel.Clear();
	m_uv1Channel.Clear();
}

inline void ndMeshEffect::dAttibutFormat::SetCount(dInt32 count)
{
	m_pointChannel.Resize(count);
	m_materialChannel.Resize(count);
	m_normalChannel.Resize(count);
	m_binormalChannel.Resize(count);
	m_colorChannel.Resize(count);
	m_uv0Channel.Resize(count);
	m_uv1Channel.Resize(count);

	m_pointChannel.SetCount(count);
	m_materialChannel.SetCount(count);
	m_normalChannel.SetCount(count);
	m_binormalChannel.SetCount(count);
	m_colorChannel.SetCount(count);
	m_uv0Channel.SetCount(count);
	m_uv1Channel.SetCount(count);
}

inline dInt32 ndMeshEffect::GetPropertiesCount() const
{
	return m_attrib.m_pointChannel.GetCount();
}

inline void ndMeshEffect::SetName(const dString& name)
{
	m_name = name;
}

inline const dString& ndMeshEffect::GetName() const
{
	return m_name;
}

inline dArray<ndMeshEffect::dMaterial>& ndMeshEffect::GetMaterials()
{
	return m_materials;
}

inline dInt32 ndMeshEffect::GetVertexCount() const
{
	return m_points.m_vertex.GetCount();
}

inline dInt32 ndMeshEffect::GetVertexStrideInByte() const
{
	return sizeof(dBigVector);
}

inline const dFloat64* ndMeshEffect::GetVertexPool() const
{
	return &m_points.m_vertex[0].m_x;
}

inline dInt32 ndMeshEffect::GetFaceMaterial(dEdge* const faceEdge) const
{
	//dTreeNode* const node = (dTreeNode*)face;
	//dEdge* const edge = &node->GetInfo();
	return dInt32(m_attrib.m_materialChannel.GetCount() ? m_attrib.m_materialChannel[dInt32(faceEdge->m_userData)] : 0);
}


#endif
