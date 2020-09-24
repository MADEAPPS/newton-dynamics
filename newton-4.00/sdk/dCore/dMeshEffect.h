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

#include "dCoreStdafx.h"
#include "dArray.h"
#include "dVector.h"
#include "dPolyhedra.h"


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

class dMeshEffect: public dPolyhedra
{
#if 0
	public:
	class dMeshBVH
	{
		public:
		class dgMeshBVHNode
		{
			public:
			dgMeshBVHNode (const dMeshEffect* const mesh, dEdge* const face, void* const userData);
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

		
		dMeshBVH (const dMeshEffect* const mesh);
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
//		virtual bool RayRayIntersect (dEdge* const edge, const dMeshEffect* const otherMesh, dEdge* const otherEdge, dFloat64& param, dFloat64& otherParam) const;
		
		const dMeshEffect* m_mesh;
		dgMeshBVHNode* m_rootNode;
		dgFitnessList m_fitness;
		friend class dMeshEffect;
	};

	
	dMeshEffect(dMemoryAllocator___* const allocator);
	dMeshEffect(dgCollisionInstance* const collision);
	dMeshEffect(const dMeshEffect& source);
	dMeshEffect(dPolyhedra& mesh, const dMeshEffect& source);
	dMeshEffect (dMemoryAllocator___* const allocator, dgDeserialize deserialization, void* const userData);

	// create from OFF or PLY file format
	dMeshEffect(dMemoryAllocator___* const allocator, const char* const fileName);

	// Create a convex hull Mesh form point cloud
	dMeshEffect (dMemoryAllocator___* const allocator, const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol);

	// create a planar Mesh
	dMeshEffect(dMemoryAllocator___* const allocator, const dMatrix& planeMatrix, dFloat32 witdth, dFloat32 breadth, dInt32 material, const dMatrix& textureMatrix0, const dMatrix& textureMatrix1);

	void Trace () const;

	void ApplyTransform (const dMatrix& matrix);
	dMatrix CalculateOOBB (dBigVector& size) const;
	void CalculateAABB (dBigVector& min, dBigVector& max) const;

	void FlipWinding(); 
	void UniformBoxMapping (dInt32 material, const dMatrix& textureMatrix);
	void CylindricalMapping (dInt32 cylinderMaterial, dInt32 capMaterial, const dMatrix& uvAligment);
	void AngleBaseFlatteningMapping (dInt32 cylinderMaterial, dgReportProgress progressReportCallback, void* const userData);

	dEdge* InsertEdgeVertex (dEdge* const edge, dFloat64 param);

	dMeshEffect* Union (const dMatrix& matrix, const dMeshEffect* const clipper) const;
	dMeshEffect* Difference (const dMatrix& matrix, const dMeshEffect* const clipper) const;
	dMeshEffect* Intersection (const dMatrix& matrix, const dMeshEffect* const clipper) const;
	void ClipMesh (const dMatrix& matrix, const dMeshEffect* const clipper, dMeshEffect** const top, dMeshEffect** const bottom) const;

	//bool PlaneClip (const dBigPlane& plane);
	
	dMeshEffect* ConvexMeshIntersection (const dMeshEffect* const convexMesh) const;

	dMeshEffect* GetFirstLayer ();
	dMeshEffect* GetNextLayer (dMeshEffect* const layer);

	void Triangulate ();
	void ConvertToPolygons ();
	void RemoveUnusedVertices(dInt32* const vertexRemapTable);
	

	dInt32 GetVertexCount() const;
	dInt32 GetVertexStrideInByte() const;
	const dFloat64* GetVertexPool () const;

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
	dgCollisionInstance* CreateConvexCollision(dgWorld* const world, dFloat64 tolerance, dInt32 shapeID, const dMatrix& matrix = dgGetIdentityMatrix()) const;

	dMeshEffect* CreateSimplification (dInt32 maxVertexCount, dgReportProgress reportProgressCallback, void* const userData) const;
	dMeshEffect* CreateConvexApproximation (dFloat32 maxConcavity, dFloat32 backFaceDistanceFactor, dInt32 maxHullOuputCount, dInt32 maxVertexPerHull, dgReportProgress reportProgressCallback, void* const userData) const;

	dMeshEffect* CreateTetrahedraIsoSurface() const;
	void CreateTetrahedraLinearBlendSkinWeightsChannel (const dMeshEffect* const tetrahedraMesh);

	static dMeshEffect* CreateVoronoiConvexDecomposition (dMemoryAllocator___* const allocator, dInt32 pointCount, dInt32 pointStrideInBytes, const dFloat32* const pointCloud, dInt32 materialId, const dMatrix& textureProjectionMatrix);
	static dMeshEffect* CreateFromSerialization (dMemoryAllocator___* const allocator, dgDeserialize deserialization, void* const userData);

	void LoadOffMesh (const char* const filename);
	void LoadTetraMesh (const char* const filename);
	void Serialize (dgSerialize callback, void* const userData) const;

	dBigVector GetVertex (dInt32 index) const;
	dInt32 GetVertexLayer (dInt32 index) const;

	void TransformMesh (const dMatrix& matrix);

	void* GetFirstVertex () const;
	void* GetNextVertex (const void* const vertex) const;
	int GetVertexIndex (const void* const vertex) const;

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
	dInt32 GetFaceMaterial (const void* const face) const;
	dInt32 GetFaceIndexCount (const void* const face) const;
	void GetFaceIndex (const void* const face, dInt32* const indices) const;
	void GetFaceAttributeIndex (const void* const face, dInt32* const indices) const;
	dBigVector CalculateFaceNormal (const void* const face) const;

	void SetFaceMaterial (const void* const face, int materialID);
	void AddInterpolatedEdgeAttribute (dEdge* const edge, dFloat64 param);
	dInt32 InterpolateVertex (const dBigVector& point, const dEdge* const face) const;

	protected:

	dBigVector GetOrigin ()const;
	dInt32 CalculateMaxAttributes () const;
	dFloat64 QuantizeCordinade(dFloat64 val) const;

	void MergeFaces (const dMeshEffect* const source);
//	void ReverseMergeFaces (dMeshEffect* const source);

	bool PlaneClip (const dMeshEffect& convexMesh, const dEdge* const face);

	dMeshEffect* GetNextLayer (dInt32 mark);
	dMeshEffect* CreateVoronoiConvex (const dBigVector* const conevexPointCloud, dInt32 count, dInt32 materialId, const dMatrix& textureProjectionMatrix, dFloat32 normalAngleInRadians) const;
	
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

		//void CopyFrom(const dChannel<T, type>& source)
		//{
		//	dArray<T>& me = *this;
		//	dChannel& src = *((dChannel*)&source);
		//
		//	Clear();
		//	m_count = src.m_count;
		//	dAssert(m_type == src.m_type);
		//	for (dInt32 i = 0; i < m_count; i++) 
		//	{
		//		me[i] = src[i];
		//	}
		//}
		//void Reserve(dInt32 size)
		//{
		//	dArray<T>::Resize(size);
		//	m_count = size;
		//}
		//void SetCount(dInt32 count)
		//{
		//	if (m_count) 
		//	{
		//		dAssert(count >= 0);
		//		dAssert(m_count >= count);
		//		m_count = count;
		//	}
		//}

		
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

	protected:
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

	public:
	D_CORE_API dMeshEffect();
	D_CORE_API virtual ~dMeshEffect();

	D_CORE_API void CalculateNormals(dFloat64 angleInRadians);
	D_CORE_API void BuildFromIndexList(const dMeshVertexFormat* const format);

	dInt32 GetPropertiesCount() const;
	D_CORE_API void GetVertexChannel64(dInt32 strideInByte, dFloat64* const bufferOut) const;
	D_CORE_API void GetVertexChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_CORE_API void GetNormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_CORE_API void GetBinormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_CORE_API void GetUV0Channel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_CORE_API void GetUV1Channel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	D_CORE_API void GetVertexColorChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	//	void GetWeightBlendChannel(dInt32 strideInByte, dFloat32* const bufferOut) const;
	//	void GetWeightIndexChannel(dInt32 strideInByte, dInt32* const bufferOut) const;

	D_CORE_API ndIndexArray* MaterialGeometryBegin();
		D_CORE_API dInt32 GetFirstMaterial(ndIndexArray* const handle) const;
		D_CORE_API dInt32 GetNextMaterial(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_CORE_API dInt32 GetMaterialID(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_CORE_API dInt32 GetMaterialIndexCount(ndIndexArray* const handle, dInt32 materialHandle) const;
		D_CORE_API void GetMaterialGetIndexStream(ndIndexArray* const handle, dInt32 materialHandle, dInt32* const index) const;
		D_CORE_API void GetMaterialGetIndexStreamShort(ndIndexArray* const handle, dInt32 materialHandle, dInt16* const index) const;
	D_CORE_API void MaterialGeomteryEnd(ndIndexArray* const handle);

	D_CORE_API void BeginBuild();
	//	D_CORE_API void BeginBuildFace();
	//		D_CORE_API void AddPoint(dFloat64 x, dFloat64 y, dFloat64 z);
	//		D_CORE_API void AddLayer(dInt32 layer);
	//		D_CORE_API void AddMaterial(dInt32 materialIndex);
	//		D_CORE_API void AddNormal(dFloat32 x, dFloat32 y, dFloat32 z);
	//		D_CORE_API void AddBinormal(dFloat32 x, dFloat32 y, dFloat32 z);
	//		D_CORE_API void AddVertexColor(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w);
	//		D_CORE_API void AddUV0(dFloat32 u, dFloat32 v);
	//		D_CORE_API void AddUV1(dFloat32 u, dFloat32 v);
	//	D_CORE_API void EndBuildFace();
	D_CORE_API void EndBuild(dFloat64 tol, bool fixTjoint = true);

	D_CORE_API dBigVector GetOrigin()const;
	D_CORE_API void SphericalMapping(dInt32 material, const dMatrix& uvAligment);
	D_CORE_API void BoxMapping(dInt32 front, dInt32 side, dInt32 top, const dMatrix& uvAligment);

	protected:
	D_CORE_API void Init();
	D_CORE_API void RepairTJoints();
	D_CORE_API virtual void BeginFace();
	D_CORE_API virtual bool EndFace();

	bool Sanity() const;
	void PackAttibuteData();
	void UnpackAttibuteData();
	bool SeparateDuplicateLoops(dEdge* const face);
	dInt32 AddInterpolatedHalfAttribute(dEdge* const edge, dInt32 midPoint);

	dPointFormat m_points;
	dAttibutFormat m_attrib;
	dInt32 m_vertexBaseCount;
	dInt32 m_constructionIndex;
};

#if 0
inline dInt32 dMeshEffect::GetVertexCount() const
{
	return m_points.m_vertex.m_count;
}

inline dInt32 dMeshEffect::GetVertexBaseCount() const
{
	return m_vertexBaseCount;
}

inline void dMeshEffect::SetVertexBaseCount(dInt32 count)
{
	m_vertexBaseCount = count;
}


inline const dInt32* dMeshEffect::GetIndexToVertexMap() const
{
	return &m_attrib.m_pointChannel[0];
}

inline dBigVector dMeshEffect::GetVertex (dInt32 index) const
{
	dAssert(index >= 0);
	dAssert(index < m_points.m_vertex.m_count);
	return m_points.m_vertex[index];
}

inline bool dMeshEffect::HasLayersChannel() const
{
	return m_points.m_layers.m_count != 0;
}

inline dInt32 dMeshEffect::GetVertexLayer(dInt32 index) const
{
	dAssert(index >= 0);
	dAssert(index < m_points.m_vertex.m_count);
	return (m_points.m_layers.m_count) ? m_points.m_layers[index] : 0;
}


inline dInt32 dMeshEffect::GetVertexStrideInByte() const 
{
	return sizeof (dBigVector);
}

inline const dFloat64* dMeshEffect::GetVertexPool () const 
{
	return &m_points.m_vertex[0].m_x;
}

inline dMeshEffect* dMeshEffect::GetFirstLayer ()
{
	return GetNextLayer (IncLRU());
}

inline dMeshEffect* dMeshEffect::GetNextLayer (dMeshEffect* const layerSegment)
{
	if (!layerSegment) {
		return nullptr;
	}
	return GetNextLayer (layerSegment->IncLRU() - 1);
}


inline dFloat64 dMeshEffect::QuantizeCordinade(dFloat64 x) const
{
	dInt32 exp;
	dFloat64 mantissa = frexp(x, &exp);
	mantissa = DG_MESH_EFFECT_PRECISION_SCALE_INV * floor (mantissa * DG_MESH_EFFECT_PRECISION_SCALE);

	dFloat64 x1 = ldexp(mantissa, exp);
	return x1;
}
#endif

template<class T, dMeshEffect::dChannelType type>
dMeshEffect::dChannel<T, type>::dChannel()
	:dArray<T>()
	,m_type(type)
	,m_isValid(false)
{
}

template<class T, dMeshEffect::dChannelType type>
dMeshEffect::dChannel<T, type>::dChannel(const dChannel& source)
	:dArray<T>(source)
	,m_type(source.m_type)
	,m_isValid(source.m_isValid)
{
}

template<class T, dMeshEffect::dChannelType type>
dMeshEffect::dChannel<T, type>::~dChannel()
{
}

template<class T, dMeshEffect::dChannelType type>
void dMeshEffect::dChannel<T, type>::Clear()
{
	m_isValid = false;
	dArray<T>::Clear();
}

template<class T, dMeshEffect::dChannelType type>
void dMeshEffect::dChannel<T, type>::PushBack(const T& element)
{
	T tmp(element);
	m_isValid = true;
	dArray<T>::PushBack(tmp);
}

inline dMeshEffect::dPointFormat::dPointFormat()
	:m_layers()
	,m_vertex()
{
}

inline dMeshEffect::dPointFormat::dPointFormat(const dPointFormat& source)
	:m_layers(source.m_layers)
	, m_vertex(source.m_vertex)
{
}

inline dMeshEffect::dPointFormat::~dPointFormat()
{
}

inline void dMeshEffect::dPointFormat::Clear()
{
	m_layers.Clear();
	m_vertex.Clear();
}

inline void dMeshEffect::dPointFormat::SetCount(dInt32 count)
{
	m_layers.SetCount(count);
	m_vertex.SetCount(count);
}

inline dMeshEffect::dAttibutFormat::dAttibutFormat()
	:m_pointChannel()
	,m_materialChannel()
	,m_normalChannel()
	,m_binormalChannel()
	,m_colorChannel()
	,m_uv0Channel()
	,m_uv1Channel()
{
}

inline dMeshEffect::dAttibutFormat::dAttibutFormat(const dAttibutFormat& source)
	:m_pointChannel(source.m_pointChannel)
	,m_materialChannel(source.m_materialChannel)
	,m_normalChannel(source.m_normalChannel)
	,m_binormalChannel(source.m_binormalChannel)
	,m_colorChannel(source.m_colorChannel)
	,m_uv0Channel(source.m_uv0Channel)
	,m_uv1Channel(source.m_uv1Channel)
{
}

inline dMeshEffect::dAttibutFormat::~dAttibutFormat()
{
}

inline void dMeshEffect::dAttibutFormat::Clear()
{
	m_pointChannel.Clear();
	m_materialChannel.Clear();
	m_normalChannel.Clear();
	m_binormalChannel.Clear();
	m_colorChannel.Clear();
	m_uv0Channel.Clear();
	m_uv1Channel.Clear();
}

inline void dMeshEffect::dAttibutFormat::SetCount(dInt32 count)
{
	m_pointChannel.SetCount(count);
	m_materialChannel.SetCount(count);
	m_normalChannel.SetCount(count);
	m_binormalChannel.SetCount(count);
	m_colorChannel.SetCount(count);
	m_uv0Channel.SetCount(count);
	m_uv1Channel.SetCount(count);
}

inline dInt32 dMeshEffect::GetPropertiesCount() const
{
	return m_attrib.m_pointChannel.GetCount();
}


#endif
