/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MESH_EFFECT_H__
#define __ND_MESH_EFFECT_H__

#include "ndCollisionStdafx.h"

#define DG_MESH_EFFECT_PRECISION_BITS		48
#define DG_MESH_EFFECT_PRECISION_SCALE		ndFloat64(ndInt64(1)<<DG_MESH_EFFECT_PRECISION_BITS)
#define DG_MESH_EFFECT_PRECISION_SCALE_INV	(ndFloat64 (1.0f) / DG_MESH_EFFECT_PRECISION_SCALE)

#define DG_VERTEXLIST_INDEXLIST_TOL			(ndFloat64 (0.0f))
#define DG_MESH_EFFECT_POINT_SPLITED		512
#define DG_MESH_EFFECT_BVH_STACK_DEPTH		256

class ndShapeInstance;

class ndIndexArray
{
	public:
	ndInt32 m_materialCount;
	ndInt32 m_indexCount;
	ndInt32 m_materials[256];
	ndInt32 m_materialsIndexCount[256];
	ndInt32* m_indexList;
};

class ndMeshEffect: public ndPolyhedra
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
	ndMeshEffect (dMemoryAllocator___* const allocator, dgDeserialize deserialization, void* const userData);

	// create from OFF or PLY file format
	ndMeshEffect(dMemoryAllocator___* const allocator, const char* const fileName);

	// Create a convex hull Mesh form point cloud
	ndMeshEffect (dMemoryAllocator___* const allocator, const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol);

	// create a planar Mesh
	ndMeshEffect(dMemoryAllocator___* const allocator, const dMatrix& planeMatrix, dFloat32 witdth, dFloat32 breadth, dInt32 material, const dMatrix& textureMatrix0, const dMatrix& textureMatrix1);

	void Trace () const;
	void CylindricalMapping (dInt32 cylinderMaterial, dInt32 capMaterial, const dMatrix& uvAligment);
	void AngleBaseFlatteningMapping (dInt32 cylinderMaterial, dgReportProgress progressReportCallback, void* const userData);

	ndMeshEffect* Union (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	ndMeshEffect* Difference (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	ndMeshEffect* Intersection (const dMatrix& matrix, const ndMeshEffect* const clipper) const;
	void ClipMesh (const dMatrix& matrix, const ndMeshEffect* const clipper, ndMeshEffect** const top, ndMeshEffect** const bottom) const;

	//bool PlaneClip (const dBigPlane& plane);

	
	
	dInt32 GetVertexBaseCount() const;
	void SetVertexBaseCount(dInt32 count);
	
	dEdge* SpliteFace (dInt32 v0, dInt32 v1);

	dInt32 GetTotalFaceCount() const;
	dInt32 GetTotalIndexCount() const;
	void GetFaces (dInt32* const faceCount, dInt32* const materials, void** const faceNodeList) const;

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
	dInt32 InterpolateVertex (const dBigVector& point, const dEdge* const face) const;

	protected:
	dBigVector GetOrigin ()const;
	dInt32 CalculateMaxAttributes () const;
//	void ReverseMergeFaces (ndMeshEffect* const source);
	
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
	class dChannel: public ndArray<T>
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
			ndInt32 m_mask;
			ndInt32 m_ordinal;
			ndInt32 m_vertexIndex;
			ndInt32 m_attibuteIndex;
		};
		class dVertexSortData
		{
			public:
			const dChannel<ndBigVector, m_point>* m_points;
			ndInt32 m_vertexSortIndex;
		};

		ndInt32 GetSortIndex(const dChannel<ndBigVector, m_point>& points, ndFloat64& dist) const;
		static ndInt32 CompareVertex(const dSortKey* const ptr0, const dSortKey* const ptr1, void* const context);
		void GetStats(const dChannel<ndBigVector, m_point>& points, ndBigVector& min, ndBigVector& max, ndBigVector& origin, ndBigVector& median) const;
	};

	class dPointFormat: public dFormat
	{
		public:
		dPointFormat();
		dPointFormat(const dPointFormat& source);
		~dPointFormat();

		void Clear();
		void SetCount(ndInt32 count);
		void CompressData(ndInt32* const indexList);
		
		dChannel<ndInt32, m_layer> m_layers;
		dChannel<ndBigVector, m_point> m_vertex;
	};

	class dAttibutFormat: public dFormat
	{
		public:
		class dgUV
		{
			public:
			ndFloat32 m_u;
			ndFloat32 m_v;
		};

		dAttibutFormat();
		dAttibutFormat(const dAttibutFormat& source);
		~dAttibutFormat();

		void Clear();
		void SetCount(ndInt32 count);
		void CopyFrom(const dAttibutFormat& source);
		void CopyEntryFrom(ndInt32 index, const dAttibutFormat& source, ndInt32 sourceIndex);
		void CompressData(const dPointFormat& points, ndInt32* const indexList);

		private:
		void CompressDataLow(const dPointFormat& points, ndInt32* const indexList);

		public:
		dChannel<ndInt32, m_vertex> m_pointChannel;
		dChannel<ndInt32, m_material> m_materialChannel;
		dChannel<ndTriplex, m_normal> m_normalChannel;
		dChannel<ndTriplex, m_binormal> m_binormalChannel;
		dChannel<ndVector, m_color> m_colorChannel;
		dChannel<dgUV, m_uv0> m_uv0Channel;
		dChannel<dgUV, m_uv1> m_uv1Channel;
	};

	public:
	class dMaterial
	{
		public:
		dMaterial()
			:m_ambient(ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(1.0f))
			,m_diffuse(ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(1.0f))
			,m_specular(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f))
			,m_opacity(ndFloat32(1.0f))
			,m_shiness(ndFloat32 (60.0f))
		{
			m_textureName[0] = 0;
		}

		ndVector m_ambient;
		ndVector m_diffuse;
		ndVector m_specular;
		ndFloat32 m_opacity;
		ndFloat32 m_shiness;
		char m_textureName[32];
	};

	class dMeshVertexFormat
	{
		public:
		class dDoubleData
		{
			public:
			const ndFloat64* m_data;
			const ndInt32* m_indexList;
			ndInt32 m_strideInBytes;
		};

		class dFloatData
		{
			public:
			const ndFloat32* m_data;
			const ndInt32* m_indexList;
			ndInt32 m_strideInBytes;
		};

		dMeshVertexFormat()
		{
			Clear();
		}

		void Clear()
		{
			memset(this, 0, sizeof(dMeshVertexFormat));
		}

		ndInt32 m_faceCount;
		const ndInt32* m_faceIndexCount;
		const ndInt32* m_faceMaterial;
		dDoubleData m_vertex;
		dFloatData m_normal;
		dFloatData m_binormal;
		dFloatData m_uv0;
		dFloatData m_uv1;
		dFloatData m_vertexColor;
	};

	class dVertexCluster
	{
		public:
		dVertexCluster()
		{
		}
		ndArray<ndInt32> m_vertexIndex;
		ndArray<ndFloat32> m_vertexWeigh;
	};

	class dClusterMap: public ndTree<dVertexCluster, const ndString>
	{
	};
	
	D_COLLISION_API ndMeshEffect();
	D_COLLISION_API ndMeshEffect(const ndMeshEffect& source);
	D_COLLISION_API ndMeshEffect(const ndShapeInstance& shape);
	D_COLLISION_API ndMeshEffect(ndPolyhedra& mesh, const ndMeshEffect& source);
	
	// Create a convex hull Mesh from point cloud
	D_COLLISION_API ndMeshEffect(const ndFloat64* const vertexCloud, ndInt32 count, ndInt32 strideInByte, ndFloat64 distTol);

	D_COLLISION_API virtual ~ndMeshEffect();

	void SetName (const ndString& name);
	const ndString& GetName() const;

	ndArray<dMaterial>& GetMaterials();
	ndInt32 GetPropertiesCount() const;

	ndInt32 GetVertexCount() const;
	ndInt32 GetVertexStrideInByte() const;
	const ndFloat64* GetVertexPool() const;

	ndInt32 GetFaceMaterial(ndEdge* const faceEdge) const;

	const dClusterMap& GetCluster() const;
	D_COLLISION_API dVertexCluster* CreateCluster(const char* const name);
	D_COLLISION_API dVertexCluster* FindCluster(const char* const name) const;

	D_COLLISION_API ndFloat64 CalculateVolume() const;
	D_COLLISION_API ndMatrix CalculateOOBB(ndBigVector& size) const;
	D_COLLISION_API void CalculateAABB(ndBigVector& min, ndBigVector& max) const;

	D_COLLISION_API void ApplyTransform(const ndMatrix& matrix);
	D_COLLISION_API void CalculateNormals(ndFloat64 angleInRadians);
	D_COLLISION_API void BuildFromIndexList(const dMeshVertexFormat* const format);
	
	D_COLLISION_API void GetVertexIndexChannel(ndInt32* const bufferOut) const;
	D_COLLISION_API void GetVertexChannel64(ndInt32 strideInByte, ndFloat64* const bufferOut) const;
	D_COLLISION_API void GetVertexChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;
	D_COLLISION_API void GetNormalChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;
	D_COLLISION_API void GetBinormalChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;
	D_COLLISION_API void GetUV0Channel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;
	D_COLLISION_API void GetUV1Channel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;
	D_COLLISION_API void GetVertexColorChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const;

	D_COLLISION_API ndIndexArray* MaterialGeometryBegin();
		D_COLLISION_API ndInt32 GetFirstMaterial(ndIndexArray* const handle) const;
		D_COLLISION_API ndInt32 GetNextMaterial(ndIndexArray* const handle, ndInt32 materialHandle) const;
		D_COLLISION_API ndInt32 GetMaterialID(ndIndexArray* const handle, ndInt32 materialHandle) const;
		D_COLLISION_API ndInt32 GetMaterialIndexCount(ndIndexArray* const handle, ndInt32 materialHandle) const;
		D_COLLISION_API void GetMaterialGetIndexStream(ndIndexArray* const handle, ndInt32 materialHandle, ndInt32* const index) const;
		D_COLLISION_API void GetMaterialGetIndexStreamShort(ndIndexArray* const handle, ndInt32 materialHandle, ndInt16* const index) const;
	D_COLLISION_API void MaterialGeometryEnd(ndIndexArray* const handle);

	D_COLLISION_API void BeginBuild();
		D_COLLISION_API void BeginBuildFace();
			D_COLLISION_API void AddPoint(ndFloat64 x, ndFloat64 y, ndFloat64 z);
			D_COLLISION_API void AddLayer(ndInt32 layer);
			D_COLLISION_API void AddMaterial(ndInt32 materialIndex);
			D_COLLISION_API void AddNormal(ndFloat32 x, ndFloat32 y, ndFloat32 z);
			D_COLLISION_API void AddBinormal(ndFloat32 x, ndFloat32 y, ndFloat32 z);
			D_COLLISION_API void AddVertexColor(ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w);
			D_COLLISION_API void AddUV0(ndFloat32 u, ndFloat32 v);
			D_COLLISION_API void AddUV1(ndFloat32 u, ndFloat32 v);
		D_COLLISION_API void EndBuildFace();
	D_COLLISION_API void EndBuild(ndFloat64 tol, bool fixTjoint = true);

	D_COLLISION_API ndBigVector GetOrigin()const;
	D_COLLISION_API void SphericalMapping(ndInt32 materialIndex, const ndMatrix& textureMatrix);
	D_COLLISION_API void UniformBoxMapping(ndInt32 materialIndex, const ndMatrix& textureMatrix);
	D_COLLISION_API void BoxMapping(ndInt32 front, ndInt32 side, ndInt32 top, const ndMatrix& textureMatrix);
	D_COLLISION_API void RepairTJoints();

	ndMeshEffect* GetFirstLayer();
	ndMeshEffect* GetNextLayer(ndMeshEffect* const layer);
	
	D_COLLISION_API void FlipWinding();
	D_COLLISION_API bool HasOpenEdges() const;
	D_COLLISION_API void Triangulate();
	D_COLLISION_API void ConvertToPolygons();
	D_COLLISION_API ndEdge* InsertEdgeVertex(ndEdge* const edge, ndFloat64 param);
	D_COLLISION_API void AddInterpolatedEdgeAttribute(ndEdge* const edge, ndFloat64 param);
	D_COLLISION_API void RemoveUnusedVertices(ndInt32* const vertexRemapTable);
	D_COLLISION_API ndInt32 PlaneClip(const ndMeshEffect& convexMesh, const ndEdge* const face);
	D_COLLISION_API ndShapeInstance* CreateConvexCollision(ndFloat64 tolerance) const;
	D_COLLISION_API ndMeshEffect* ConvexMeshIntersection(const ndMeshEffect* const convexMesh) const;
	D_COLLISION_API ndMeshEffect* InverseConvexMeshIntersection(const ndMeshEffect* const convexMesh) const;
	D_COLLISION_API ndMeshEffect* CreateVoronoiConvexDecomposition(const ndArray<ndVector>& pointCloud, ndInt32 interiorMaterialIndex, const ndMatrix& textureProjectionMatrix);

	protected:
	D_COLLISION_API void Init();
	D_COLLISION_API virtual void BeginFace();
	D_COLLISION_API virtual bool EndFace();
	ndFloat64 QuantizeCordinade(ndFloat64 val) const;

	bool Sanity() const;
	void PackPoints(ndFloat64 tol);
	void UnpackPoints();
	void PackAttibuteData();
	void UnpackAttibuteData();
	bool SeparateDuplicateLoops(ndEdge* const face);
	ndInt32 AddInterpolatedHalfAttribute(ndEdge* const edge, ndInt32 midPoint);
	
	void MergeFaces(const ndMeshEffect* const source);
	D_COLLISION_API ndMeshEffect* GetNextLayer(ndInt32 mark);

	ndString m_name;
	dPointFormat m_points;
	dAttibutFormat m_attrib;
	dClusterMap m_clusters;
	ndArray<dMaterial> m_materials;
	ndInt32 m_vertexBaseCount;
	ndInt32 m_constructionIndex;
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
#endif

inline ndFloat64 ndMeshEffect::QuantizeCordinade(ndFloat64 x) const
{
	ndInt32 exp;
	ndFloat64 mantissa = frexp(x, &exp);
	mantissa = DG_MESH_EFFECT_PRECISION_SCALE_INV * floor (mantissa * DG_MESH_EFFECT_PRECISION_SCALE);

	ndFloat64 x1 = ldexp(mantissa, exp);
	return x1;
}

template<class T, ndMeshEffect::dChannelType type>
ndMeshEffect::dChannel<T, type>::dChannel()
	:ndArray<T>()
	,m_type(type)
	,m_isValid(false)
{
}

template<class T, ndMeshEffect::dChannelType type>
ndMeshEffect::dChannel<T, type>::dChannel(const dChannel& source)
	:ndArray<T>(source)
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
	ndArray<T>::Clear();
}

template<class T, ndMeshEffect::dChannelType type>
void ndMeshEffect::dChannel<T, type>::PushBack(const T& element)
{
	T tmp(element);
	m_isValid = true;
	ndArray<T>::PushBack(tmp);
}

inline ndMeshEffect::dPointFormat::dPointFormat()
	:m_layers()
	,m_vertex()
{
}

inline ndMeshEffect::dPointFormat::dPointFormat(const dPointFormat& source)
	:m_layers(source.m_layers)
	,m_vertex(source.m_vertex)
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

inline void ndMeshEffect::dPointFormat::SetCount(ndInt32 count)
{
	m_vertex.Resize(count);
	m_vertex.SetCount(count);

	m_layers.Resize(count);
	m_layers.SetCount(count);
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

inline void ndMeshEffect::dAttibutFormat::SetCount(ndInt32 count)
{
	m_pointChannel.Resize(count);
	m_pointChannel.SetCount(count);

	m_materialChannel.Resize(count);
	m_materialChannel.SetCount(count);

	m_normalChannel.Resize(count);
	m_normalChannel.SetCount(count);

	m_binormalChannel.Resize(count);
	m_binormalChannel.SetCount(count);

	m_colorChannel.Resize(count);
	m_colorChannel.SetCount(count);

	m_uv0Channel.Resize(count);
	m_uv0Channel.SetCount(count);

	m_uv1Channel.Resize(count);
	m_uv1Channel.SetCount(count);
}

inline ndInt32 ndMeshEffect::GetPropertiesCount() const
{
	return m_attrib.m_pointChannel.GetCount();
}

inline void ndMeshEffect::SetName(const ndString& name)
{
	m_name = name;
}

inline const ndString& ndMeshEffect::GetName() const
{
	return m_name;
}

inline ndArray<ndMeshEffect::dMaterial>& ndMeshEffect::GetMaterials()
{
	return m_materials;
}

inline ndInt32 ndMeshEffect::GetVertexCount() const
{
	return m_points.m_vertex.GetCount();
}

inline ndInt32 ndMeshEffect::GetVertexStrideInByte() const
{
	return sizeof(ndBigVector);
}

inline const ndFloat64* ndMeshEffect::GetVertexPool() const
{
	return &m_points.m_vertex[0].m_x;
}

inline ndInt32 ndMeshEffect::GetFaceMaterial(ndEdge* const faceEdge) const
{
	return ndInt32(m_attrib.m_materialChannel.GetCount() ? m_attrib.m_materialChannel[ndInt32(faceEdge->m_userData)] : 0);
}

inline ndMeshEffect* ndMeshEffect::GetFirstLayer()
{
	return GetNextLayer(IncLRU());
}

inline ndMeshEffect* ndMeshEffect::GetNextLayer(ndMeshEffect* const layerSegment)
{
	if (!layerSegment) 
	{
		return nullptr;
	}
	return GetNextLayer(layerSegment->IncLRU() - 1);
}

inline const ndMeshEffect::dClusterMap& ndMeshEffect::GetCluster() const
{
	return m_clusters;
}

#endif
