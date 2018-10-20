/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxgeometryconverter.h
#ifndef _FBXSDK_UTILS_GEOMETRY_CONVERTER_H_
#define _FBXSDK_UTILS_GEOMETRY_CONVERTER_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxarray.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

class FbxManager;
class FbxMesh;
class FbxPatch;
class FbxNurbs;
class FbxNurbsSurface;
class FbxNurbsCurve;
class FbxWeightedMapping;
class FbxSurfaceEvaluator;

/** 
* This class provides the functionality to convert geometry nodes 
* attributes (FbxMesh, FbxNurbs and FbxPatch) and mainly focuses on the two
* major categories: Triangulation and conversion between NURBS and Patches surfaces. 
* \nosubgrouping
*/
class FBXSDK_DLL FbxGeometryConverter
{
public:
	/** Constructor.
    * \param pManager SDK object manager.
    */
    FbxGeometryConverter(FbxManager* pManager);

	/** Destructor.
    */
    ~FbxGeometryConverter();

    /** 
    * \name Triangulation
    */
    //@{
		/** Triangulate a mesh.
		  * \param pMesh     Pointer to the mesh to triangulate.
		  * \return          Pointer to the new triangulated mesh.
		  * \remarks         This method creates a new mesh, leaving the source mesh unchanged.
		  */
		FbxMesh* TriangulateMesh(FbxMesh const* pMesh);

		/** Triangulate a patch.
		  * \param pPatch     Pointer to the patch to triangulate.
		  * \return           Pointer to the new triangulated mesh.
		  * \remarks          The current deformations (skins & shapes) on the patch are also converted and applied to the
		  *                   resulting mesh.
		  */
		FbxMesh* TriangulatePatch(FbxPatch const* pPatch);

		/** Triangulate a nurb.
		  * \param pNurbs     Pointer to the nurb to triangulate.
		  * \return          Pointer to the new triangulated mesh.
		  * \remarks         The current deformations (skins and shapes) on the nurb are also converted and applied to the
		  *                  resulting mesh.
		  */
		FbxMesh* TriangulateNurbs(FbxNurbs const* pNurbs);

		/** Triangulate a mesh, patch or nurb contained in a node and preserve the
		  * skins and shapes animation channels.
		  * \param pNode     Pointer to the node containing the geometry to triangulate.
		  * \return          \c true on success, or \c false if the node attribute is not a mesh, a patch or a nurb.
		  * \remarks         See the remarks for functions TriangulateMesh(), TriangulatePatch() and TriangulateNurbs().
		  */
		bool TriangulateInPlace(FbxNode* pNode);

		/** Add an "alternate" geometry to the node.
		  * \param pNode                        Pointer to the node containing the geometry.
		  * \param pSrcGeom                     Pointer to the source geometry.
		  * \param pAltGeom                     Pointer to the alternate geometry.
		  * \param pSrcToAltWeightedMapping     Pointer to the weighted mapping table (optional).
		  * \param pConvertDeformations         Flag used only if parameter pSrcToAltWeightedMapping is a valid pointer to a weighted mapping table.
		  *                                     Set to \c true to convert deformations using the weighted mapping table.
		  * \return                             \c true on success, or \c false if the node attribute is not a mesh, a patch or a nurb.
		  * \remarks							  Although this method is publicly available, its only use in the FBX SDK is internal to some of
		  *                                     the conversion routines declared in this class.
		  */
		bool AddAlternateGeometry(FbxNode* pNode, FbxGeometry* pSrcGeom, FbxGeometry* pAltGeom, FbxWeightedMapping* pSrcToAltWeightedMapping, bool pConvertDeformations);

		/** Convert skins and shapes from source to destination geometry.
		  * \param pNode        Pointer to the node containing the geometry.
		  * \param pSrcGeom     Pointer to the source geometry.
		  * \param pDstGeom     Pointer to the destination geometry.
		  * \return             \c true on success, \c false otherwise.
		  * \remarks            Source and destination geometry must belong to the same node and must be linked by a geometry weighted map.
		  * \remarks			  Although this method is publicly available, its only use in the FBX SDK is internal to some of
		  *                     the conversion routines declared in this class.
		  */
		bool ConvertGeometryAnimation(FbxNode* pNode, FbxGeometry* pSrcGeom, FbxGeometry* pDstGeom);

		/** Compute a "vertex-correspondence" table that helps passing from source to destination geometry.
		  * \param pSrcGeom                     Pointer to the source geometry.
		  * \param pDstGeom                     Pointer to the destination geometry.
		  * \param pSrcToDstWeightedMapping     Pointer to the weighted mapping table.
		  * \param pSwapUV                      Set to \c true to swap UVs.
		  * \return                             \c true on success, \c false if the function fails to compute the correspondence.
		  * \remarks                            Skins and shapes are also converted to fit the alternate geometry.
		  */
		bool ComputeGeometryControlPointsWeightedMapping(FbxGeometry* pSrcGeom, FbxGeometry* pDstGeom, FbxWeightedMapping* pSrcToDstWeightedMapping, bool pSwapUV=false);
    //@}

    /** 
    * \name Geometry Conversion
    */
    //@{
		/** Convert from patch to nurb.
		  * \param pPatch     Pointer to the patch to convert.
		  * \return           Created nurb or \c NULL if the conversion fails.
		  * \remarks          The patch must be of type eBSpline, eBezier or eLinear.
		  */
		FbxNurbs* ConvertPatchToNurbs(FbxPatch *pPatch);

		/** Convert a patch contained in a node to a nurb. Use this function to preserve the patch's 
		  *   skins and shapes animation channels.
		  * \param pNode     Pointer to the node containing the patch.
		  * \return          \c true on success, \c false if the node attribute is not a patch.
		  * \remarks         The patch must be of type eBSpline, eBezier or eLinear.
		  */
		bool ConvertPatchToNurbsInPlace(FbxNode* pNode);

		/** Convert a patch to nurb surface.
		  * \param pPatch     Pointer to the patch to convert.
		  * \return           Created nurb surface or \c NULL if conversion fails.
		  * \remarks          The patch must be of type eBSpline, eBezier or eLinear.
		  */
		FbxNurbsSurface* ConvertPatchToNurbsSurface(FbxPatch *pPatch);

		/** Convert a patch contained in a node to a nurb surface. Use this function to preserve 
		  *   the patch's skins and shapes animation channels.
		  * \param pNode     Pointer to the node containing the patch.
		  * \return          \c true on success, \c false if the node attribute is not a patch.
		  * \remarks         The patch must be of type eBSpline, eBezier or eLinear.
		  */
		bool ConvertPatchToNurbsSurfaceInPlace(FbxNode* pNode);

		/** Convert a FbxNurbs to a FbxNurbsSurface
		  * \param pNurbs     Pointer to the original nurb
		  * \return          A FbxNurbsSurface that is equivalent to the original nurb.
		  */
		FbxNurbsSurface* ConvertNurbsToNurbsSurface( FbxNurbs* pNurbs );

		/** Convert a FbxNurbsSurface to a FbxNurbs
		  * \param pNurbs     Pointer to the original nurbs surface
		  * \return          A FbxNurbs that is equivalent to the original nurbs surface.
		  */
		FbxNurbs* ConvertNurbsSurfaceToNurbs( FbxNurbsSurface* pNurbs );

		/** Convert a nurb, contained in a node, to a nurbs surface. Use this function to preserve 
		  *   the nurb's skins and shapes animation channels.
		  * \param pNode     Pointer to the node containing the nurb.
		  * \return          \c true on success, \c false otherwise
		  */
		bool ConvertNurbsToNurbsSurfaceInPlace(FbxNode* pNode);

		/** Convert a nurb contained in a node to a nurbs surface. Use this function to preserve 
		  *   the nurb's skins and shapes animation channels.
		  * \param pNode     Pointer to the node containing the nurbs surface.
		  * \return          \c true on success, \c false otherwise
		  */
		bool ConvertNurbsSurfaceToNurbsInPlace(FbxNode* pNode);
    //@}

    /** 
    * \name Nurb UV and Links Swapping
    */
    //@{
		/** Flip UV and/or skin clusters of a nurb.
		  * \param pNurbs             Pointer to the Source nurb.
		  * \param pSwapUV           Set to \c true to swap the UVs.
		  * \param pSwapClusters     Set to \c true to swap the control point indices of clusters.
		  * \return                  A flipped FbxNurbs, or \c NULL if the function fails.
		  */
		FbxNurbs* FlipNurbs(FbxNurbs* pNurbs, bool pSwapUV, bool pSwapClusters);

		/** Flip UV and/or skin clusters of a nurb surface.
		  * \param pNurbs             Pointer to the Source nurb surface.
		  * \param pSwapUV           Set to \c true to swap the UVs.
		  * \param pSwapClusters     Set to \c true to swap the control point indices of clusters.
		  * \return                  A flipped FbxNurbsSurface, or \c NULL if the function fails.
		  */
		FbxNurbsSurface* FlipNurbsSurface(FbxNurbsSurface* pNurbs, bool pSwapUV, bool pSwapClusters);
    //@}

    /** 
    * \name Normals By Polygon Vertex Emulation
    */
    //@{
		/** Emulate normals by polygon vertex mode for a mesh.
		  * \param pMesh     Pointer to the mesh object.
		  * \return          \c true on success, \c false if the number of normals in the 
		  *                  mesh and in its associated shapes don't match the number of polygon
		  *                  vertices.
		  * \remarks         For applications that only supports normals by control points, 
		  *                  this function duplicates control points to equal the 
		  *                  number of polygon vertices. skins and shapes are also converted.
		  *                  As preconditions:
		  *                       -# polygons must have been created
		  *                       -# the number of normals in the mesh and in its associated shapes must match the 
		  *                          number of polygon vertices.
		  */
		bool EmulateNormalsByPolygonVertex(FbxMesh* pMesh);

		/** Create edge smoothing information from polygon-vertex mapped normals.
		  * Existing smoothing information is removed and edge data is created if
		  * none exists on the mesh.
		  * \param pMesh     The mesh used to generate edge smoothing.
		  * \return          \c true on success, \c false otherwise.
		  * \remarks         The edge smoothing data is placed on Layer 0 of the mesh.
		  *                  Normals do not need to be on Layer 0, since the first layer with
		  *                  per polygon vertex normals is used.
		  */
		bool ComputeEdgeSmoothingFromNormals( FbxMesh* pMesh ) const;

		/** Convert edge smoothing to polygon smoothing group.
		  * Existing smoothing information is replaced.
		  * 
		  * \param pMesh     The mesh that contains the smoothing to be converted.
		  * \param pIndex    The index of the layer smoothing to be converted.
		  * \return          \c true on success, \c false otherwise.
		  * \remarks         The smoothing group is bitwise.  Each bit of the integer represents
		  *                  one smoothing group.  Therefore, there is a maximum of 32 smoothing groups.
		  */
		bool ComputePolygonSmoothingFromEdgeSmoothing( FbxMesh* pMesh, int pIndex=0 ) const;

		/** Convert polygon smoothing group to edge smoothing.
		  * Existing smoothing information is replaced.
		  * 
		  * \param pMesh     The mesh that contains the smoothing to be converted.
		  * \param pIndex    The index of the layer smoothing to be converted
		  * \return          \c true on success, \c false otherwise.
		  */
		bool ComputeEdgeSmoothingFromPolygonSmoothing( FbxMesh* pMesh, int pIndex=0 ) const;
    //@}

	/** Replace node attribute with new one.
	  * \param pNode Pointer to the node which node attribute will be replaced.
	  * \param pNewNodeAttr new node attribute.
	  * \remark a)node attribute of all instance object will also be replaced;
	  *         b)the old node attribute will be destroyed.
	  */
	void ReplaceNodeAttribute(FbxNode* pNode, FbxNodeAttribute* pNewNodeAttr);

	/** Add a "triangulated mesh" geometry to the node.
	  * \param pNode Pointer to the node containing the geometry.
	  * \param pUVStepCoeff Coefficient factor for the U/V steps. Must be >= 1.
	  * \return \c true on success, \c false if the node attribute is not a mesh, 
	  * a patch or a nurb.
	  * \remarks The remarks relative to functions TriangulateMesh(), TriangulatePatch()
	  * , TriangulateNurbs() and TriangulateInPlace() are applicable.
	  */
	bool AddTriangulatedMeshGeometry(FbxNode* pNode, int pUVStepCoeff);

	/** Split Mesh Per Material.
	  * Each split mesh only has a single material on it.
	  * \param pMesh     The mesh that contains the smoothing to be converted.    
	  * \return          \c true on success, \c false otherwise.
	  * \remarks          It will work only on mesh that have material mapped "per-face" (Mapping Mode is FbxLayerElement::eByPolygon).
	  *                   It does NOT work on meshes with material mapped per-vertex/per-edge/etc.
	  *                   It will create as many meshes on output that there are materials applied to it. 
	  *                   If one mesh have some polygons with material A, some polygons with material B, 
	  *                   and some polygons with NO material, it should create 3 meshes after calling this function.
	  *                   The newly created meshes should be attached to the same FbxNode that hold the original FbxMesh.
	  *                   The original FbxMesh STAY UNCHANGED.
	  *                   Now, the new mesh will have Normals, UVs, vertex color, material and textures.
      *                   Does NOT support multi-layered materials.  The function will exit and return false if applied on a mesh containing multi-layered materials.
	  */
	bool SplitMeshPerMaterial(FbxMesh* pMesh);

	/** Split all the mesh in the scene.
	  * \param pScene    each mesh in the scene will be split.
	  * \return          \c true on success, \c false otherwise.
	  */
	bool SplitMeshesPerMaterial(FbxScene* pScene);

	/** Create LayerElement, and copy settings from pRefMesh to pNewMesh.
	  * \param pNewMesh    new mesh to create layerElement.
	  * \param pRefMesh    reference mesh, to copy layerElement settings from it. 
	  * \return          \c true on success, \c false otherwise.
	  */
	bool CreateAndCopyLayerElement(FbxMesh *pNewMesh, FbxMesh *pRefMesh);

	/** Set Normals, UVs and Vertex Color, when building new mesh.
	  * \remarks          If MappingMode is FbxLayerElement::eByPolygon,
	  *                   only pIsEndPolygon/pPolygonIndex are meaningful here,
	  *                   pIsSearched/pPolyPointIndex/pLoopIndex are not used here.
	  */
	bool SetLayerElements(FbxMesh *pNewMesh, FbxMesh *pMesh, int pPolygonIndex, int pPolyPointIndex, int pLoopIndex, bool pIsSearched, bool pIsEndPolygon);

	/** Triangulate a mesh(support without holes or with holes simple polygon)
	  * \param pMesh     Pointer to the mesh to triangulate.
	  * \param pStatus   The status is \c true if the triangulation is successful for the whole mesh.
	  * \return          Pointer to the new triangulated mesh.
	  * \remarks         This method creates a new mesh, leaving the source mesh unchanged.
	  */
	FbxMesh* TriangulateMeshAdvance(FbxMesh const* pMesh, bool& pStatus);

	 /**
	 * Merge multiple meshes to one mesh.
	 * The method will merge: 
	 * a) mesh vertex;
	 * b) mesh polygon;
	 * c) mesh edge;
	 * d) all mesh elements; only the layer 0 elements is merged.
	 * e) if there are skins for old mesh, merge these skins. The new skin clusters link to old skeletons.
	 *
	 * \param pMeshNodes FBX nodes that hold multiple meshes. These meshes will be merged.
	 * \param pNodeName	 Name of new mesh node.
	 * \param pScene     The scene that will contain the new mesh node.
	 * \return			 The new mesh node if merge successfully, otherwise NULL is returned.
	 * \remarks			 This method creates a new mesh, leaving the source mesh unchanged.
	 *                   The transform of new mesh node is: translate (0, 0, 0), rotation (0, 0, 0), scale (1, 1, 1).
	 *					 For layer element material, normal, smoothing, UV set, vertex color, binormal, tangent and polygon group,
	 *					 if any mesh misses these element, the merge for this kind of element is skipped.
	 *					 For layer element crease, hole, visibility and user data, if any mesh has such element, the kind of element
	 *                   will be merged. The missing element will be filled with default values.
	 *					 For meshes with skin binding, if the pose of frame 0 is different with bind pose, the new mesh will be distorted.
	 */
	FbxNode* MergeMeshes(FbxArray<FbxNode*>& pMeshNodes, const char* pNodeName, FbxScene* pScene);

    /** FbxTriangulation
    * \param Index Output array of triangle indices
    * \param pNumSide Input number of sides of the polygon to triangulate
    * -- Triangulation algorithm is strip, Sorting vertex index for strip in clockwise.
    *
    *          2           3         4
    *           0----------0---------0
    *          /                      `
    *         /                        `  5
    *      1 /                          0 
    *       0                          /
    *        `                        /
    *          `                     /
    *         0 0-----0-------------0 6
    *                 7
    *  The result of this one will be [{0,1,2},{2,3,0},{0,3,7},{3,4,7},{7,4,6},{4,5,6}]
    *
    */
    static void FbxTriangulation(int *Index, int pNumSide);

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
protected:
    bool ComputePatchToMeshControlPointsWeightedMapping(FbxPatch* pSrcPatch, FbxMesh* pDstMesh, FbxWeightedMapping* pMapping, bool pSwapUV=false);
    bool ComputeNurbsToMeshControlPointsWeightedMapping(FbxNurbsSurface* pSrcNurbs, FbxMesh* pDstMesh, FbxWeightedMapping* pMapping, bool pRescaleUVs=false, bool pSwapUV=false);

    void InitializeWeightInControlPoints(FbxGeometryBase* pGeometry);
    void InitializeWeightInNormals(FbxLayerContainer* pLayerContainer);
    void TriangulateContinuousSurface(FbxMesh* pMesh, FbxSurfaceEvaluator* pSurface, FbxUInt pPointCountX, FbxUInt pPointCountY, bool ClockWise=false);
    void CheckForZeroWeightInShape(FbxGeometry *pGeometry);
    FbxMesh* CreateMeshFromParametricSurface(FbxGeometry const* pGeometry);
    FbxNurbs* CreateNurbsFromPatch(FbxPatch* pPatch);
    FbxNurbsSurface* CreateNurbsSurfaceFromPatch(FbxPatch* pPatch);

    void ConvertShapes(FbxGeometry const* pSource, FbxGeometry* pDestination, FbxSurfaceEvaluator* pEvaluator, int pUCount, int pVCount);
    void ConvertShapes(FbxGeometry const* pSource, FbxGeometry* pDestination, FbxWeightedMapping* pSourceToDestinationMapping);
    void ConvertClusters(FbxGeometry const* pSource, FbxGeometry* pDestination, FbxWeightedMapping* pSourceToDestinationMapping);
    void ConvertClusters(FbxArray<FbxCluster*> const& pSourceClusters, int pSourceControlPointsCount, FbxArray<FbxCluster*>& pDestinationClusters, int pDestinationControlPointsCount, FbxWeightedMapping* pSourceToDestinationMapping);
    void BuildClusterToSourceMapping(FbxArray<FbxCluster*> const& pSourceClusters, FbxWeightedMapping* pClusterToSourceMapping);
    void CheckClusterToSourceMapping(FbxWeightedMapping* pClusterToSourceMapping);
    void ConvertCluster(int pSourceClusterIndex, FbxWeightedMapping* pClusterToSourceMapping, FbxWeightedMapping* pSourceToDestinationMapping, FbxCluster* pDestinationCluster);
    void DuplicateControlPoints(FbxArray<FbxVector4>& pControlPoints, FbxArray<int>& pPolygonVertices);
    void UpdatePolygon(FbxMesh *pNewMesh, FbxMesh const *pRefMesh, int pPolygonIndex, int* pNewIndex, int &pVerticeIndexMeshTriangulated, int &pPolygonIndexMeshTriangulated);
    void UpdatePolygon(FbxMesh *pNewMesh, FbxMesh const *pRefMesh, int pPolygonIndex, int* pNewIndex, int &pVerticeIndexMeshTriangulated, int &pPolygonIndexMeshTriangulated, int pTriangleNum);
    void ResizePolygon(FbxMesh *pNewMesh, int pNewCountVertices = 0, int pNewCountPolygons =0, bool pClearFlag = true);

    template <class T1, class T2> void ConvertNurbs(T1* pNewNurbs, T2* pOldNurb);

    bool CopyAnimationCurves(FbxNode* pNode, FbxGeometry* pNewGeometry);
    bool FlipNurbsCurve(FbxNurbsCurve* pCurve) const;
    void FlipControlPoints(FbxGeometryBase* pPoints, int pUCount, int pVCount) const;
    bool ConvertMaterialReferenceMode(FbxMesh* pMeshRef) const;
    void RevertMaterialReferenceModeConversion(FbxMesh* pMeshRef) const;

    FbxManager* mManager;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_UTILS_GEOMETRY_CONVERTER_H_ */
