/////////////////////////////////////////////////////////////////////////////
// Name:        ColladaImport.cpp
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "StdAfx.h"
#include "dPlugInStdafx.h"
#include "ColladaImport.h"
//#include "dGeometryNodeSkinModifierInfo.h"

ColladaImport::SourceBuffer::SourceBuffer (domSource* source)
{
	if (source) {
		domFloat_array *floatArray = source->getFloat_array();
		domListOfFloats &srcArray = floatArray->getValue();

		domSource::domTechnique_common *technique = source->getTechnique_common();
		_ASSERTE (technique);

		domAccessor *accessor = technique->getAccessor();
		_ASSERTE (accessor);

		int stride = int (accessor->getStride());
		int count = int (accessor->getCount());

		m_count = count;
		m_data = new dVector[count];
		for (int i = 0 ; i < count; i ++) {
			int index = i * stride;
			dVector p(0.0f, 0.0f, 0.0f, 0.0f);
			for (int j = 0; j < stride; j ++) {
				p[j] = dFloat (srcArray[index + j]);
			}
			m_data[i] = p;
		}

	} else {
		m_count = 1;
		m_data = new dVector[1];
		m_data[0] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	}
}

ColladaImport::SourceBuffer::~SourceBuffer ()
{
	delete[] m_data;
}



ColladaImport::ColladaImport()
	:dImportPlugin()
{
#if 0
NewtonWorld* xxx = NewtonCreate ();
dScene xxxx(xxx);
Import ("D:/NewtonWin-2.21/samples/sdkDemos/coefficients_of_friction.dae", &xxxx);
xxxx.Serialize("D:/NewtonWin-2.21/samples/sdkDemos/coefficients_of_friction.xml");
#endif
}

ColladaImport::~ColladaImport()
{
}


ColladaImport* ColladaImport::GetPlugin()
{
	static ColladaImport gImporter;
	return &gImporter;
}


dMatrix ColladaImport::GetMatrix (domNode* node) const
{
	dMatrix matrix (GetIdentityMatrix());
	domMatrix_Array matrixArray = node->getMatrix_array();
	if (matrixArray.getCount()) {
		// parse the matrix by concatenating each of the individual components	
		for (int i = 0; i < int (matrixArray.getCount()); i ++) {
			const domFloat4x4& data = matrixArray[i]->getValue();
			dMatrix colladaMatrix; 
			for (int j = 0; j < 4; j ++) {
				for (int k = 0; k < 4; k ++) {
					colladaMatrix[k][j] = dFloat (data[j * 4 + k]);
				}
			}
			matrix = colladaMatrix * matrix;
		}
	} else {
		// or parse the matrix by concatenating each of the individual components
		domTranslate_Array &translationArray = node->getTranslate_array();
		for (int i = 0; i < int (translationArray.getCount()); i ++) {
			const domFloat3& data = translationArray[i]->getValue();
			matrix[3][0] += dFloat (data[0]);
			matrix[3][1] += dFloat (data[1]);
			matrix[3][2] += dFloat (data[2]);
		}

		domRotate_Array &rotationArray = node->getRotate_array();
		for (int i = 0; i < int (rotationArray.getCount()); i ++) {
			dFloat angle;
			const domFloat4& data = rotationArray[i]->getValue();
			angle = dFloat (data[3]) * 3.1316f / 180.0f;
			dFloat x = dFloat (data[0]);
			dFloat y = dFloat (data[1]);
			dFloat z = dFloat (data[2]);
			dVector axis (x, y, z, 0.0f);
			axis = axis.Scale (1.0f / sqrtf (axis % axis));
			dQuaternion rot (axis, angle);
			dMatrix colladaMatrix (rot, dVector (0.0f, 0.0f, 0.0f, 1.0f));
			matrix = colladaMatrix * matrix;
		}
	}
	return matrix;
}

void ColladaImport::LoadNodeData (DAE* collada, 
	dScene::dTreeNode* node, domNode* colladaNode, dScene* world, 
	GeometryChache& meshCache, ImageCache& imageCache)
{
	dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) world->GetInfoFromNode(node); 

	// get the name ID (note: many exporters do not obey Collada specifications and use the Id instead of the Name) 
	if (colladaNode->getSid()) {
		sceneInfo->SetName(colladaNode->getSid());
	} else if (colladaNode->getName()) {
		sceneInfo->SetName(colladaNode->getName());
	} else {
		sceneInfo->SetName(colladaNode->getId());
	}


	dMatrix matrix ( m_globalRotation.Inverse() * GetMatrix (colladaNode) * m_globalRotation);
	matrix.m_posit = matrix.m_posit.Scale (m_scale);
//	bone->SetMatrix(matrix);
	sceneInfo->SetTransform(matrix);

	// instance controller may make reference to nodes that may not being read yet, wee need to read them in a second pass.
	const domInstance_controller_Array& controllerArray = colladaNode->getInstance_controller_array();
	if (controllerArray.getCount()) {
		domInstance_controller* instance = controllerArray[0];
		daeURI uri (instance->getUrl());
		const daeElement* element = uri.getElement();
		domController* controller = (domController*) element;
		domSkin* skin = controller->getSkin();

		daeURI uriMesh (skin->getSource());
		element = uriMesh.getElement();
		domGeometry* colladaGeometry = (domGeometry*) element;

		//CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
		GeometryChache::dTreeNode* cacheNode = meshCache.Find(colladaGeometry);
		if (!cacheNode) {
			dScene::dTreeNode* mesh = NULL;
			// Load this Mesh for the firstTime time
			if (colladaGeometry->getMesh()) {
				//mesh = LoadMesh (collGeometry->getMesh(), context, modifierVertexMapCache, materialCache, imageCache);
				mesh = LoadMesh (collada, colladaGeometry->getMesh(), node, world, imageCache);
			} else if (colladaGeometry->getConvex_mesh()){
				_ASSERTE (0);
				//mesh = LoadConvexMesh (collGeometry->getConvex_mesh(), context, modifierVertexMapCache, materialCache, imageCache);
			} else {
				_ASSERTE (0);
				//geometry = new dGeometry;
			}

			const char* name = colladaGeometry->getId();
			dGeometryNodeInfo* geom = (dGeometryNodeInfo*) world->GetInfoFromNode(mesh);
			geom->SetName(name);
			cacheNode = meshCache.Insert(mesh, colladaGeometry);
			_ASSERTE (cacheNode);
		} else {
			dScene::dTreeNode* mesh = NULL;
			mesh = cacheNode->GetInfo();
			world->AddReference (node, mesh);
		}
	}


	// read the Mesh instantiated by this node if there are not read yet
	const domInstance_geometry_Array& gemeortyArray = colladaNode->getInstance_geometry_array();
	if (gemeortyArray.getCount()) {
		// for know we will only allow one mesh per node, as the dModel format does no support array of dMesh per nodes yet
		// it will be in the future
		dScene::dTreeNode* mesh = NULL;
		domInstance_geometry* instance = gemeortyArray[0];

		// get the pointer to the Mesh instance from the uri
		const daeURI& uri = instance->getUrl();
		const daeElement* element = uri.getElement();
		if (element) {
			domGeometry* colladaGeometry = (domGeometry*) element;

			// if the mesh is not in the model, we must load this new mesh instance
			GeometryChache::dTreeNode* cachedNode = meshCache.Find(colladaGeometry);
			if (!cachedNode) {
				// Load this Mesh for the firseTime time
				if (colladaGeometry->getMesh()) {
//					mesh = LoadMesh (collGeometry->getMesh(), context, modifierVertexMapCache, materialCache, imageCache);
					mesh = LoadMesh (collada, colladaGeometry->getMesh(), node, world, imageCache);
				} else if (colladaGeometry->getConvex_mesh()){
					_ASSERTE (0);
//					mesh = LoadConvexMesh (collGeometry->getConvex_mesh(), context, modifierVertexMapCache, materialCache, imageCache);
				} else {
					_ASSERTE (0);
//					geometry = new dGeometry;
				}
				const char* name = colladaGeometry->getId();

				dGeometryNodeInfo* geom = (dGeometryNodeInfo*) world->GetInfoFromNode(mesh);
				geom->SetName(name);
				meshCache.Insert(mesh, colladaGeometry);
			} else {
				mesh = cachedNode->GetInfo();
				world->AddReference (node, mesh);
			}
		}
	}
}



void ColladaImport::GetOffsetsAndSources (
	const domInputLocalOffset_Array& inputArray, 
	int& vertexOffset, int& normalOffset, int& uv0Offset, int& uv1Offset, 
	domSource** positSrc, domSource** normalSrc, domSource** uv0Src, domSource** uv1Src)
{
	uv0Offset = -1;
	uv1Offset = -1;
	vertexOffset = -1;
	normalOffset = -1;
	int sourcesCount = int (inputArray.getCount());
	for (int k = 0; k < sourcesCount; k ++) {
		domInputLocalOffset* offset = inputArray[k];

		daeURI uri (offset->getSource());
		daeElement* element = uri.getElement();
		_ASSERTE (element);

		if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_VERTEX)) {
			vertexOffset = int (offset->getOffset());

			domVertices* vertices = (domVertices*) element;
			domInputLocal_Array &inputArray = vertices->getInput_array();
			for (int i = 0; i < int (inputArray.getCount()); i ++) {
				domInputLocal* offset = inputArray[i];
				domInputLocal* input = inputArray[i];
				if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_POSITION)) {
					daeURI uri1 (input->getSource());
					element = uri1.getElement();
					_ASSERTE (element);
					domSource *source = (domSource *) element;
					_ASSERTE (!*positSrc || *positSrc == source);
					*positSrc = source;
					break;
				}
			}

		} else if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_NORMAL)) {
			normalOffset = int (offset->getOffset());

			domSource *source = (domSource*) element;

			_ASSERTE (!*normalSrc || *normalSrc == source);
			*normalSrc = source;

		} else if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_TEXCOORD)) {

			domSource *source = (domSource*) element;
			if (offset->getSet() == 0) {
				uv0Offset = int (offset->getOffset());
				_ASSERTE (!*uv0Src || *uv0Src == source);
				*uv0Src = source;
			} else {
				uv1Offset = int (offset->getOffset());
				_ASSERTE (!*uv1Src || *uv1Src == source);
				*uv1Src = source;
			}

		} else {
			_ASSERTE (0);
		}
	}
}


void ColladaImport::LoadPolygons(DAE* collada, 
	 dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
	 domPolygons_Array &polygonArray, int* faceIndexCount, int* materialIndex, 
	 int* vertex, int* normal, int* uv0, int* uv1,
	 domSource** positSrc, domSource** normalSrc, domSource** uv0Src, domSource** uv1Src)
{
	// load the triangles
	int index = 0;
	int faceCount = 0;
	for (int j = 0; j < int (polygonArray.getCount()); j ++) {
		domPolygons* polygons = polygonArray[j];

		int materilID = CreateMaterial (collada, polygons->getMaterial(), imageCache, world, gemetryNode);
		int uv0Offset = -1;
		int uv1Offset = -1;
		int vertexOffset = -1;
		int normalOffset = -1;
		const domInputLocalOffset_Array& inputArray = polygons->getInput_array();
		GetOffsetsAndSources (inputArray, vertexOffset, normalOffset, uv0Offset, uv1Offset, positSrc, normalSrc, uv0Src, uv1Src);

		int sourcesCount = int (inputArray.getCount());

		int polygonCount = int (polygons->getCount());
		domP_Array& indexArray = polygons->getP_array();
		_ASSERTE (int (indexArray.getCount()) == polygonCount);
		for (int k = 0; k < polygonCount; k ++) {
			domPRef pIndices = indexArray[k];
			domListOfUInts& dataIndices = pIndices->getValue();

			int vCount = dataIndices.getCount();
			faceIndexCount[faceCount] = vCount  / sourcesCount;
			materialIndex[faceCount] = materilID;
			faceCount ++;

			for (int k = 0 ; k < vCount; k += sourcesCount) {
				vertex[index] = int (dataIndices[k + vertexOffset]);

				if (normalOffset != -1) {
					normal[index] = int (dataIndices [k + normalOffset]);
				} else {
					normal[index] = 0;
				}

				if (uv0Offset != -1) {
					uv0[index] = int (dataIndices[k + uv0Offset]);
				} else {
					uv0[index] = 0;
				}

				if (uv1Offset != -1) {
					uv1[index] = int (dataIndices[k + uv1Offset]);
				} else {
					uv1[index] = 0;
				}
				index ++;
			}
		}
	}
}


void ColladaImport::LoadPolygonsList(DAE* collada, 
	 dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
	 domPolylist_Array &polygonListArray,  
	 int* faceIndexCount, int* materialIndex, 
	 int* vertex, int* normal, int* uv0, int* uv1,
	 domSource** positSrc, domSource** normalSrc, domSource** uv0Src, domSource** uv1Src)
{
	// load the polygonList array
	int index = 0;
	int faceCount = 0;
	for (int j = 0; j < int (polygonListArray.getCount()); j ++) {

		domPolylist* polygon = polygonListArray[j];

		int materilID = CreateMaterial (collada, polygon->getMaterial(), imageCache, world, gemetryNode);

		int uv0Offset = -1;
		int uv1Offset = -1;
		int vertexOffset = -1;
		int normalOffset = -1;
		const domInputLocalOffset_Array& inputArray = polygon->getInput_array();
		GetOffsetsAndSources (inputArray, vertexOffset, normalOffset, uv0Offset, uv1Offset, positSrc, normalSrc, uv0Src, uv1Src);

		int stride = int (polygon->getInput_array().getCount());

		domPolylist::domVcount* vcount = polygon->getVcount();
		const domListOfUInts& vIndices = vcount->getValue();

		domPRef elemP = polygon->getP();
		const domListOfUInts& dataIndices = elemP->getValue();
		const domUint* indexP = &dataIndices[0];

		//		int pointsStride = int (positSrc->m_stride);
		int pointcount = int (vIndices.getCount());


		for (int k = 0; k < pointcount; k ++) {
			int count = int (vIndices[k]);
			faceIndexCount[faceCount] = count;
			materialIndex[faceCount] = materilID;

			faceCount ++;
			for (int k = 0; k < count; k ++) {
				vertex[index] = int (indexP[k + vertexOffset]);

				if (normalOffset != -1) {
					normal[index] = int (indexP[k + normalOffset]);
				} else {
					normal[index] = 0;
				}

				if (uv0Offset != -1) {
					uv0[index] = int (indexP[k + uv0Offset]);
				} else {
					uv0[index] = 0;
				}

				if (uv1Offset != -1) {
					uv1[index] = int (indexP[k + uv1Offset]);
				} else {
					uv1[index] = 0;
				}
				index ++;
			}
			indexP += count * stride;
		}
	}
}


void ColladaImport::LoadTriangles(DAE* collada, 
	dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
	domTriangles_Array& trianglesArray, 
	int* faceIndexCount, int* materialIndex, 
	int* vertex, int* normal, int* uv0, int* uv1,
	domSource** positSrc, domSource** normalSrc, domSource** uv0Src, domSource** uv1Src)
{
	// load the triangles
	int index = 0;
	for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
		domTriangles* triangles = trianglesArray[j];;

		int materilID = CreateMaterial (collada, triangles->getMaterial(), imageCache, world, gemetryNode);
		int uv0Offset = -1;
		int uv1Offset = -1;
		int vertexOffset = -1;
		int normalOffset = -1;
		const domInputLocalOffset_Array& inputArray = triangles->getInput_array();
		GetOffsetsAndSources (inputArray, vertexOffset, normalOffset, uv0Offset, uv1Offset, positSrc, normalSrc, uv0Src, uv1Src);

		int sourcesCount = int (inputArray.getCount());
		domP *p = triangles->getP();
		domListOfUInts& indices = p->getValue();

		int indexCount = int (indices.getCount());
		for (int k = 0; k < indexCount; k += sourcesCount) {
			vertex[index] = int (indices[k + vertexOffset]);

			faceIndexCount[index/3] = 3;
			materialIndex[index/3] = materilID;

			if (normalOffset != -1) {
				normal[index] = int (indices [k + normalOffset]);
			} else {
				normal[index] = 0;
			}

			if (uv0Offset != -1) {
				uv0[index] = int (indices[k + uv0Offset]);
			} else {
				uv0[index] = 0;
			}

			if (uv1Offset != -1) {
				uv1[index] = int (indices[k + uv1Offset]);
			} else {
				uv1[index] = 0;
			}

			index ++;
		}
	}
}


dScene::dTreeNode* ColladaImport::LoadMesh (DAE* collada, domMesh* colladaMesh, dScene::dTreeNode* parent, dScene* world, ImageCache& imageCache)
{
	dScene::dTreeNode* node = world->CreateMeshNode(parent);

	// calculate the total triangle count
	int faceCount = 0;
	int indexCount = 0;
	int polygonListFaceStart = faceCount;
	int polygonListFaceIndexStart = indexCount;
	domPolylist_Array &polygonListArray = colladaMesh->getPolylist_array();
	for (int j = 0; j < int (polygonListArray.getCount()); j ++) {
		_ASSERTE (0);
		domPolylist* polygon = polygonListArray[j];
		domPolylist::domVcount* vcount = polygon->getVcount();
		const domListOfUInts& vIndices = vcount->getValue();

		faceCount += int (vIndices.getCount());
		for (int k = 0; k < int (vIndices.getCount()); k ++) {
			int count = int (vIndices[k]);
			//vertexCount += (count - 2) * 3;
			indexCount += count; 
		}
	}
	

	// count polygons
	int polygonFaceStart = faceCount;
	int polygonFaceIndexStart = indexCount;
	domPolygons_Array &polygonArray = colladaMesh->getPolygons_array();
	for (int j = 0; j < int (polygonArray.getCount()); j ++) {
		domPolygons* polygons = polygonArray[j];
		int polygonCount = int (polygons->getCount());
		int stride = int (polygons->getInput_array().getCount());
		domP_Array& indexArray = polygons->getP_array();
		_ASSERTE (int (indexArray.getCount()) == polygonCount);

		faceCount += polygonCount;
		for (int k = 0; k < polygonCount; k ++) {
			domPRef pIndices = indexArray[k];
			domListOfUInts& dataIndices = pIndices->getValue();
			int vCount = int (dataIndices.getCount() / stride);
			indexCount += vCount;
		}
	}

	// count triangles
	int triangleFaceStart = faceCount;
	int triangleFaceIndexStart = indexCount;
	domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
	for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
		domTriangles* triangles = trianglesArray[j];
		faceCount += int (triangles->getCount());
		indexCount += int (triangles->getCount()) * 3;
	}


	int* faceIndexCount = new int[faceCount];
	int* materialIndex = new int [faceCount];
	int* vertex = new int [indexCount];
	int* normal = new int [indexCount];
	int* uv0 = new int [indexCount];
	int* uv1 = new int [indexCount];


	domSource* positSrc = NULL;
	domSource* normalSrc = NULL;
	domSource* uv0Src = NULL;
	domSource* uv1Src = NULL;
	LoadTriangles(collada, world, node, imageCache, trianglesArray, 
				  &faceIndexCount[triangleFaceStart], &materialIndex[triangleFaceStart], 
		          &vertex[triangleFaceIndexStart], &normal[triangleFaceIndexStart], 
				  &uv0[triangleFaceIndexStart], &uv1[triangleFaceIndexStart],
				  &positSrc, &normalSrc, &uv0Src, &uv1Src);

	LoadPolygons(collada, world, node, imageCache, polygonArray, 
				 &faceIndexCount[polygonFaceStart], &materialIndex[polygonFaceStart], 
				 &vertex[polygonFaceIndexStart], &normal[polygonFaceIndexStart], 
				 &uv0[polygonFaceIndexStart], &uv1[polygonFaceIndexStart],
			     &positSrc, &normalSrc, &uv0Src, &uv1Src);


	LoadPolygonsList(collada, world, node, imageCache, polygonListArray, 
					&faceIndexCount[polygonListFaceStart ], &materialIndex[polygonFaceStart], 
					&vertex[polygonListFaceIndexStart], &normal[polygonListFaceIndexStart], 
					&uv0[polygonListFaceIndexStart], &uv1[polygonListFaceIndexStart],
					&positSrc, &normalSrc, &uv0Src, &uv1Src);

//	materialCache, imageCache, points, materialTriangle, vertexCount);

	SourceBuffer vertexSource (positSrc);
	SourceBuffer normalSource (normalSrc);
	SourceBuffer uv0Source (uv0Src);
	SourceBuffer uv1Source (uv1Src);
_ASSERTE (0);
/*
	dMatrix matrix (GetIdentityMatrix());
	matrix[0][0] = m_scale;
	matrix[1][1] = m_scale;
	matrix[2][2] = m_scale;
	matrix = matrix * m_globalRotation;
	matrix .TransformTriplex(vertexSource.m_data, sizeof (dVector), vertexSource.m_data, sizeof (dVector), vertexSource.m_count);
	m_globalRotation.TransformTriplex(normalSource.m_data, sizeof (dVector), normalSource.m_data, sizeof (dVector), normalSource.m_count);

	dMeshNodeInfo* geometry = (dMeshNodeInfo*) world->GetInfoFromNode(node);

	geometry->BuildFromVertexListIndexList(faceCount, faceIndexCount, materialIndex, 
					      				   &vertexSource.m_data[0].m_x, sizeof (dVector), vertex,
										   &normalSource.m_data[0].m_x, sizeof (dVector), normal,
										   &uv0Source.m_data[0].m_x, sizeof (dVector), uv0,
										   &uv1Source.m_data[0].m_x, sizeof (dVector), uv1);
	
	delete[] faceIndexCount;
	delete[] materialIndex;
	delete[] vertex;
	delete[] normal;
	delete[] uv0;
	delete[] uv1;
*/
	return node;
}


//dMesh* LoadConvexMesh (domConvex_mesh* colladaMesh, dLoaderContext* context, //domGeometry *colladaGeometry,
//					   ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
dScene::dTreeNode* ColladaImport::LoadConvexMesh (DAE* collada, domConvex_mesh* colladaMesh, dScene::dTreeNode* parent, dScene* world)
{
	dScene::dTreeNode* const node = world->CreateMeshNode(parent);

	// calculate the total triangle count
	int faceCount = 0;
	int indexCount = 0;
	int polygonListFaceStart = faceCount;
	int polygonListFaceIndexStart = indexCount;
	domPolylist_Array &polygonListArray = colladaMesh->getPolylist_array();

	for (int j = 0; j < int (polygonListArray.getCount()); j ++) {
		_ASSERTE (0);
		domPolylist* polygon = polygonListArray[j];
		domPolylist::domVcount* vcount = polygon->getVcount();
		const domListOfUInts& vIndices = vcount->getValue();

		faceCount += int (vIndices.getCount());
		for (int k = 0; k < int (vIndices.getCount()); k ++) {
			int count = int (vIndices[k]);
			//vertexCount += (count - 2) * 3;
			indexCount += count; 
		}
	}


	// count polygons
	int polygonFaceStart = faceCount;
	int polygonFaceIndexStart = indexCount;
	domPolygons_Array &polygonArray = colladaMesh->getPolygons_array();
	for (int j = 0; j < int (polygonArray.getCount()); j ++) {
		domPolygons* polygons = polygonArray[j];
		int polygonCount = int (polygons->getCount());
		int stride = int (polygons->getInput_array().getCount());
		domP_Array& indexArray = polygons->getP_array();
		_ASSERTE (int (indexArray.getCount()) == polygonCount);

		faceCount += polygonCount;
		for (int k = 0; k < polygonCount; k ++) {
			domPRef pIndices = indexArray[k];
			domListOfUInts& dataIndices = pIndices->getValue();
			int vCount = int (dataIndices.getCount() / stride);
			indexCount += vCount;
		}
	}

	// count triangles
	int triangleFaceStart = faceCount;
	int triangleFaceIndexStart = indexCount;
	domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
	for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
		domTriangles* triangles = trianglesArray[j];
		faceCount += int (triangles->getCount());
		indexCount += int (triangles->getCount()) * 3;
	}


	int* const faceIndexCount = new int[faceCount];
	int* const materialIndex = new int [faceCount];
	int* const vertex = new int [indexCount];
	int* const normal = new int [indexCount];
	int* const uv0 = new int [indexCount];
	int* const uv1 = new int [indexCount];

	domSource* positSrc = NULL;
	domSource* normalSrc = NULL;
	domSource* uv0Src = NULL;
	domSource* uv1Src = NULL;

	ImageCache imageCache;
	LoadTriangles(collada, world, node, imageCache, trianglesArray, 
				  &faceIndexCount[triangleFaceStart], &materialIndex[triangleFaceStart], 
				  &vertex[triangleFaceIndexStart], &normal[triangleFaceIndexStart], 
				  &uv0[triangleFaceIndexStart], &uv1[triangleFaceIndexStart],
				  &positSrc, &normalSrc, &uv0Src, &uv1Src);


	LoadPolygons(collada, world, node, imageCache, polygonArray, 
				 &faceIndexCount[polygonFaceStart], &materialIndex[polygonFaceStart], 
				 &vertex[polygonFaceIndexStart], &normal[polygonFaceIndexStart], 
				 &uv0[polygonFaceIndexStart], &uv1[polygonFaceIndexStart],
				 &positSrc, &normalSrc, &uv0Src, &uv1Src);


	LoadPolygonsList(collada, world, node, imageCache, polygonListArray, 
					 &faceIndexCount[polygonListFaceStart ], &materialIndex[polygonFaceStart], 
					 &vertex[polygonListFaceIndexStart], &normal[polygonListFaceIndexStart], 
					 &uv0[polygonListFaceIndexStart], &uv1[polygonListFaceIndexStart],
					 &positSrc, &normalSrc, &uv0Src, &uv1Src);

	//	materialCache, imageCache, points, materialTriangle, vertexCount);

	SourceBuffer vertexSource (positSrc);
	SourceBuffer normalSource (normalSrc);
	SourceBuffer uv0Source (uv0Src);
	SourceBuffer uv1Source (uv1Src);
_ASSERTE (0);
/*
	dMatrix matrix (GetIdentityMatrix());
	matrix[0][0] = m_scale;
	matrix[1][1] = m_scale;
	matrix[2][2] = m_scale;
	matrix = matrix * m_globalRotation;
	matrix .TransformTriplex(vertexSource.m_data, sizeof (dVector), vertexSource.m_data, sizeof (dVector), vertexSource.m_count);
	m_globalRotation.TransformTriplex(normalSource.m_data, sizeof (dVector), normalSource.m_data, sizeof (dVector), normalSource.m_count);

	dMeshNodeInfo* geometry = (dMeshNodeInfo*) world->GetInfoFromNode(node);

	geometry->BuildFromVertexListIndexList(faceCount, faceIndexCount, materialIndex, 
										   &vertexSource.m_data[0].m_x, sizeof (dVector), vertex,
										   &normalSource.m_data[0].m_x, sizeof (dVector), normal,
										   &uv0Source.m_data[0].m_x, sizeof (dVector), uv0,
										   &uv1Source.m_data[0].m_x, sizeof (dVector), uv1);

	delete[] faceIndexCount;
	delete[] materialIndex;
	delete[] vertex;
	delete[] normal;
	delete[] uv0;
	delete[] uv1;
*/
	return node;
}



void ColladaImport::LoadSkinController (DAE* collada, dScene* world, domInstance_controller* colladaSkinController, GeometryChache& meshCache)
{
	daeURI controllerUri (colladaSkinController->getUrl());
	const daeElement* element = controllerUri.getElement();
	domController* controller = (domController*) element;
	_ASSERTE (controller);
	domSkin* colladaSkin = controller->getSkin();

	const daeURI& uriMesh = colladaSkin->getSource();
	element = uriMesh.getElement();
	domGeometry* colladaGeometry = (domGeometry*) element;

	GeometryChache::dTreeNode* node = meshCache.Find(colladaGeometry);
	_ASSERTE (node);
	dScene::dTreeNode* meshNode = node->GetInfo();
	dScene::dTreeNode* skinNode = world->CreateSkinModifierNode(meshNode);

	domSkin::domVertex_weights *vertexWeights = colladaSkin->getVertex_weights();
	domInputLocalOffset* weights = vertexWeights->getInput_array()[0];
	domInputLocalOffset* jointInputs = vertexWeights->getInput_array()[1];
	if (strcmp (jointInputs->getSemantic(), COMMON_PROFILE_INPUT_JOINT)) {
		domInputLocalOffset *tmp = weights;
		weights = jointInputs;
		jointInputs = tmp;
	}
	_ASSERTE (!strcmp (weights->getSemantic(), COMMON_PROFILE_INPUT_WEIGHT));
	_ASSERTE (!strcmp (jointInputs->getSemantic(), COMMON_PROFILE_INPUT_JOINT));


	dScene::dTreeNode* bonesArray[1024];
	daeURI uri (jointInputs->getSource());
	element = uri.getElement ();
	domSource *boneSource = (domSource *)element;
	domName_array *boneNames = boneSource->getName_array();
	domListOfNames &boneSrcArray = boneNames->getValue();
	for (int i = 0; i < int (boneSrcArray.getCount()); i ++) {
		const char* name = boneSrcArray[i];
		dScene::Iterator iter (*world);
		for (iter.Begin(); iter; iter ++) {
			dScene::dTreeNode* boneNode = iter.GetNode();
			dNodeInfo* info = world->GetInfoFromNode(boneNode);
			if (info->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
				if (!strcmp (name, info->GetName())) {
					bonesArray[i] = boneNode;
					world->AddReference(skinNode, boneNode);
					break;
				}
			}
		}
	}


	daeURI uri1 (weights->getSource());
	element = uri1.getElement ();
	domSource *weightSource = (domSource *)element;
	domFloat_array *weightValue = weightSource->getFloat_array();
	domListOfFloats &weightValueArray = weightValue->getValue();

	dMeshNodeInfo* info = (dMeshNodeInfo*) world->GetInfoFromNode (meshNode);
	NewtonMesh* mesh = info->GetMesh();
	int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat);
	int vertexCount = NewtonMeshGetVertexCount(mesh);

_ASSERTE (0);
/*
	const dFloat* const vertex = NewtonMeshGetVertexArray(mesh);

	domSkin::domVertex_weights::domV *v = vertexWeights->getV();
	domSkin::domVertex_weights::domVcount *vCount = vertexWeights->getVcount();

	domListOfInts &vIndices = v->getValue();
	domListOfUInts &vCountIndices = vCount->getValue();
	int jointOffset = int (jointInputs->getOffset());
	int weightOffset = int (weights->getOffset());

	// GetVertex Layers;
	int layersCount = 1;
	for (int i = 0; i < vertexCount; i ++) {
		layersCount = max (layersCount, int (vertex[i * stride + 3]) + 1);
	}
	int vertexBaseCount = vertexCount / layersCount;

	int skinCount = 0;
	dGeometryNodeSkinModifierInfo::dBoneVertexWeightData* skindata = new dGeometryNodeSkinModifierInfo::dBoneVertexWeightData[vertexCount * 4];
	for (int layer = 0; layer < layersCount; layer ++) {
		int weightStartIndex = 0; 
		for (int collVertexIndex = 0; collVertexIndex < (int) vCountIndices.getCount(); collVertexIndex ++) {
			int count = int (vCountIndices[collVertexIndex]);
			for (int j = 0; j < count; j ++) {
				int boneIndex = int (vIndices[weightStartIndex * 2 + jointOffset]);
				int weightIndex = int (vIndices[weightStartIndex * 2 + weightOffset]);
				dFloat weightValue = dFloat (weightValueArray[weightIndex]);
				weightStartIndex ++;

				if (weightValue > 1.0e-3f) {
					skindata[skinCount].m_boneNode = bonesArray[boneIndex];
					skindata[skinCount].m_weight = weightValue;
					skindata[skinCount].m_vertexIndex = collVertexIndex + layer * vertexBaseCount;
					skinCount ++;
				}
			}
		}
	}

	dGeometryNodeSkinModifierInfo* skinInfo = (dGeometryNodeSkinModifierInfo*) world->GetInfoFromNode(skinNode);
	skinInfo->SkinMesh(skinNode, world, skindata, skinCount);
	delete[] skindata;
*/
}


void ColladaImport::LoadVisualScene (DAE* collada, domVisual_scene* visualScene, dScene* world, GeometryChache& meshCache, ImageCache& imageCache, NodeChache& nodeCache)
{
	domNode* nodePool[1024];
	dScene::dTreeNode* parentBones[1024];
	dTree<dScene::dTreeNode*, domNode*> controllersIntances; 

	// create a dummy root bode;
	dScene::dTreeNode* rootBone = world->GetRootNode();

	int stack = 0;
	const domNode_Array& nodeArray = visualScene->getNode_array();
	for (int i = 0; i < int (nodeArray.getCount()); i ++) {
		parentBones[stack] = rootBone;
		nodePool[stack] = nodeArray[i];
		stack ++;
	}


	while (stack) {
		stack --;

		domNode* colNode = nodePool[stack];
		dScene::dTreeNode* parent = parentBones[stack]; 
		
		dScene::dTreeNode* node = world->CreateSceneNode (parent);
		if (colNode->getInstance_controller_array().getCount()) {
			// save this node for later modifirs binding
			controllersIntances.Insert(node, colNode);
		}

		// save this node for later physics binding
		nodeCache.Insert (node, colNode);

		LoadNodeData (collada, node, colNode, world, meshCache, imageCache);

		if (parent != rootBone) {
			dSceneNodeInfo* parenInfo = (dSceneNodeInfo*) world->GetInfoFromNode(parent);
			dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) world->GetInfoFromNode(node);
			dMatrix matrix (sceneInfo->GetTransform() * parenInfo->GetTransform());
			sceneInfo->SetTransform(matrix);
		}


		// check if this collada file has Node instances, a feature support by some modelers but not all  
		domInstance_node_Array& instansceNodeArray = colNode->getInstance_node_array();
		for (int i = 0; i < int (instansceNodeArray.getCount()); i ++) {

			// if there are node instances we will just read them as real nodes copies.
			domInstance_node* instansceNode = instansceNodeArray[0];
			daeURI uri (instansceNode->getUrl());
			daeElement* element = uri.getElement ();
			domNode* childNode = (domNode *)element;

			parentBones[stack] = node;
			nodePool[stack] = childNode;
			stack ++;
		}


		const domNode_Array &nodeArray = colNode->getNode_array();
		for (int i = 0; i < int (nodeArray.getCount()); i ++) {
			parentBones[stack] = node;
			nodePool[stack] = nodeArray[i];
			stack ++;
		}
	}

	// now load all meshes controllers
	while (controllersIntances.GetCount()) {
		domNode* colladaNode = controllersIntances.GetRoot()->GetKey();
		controllersIntances.Remove(controllersIntances.GetRoot());

		domInstance_controller* instanceController = colladaNode->getInstance_controller_array()[0];
		LoadSkinController (collada, world, instanceController, meshCache);
	}
}


domMaterial* ColladaImport::FindMaterial (domLibrary_materials* library, const char* name) const
{
	if (name && library) {
		const domMaterial_Array& array = library->getMaterial_array();
		for (unsigned i = 0; i < array.getCount(); i ++) {
			domMaterial* material = array[i];
			if (!cdom::strcasecmp (name, material->getId())) {
				return material;
			}
		}
	}
	return NULL;
}

// check image duplicates in the library
domImage* ColladaImport::FindImage (domLibrary_images* library, const char*name) const
{
	if (name && library) {
		const domImage_Array& array = library->getImage_array();
		for (unsigned i = 0; i < array.getCount(); i ++) {
			domImage* image = array[i];
			if (!strcmp (name, image->getId())) {
				return image;
			}
		}
	}
	return NULL;
}


int ColladaImport::CreateMaterial (DAE* collada, const char* matName, ImageCache& imageCache, dScene* world, dScene::dTreeNode* gemetryNode)
{
_ASSERTE (0);
return 0;
#if 0
	daeDatabase* database = collada->getDatabase();
	_ASSERTE (database);

	daeDocument *document = database->getDocument(daeUInt (0));
	_ASSERTE (document);

	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

	domLibrary_materials* library = NULL;
	if (domRoot->getLibrary_materials_array().getCount()) {
		library = domRoot->getLibrary_materials_array()[0];
	}


//	CollMaterial* matId = NULL;
	int matID = -1;
	domMaterial* material = FindMaterial (library, matName);
	if (material) {
		domInstance_effect* instanceEffect;
		instanceEffect = material->getInstance_effect();
		_ASSERTE (instanceEffect);

		if (instanceEffect) {
			daeURI uri (instanceEffect->getUrl());
			daeElement* element = uri.getElement();
			_ASSERTE (element);

			domEffect* effect = (domEffect*) element;
			if (effect) {
				matID = 0;
				for (void* ptr = world->GetFirstChild(gemetryNode); ptr; ptr = world->GetNextChild(gemetryNode, ptr)) {
					matID ++;
				}
				dScene::dTreeNode* matNode = world->CreateMaterialNode(gemetryNode, matID);
				dMaterialNodeInfo* material = (dMaterialNodeInfo*) world->GetInfoFromNode(matNode);
			
				domLibrary_images* imageLibrary = NULL;
				if (domRoot->getLibrary_images_array().getCount()) {
					imageLibrary = domRoot->getLibrary_images_array()[0];
					_ASSERTE (imageLibrary);
				}

				const domFx_profile_abstract_Array &profileArray = effect->getFx_profile_abstract_array();
				domProfile_COMMON *profile = daeSafeCast<domProfile_COMMON> (profileArray[0]);

				if (profile) {
					// and the diffuse color from the type form the material type
					domProfile_COMMON::domTechnique* technique = profile->getTechnique();
					_ASSERTE (technique);

					domCommon_color_or_texture_type* diffuse = NULL;

					if (technique->getLambert()) {
						domProfile_COMMON::domTechnique::domLambert* parameters;
						parameters = technique->getLambert();
						_ASSERTE (parameters);
						diffuse = parameters->getDiffuse();
					} else if (technique->getPhong()) {
						domProfile_COMMON::domTechnique::domPhong* parameters;
						parameters = technique->getPhong();
						_ASSERTE (parameters);
						diffuse = parameters->getDiffuse();
					} else if (technique->getBlinn()) {
						domProfile_COMMON::domTechnique::domBlinn* parameters;
						parameters = technique->getBlinn();
						_ASSERTE (parameters);
						diffuse = parameters->getDiffuse();
					} else {
						_ASSERTE (0);
					}

					// since this is a prove of concept we only support the diffuse color texture feature
					// more complex material are unsupported at these time
					if (diffuse) {
						domCommon_color_or_texture_type::domTexture* texture = diffuse->getTexture();
						if (texture) {
							const char* textName = texture->getTexture();
/*
							const char* textName = NULL;
							const char* textSample = texture->getTexture();
							domCommon_newparam_type_Array& paramaArray = profile->getNewparam_array();

							// find the texture sample 
							for (int j = 0; !textName && (j < int (paramaArray.getCount())); j ++) {

								const char* name = paramaArray[j]->getSid();
								if (!cdom::strcasecmp (name, textSample)) {
									domCommon_newparam_type_complexType* type = paramaArray[j];
									domFx_sampler2D_common* sampler2d = type->getSampler2D();
									domFx_sampler2D_common_complexType::domSource* source = sampler2d->getSource();
									const char* textSample2d = source->getValue();

									// find the image name for this diffuse effect
									for (int k = 0; !textName && (k < int (paramaArray.getCount())); k ++) {
										const char* name = paramaArray[k]->getSid(); 
										if (!cdom::strcasecmp (name, textSample2d)) {
											type = paramaArray[k];
											domFx_surface_common* surface = type->getSurface();
											//domFx_surface_common::domInit_from_Array& initFromArray = surface->getInit_from_array();

											domFx_surface_init_commonRef surfInit = surface->getFx_surface_init_common();
											domFx_surface_init_from_common_Array initFromArray = surfInit->getInit_from_array();

											// finally get the name of the texture and bail out from form all loops
											for (int m = 0; !textName && (m < int (paramaArray.getCount())); m ++) {
												const xsIDREF& id = initFromArray[m]->getValue();
												textName = id.getID();
											}
										}
									}
								}
							}
*/

							// if we have a texture name save this material as one of the material used by the geometries
							if (textName) {
								domImage* image = FindImage (imageLibrary, textName);
								_ASSERTE (image);

								domImage::domInit_from* initfrom = image->getInit_from();
								_ASSERTE (initfrom);

								if (initfrom->getValue().getOriginalURI()) {
									//CollMaterial collMaterial;
									xsAnyURI uri (initfrom->getValue());
									textName = uri.getURI();

									const char* ptr = strrchr (textName, '\\');
									if (ptr) {
										ptr ++;
									} else {
										ptr = strrchr (textName, '/');
										if (ptr) {
											ptr ++;
										} else {
											ptr = textName;
										}
									}

									int crc = dCRC(ptr);
									#ifdef _DEBUG
										for (void* ptr = world->GetFirstChild(matNode); ptr; ptr = world->GetNextChild(matNode, ptr)) {
											dScene::dTreeNode* textNode = world->GetNodeFromLink(ptr);
											dTextureNodeInfo* texture = (dTextureNodeInfo*) world->GetInfoFromNode(textNode);
											if (texture->GetTypeId() == dTextureNodeInfo::GetRttiType()) {
												_ASSERTE (texture->GetId() != crc);
											}
										}
									#endif
									dTree<dScene::dTreeNode*, int>::dTreeNode* cacheNode = imageCache.Find(crc);
									if (!cacheNode) {
										dScene::dTreeNode* textNode = world->CreateTextureNode(ptr);
										cacheNode = imageCache.Insert(textNode, crc);
									}
									dScene::dTreeNode* textNode = cacheNode->GetInfo();
									world->AddReference(matNode, textNode);
									dTextureNodeInfo* texture = (dTextureNodeInfo*) world->GetInfoFromNode(textNode);
									material->SetDiffuseTextId(texture->GetId());
								}
							}
						} else {
							// no technique for this segment only color
							//_ASSERTE (0);
						}
					}
				}
			}
		}
	}

	if (matID == -1) {
		_ASSERTE (0);
	}

	return matID;
#endif
}


domTechnique *ColladaImport::FindProfileExtra (domExtra_Array& extraArray, const char* keyword) const
{
	// if the geometry have extras, the search the Veter format option 
	for (int j = 0; j < int (extraArray.getCount()); j ++) {
		domExtra* extra = extraArray[j];
		const domTechnique_Array& techniqueArray = extra->getTechnique_array();
		for (int k = 0; k < int (techniqueArray.getCount()); k ++) {
			if (!cdom::strcasecmp (techniqueArray[k]->getProfile(), keyword)) {
				return techniqueArray[k];
			}
		}
	}

	return NULL;
}


dScene::dTreeNode* ColladaImport::LoadCollision (
	DAE* collada, 
	const domRigid_body::domTechnique_common::domShape* shape, 
	dScene* world, 
	dScene::dTreeNode* parent,
	GeometryChache& meshCache)
{
	dMatrix alignMatrix (GetIdentityMatrix());
	dScene::dTreeNode* node = NULL;
	if (shape->getBox()) {
		domBox* box = shape->getBox(); 
		domBox::domHalf_extents* halfExtend = box->getHalf_extents(); 
		dVector size (dFloat(halfExtend->getValue()[0]), dFloat(halfExtend->getValue()[1]), dFloat(halfExtend->getValue()[2]), 0.0f);
		size = size.Scale (m_scale * 2.0f);
		node = world->CreateCollisionBoxNode (parent);
		dCollisionBoxNodeInfo* const collInfo = (dCollisionBoxNodeInfo*) world->GetInfoFromNode(node);
		collInfo->SetSize(size);

	} else if (shape->getCapsule()) {
		domCapsule* capsule = shape->getCapsule(); 
		dFloat radius = dFloat (capsule->getRadius()->getValue()[0]); 
		dFloat height = (dFloat (capsule->getHeight()->getValue()) + radius) * 2.0f;

		radius *= m_scale;
		height *= m_scale;

		node = world->CreateCollisionCapsuleNode (parent);
		alignMatrix = dRollMatrix(0.5f * 3.14159265f);
		dCollisionCapsuleNodeInfo* const collInfo = (dCollisionCapsuleNodeInfo*) world->GetInfoFromNode(node);
		collInfo->SetRadius(radius);
		collInfo->SetHeight(height);

	} else if (shape->getCylinder()) {

		domCylinder* cylinder = shape->getCylinder();
		dFloat height = dFloat (cylinder->getHeight()->getValue()) * m_scale * 2.0f; 
		dFloat radius = dFloat (cylinder->getRadius()->getValue()[0]) * m_scale;

		alignMatrix = dRollMatrix(0.5f * 3.14159265f);
		domTechnique* technique = FindProfileExtra (cylinder->getExtra_array(), "chamferCylinder");
		if (technique) {
//			collision = NewtonCreateChamferCylinder (world, radius, height, shapeID, &matrix[0][0]);
			node = world->CreateCollisionChamferCylinderNode(parent);
			dCollisionChamferCylinderNodeInfo* const collInfo = (dCollisionChamferCylinderNodeInfo*) world->GetInfoFromNode(node);
			collInfo->SetRadius(radius);
			collInfo->SetHeight(height);
		} else {
//			collision = NewtonCreateCylinder (world, radius, height, shapeID, &matrix[0][0]);
			node = world->CreateCollisionCylinderNode (parent);
			dCollisionCylinderNodeInfo* const collInfo = (dCollisionCylinderNodeInfo*) world->GetInfoFromNode(node);
			collInfo->SetRadius(radius);
			collInfo->SetHeight(height);
		}

	} else if (shape->getTapered_cylinder()) {

		domTapered_cylinder* cone = shape->getTapered_cylinder();
		dFloat height = dFloat (cone->getHeight()->getValue())  * m_scale * 2.0f;
		dFloat radius1 = dFloat (cone->getRadius1()->getValue()[0]) * m_scale;
		dFloat radius2 = dFloat (cone->getRadius2()->getValue()[0]) * m_scale;

		alignMatrix = dRollMatrix(0.5f * 3.14159265f);
		node = world->CreateCollisionConeNode (parent);
		dCollisionConeNodeInfo* const collInfo = (dCollisionConeNodeInfo*) world->GetInfoFromNode(node);
		collInfo->SetHeight(height);

		if (radius1 >= radius2) {
//			collision = NewtonCreateCone (world, radius1, height, shapeID, &matrix[0][0]);
			collInfo->SetRadius(radius1);
		} else {
			alignMatrix = dPitchMatrix (3.141592f);
//			collision = NewtonCreateCone (world, radius2, height, shapeID, &matrix[0][0]);
			collInfo->SetRadius(radius2);
		}

/*
	} else if (shape->getSphere()) {
		domSphere* sphere = shape->getSphere(); 

		dFloat radius0 = dFloat (sphere->getRadius()->getValue());
		dFloat radius1 = radius0;
		dFloat radius2 = radius0;
		
		domTechnique* technique = FindProfileExtra (sphere->getExtra_array(), "ellipse");
		if (technique) {
			const daeElementRefArray& array = technique->getContents();
			daeElement* element = array[0];
			domAny *domExtraExtension = (domAny*)element; 
			_ASSERTE (!strcmp (domExtraExtension->getElementName(), "radios"));

			const char* radios = domExtraExtension->getValue();
			sscanf (radios, "%f %f %f", &radius0, &radius1, &radius2);
		}
	
		collision = NewtonCreateSphere (world, radius0 * m_scale, radius1 * m_scale, radius2 * m_scale, shapeID, &matrix[0][0]);
*/
	} else if (shape->getInstance_geometry()) {

		domInstance_geometry* instance = shape->getInstance_geometry(); 
		daeURI uri (instance->getUrl());
		const daeElement* element = uri.getElement();
		domGeometry *collGeometry = (domGeometry*) element;
		if (collGeometry->getConvex_mesh()) {
			domConvex_mesh* colladaMesh = collGeometry->getConvex_mesh();
			domVertices* vertices = colladaMesh->getVertices();
			domInputLocal_Array &inputArray = vertices->getInput_array();
			domSource *source = NULL;
			for (int i = 0; i < int (inputArray.getCount()); i ++) {
				domInputLocal* offset = inputArray[i];
				domInputLocal* input = inputArray[i];
				if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_POSITION)) {
					daeURI uri1 (input->getSource());
					element = uri1.getElement();
					_ASSERTE (element);
					source = (domSource *) element;
					break;
				}
			}
			_ASSERTE (source);

			SourceBuffer vertexSource (source);
_ASSERTE (0);
/*
			dMatrix matrix (GetIdentityMatrix());
			matrix[0][0] = m_scale;
			matrix[1][1] = m_scale;
			matrix[2][2] = m_scale;
			matrix = matrix * m_globalRotation;
			matrix.TransformTriplex(vertexSource.m_data, sizeof (dVector), vertexSource.m_data, sizeof (dVector), vertexSource.m_count);
			const NewtonMesh* const mesh = NewtonMeshConvexHull (world->GetNewtonWorld(), vertexSource.m_count, &vertexSource.m_data[0].m_x, sizeof (dVector), 0.0f);

			int vCount = NewtonMeshGetVertexCount(mesh);
			int stride = NewtonMeshGetVertexStrideInByte(mesh);
			const dFloat* const points = NewtonMeshGetVertexArray(mesh);

			node = world->CreateCollisionConvexHullNode (parent);
			dCollisionConvexHullNodeInfo* const collInfo = (dCollisionConvexHullNodeInfo*) world->GetInfoFromNode(node);
			collInfo->SetFaceSelection (vCount, points, stride);
			NewtonMeshDestroy(mesh);
*/
		} else {
			_ASSERTE (0);
/*
			CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
			if (!cacheNode) {
				dLoaderContext context;
				dMesh* mesh = LoadMesh (collGeometry->getMesh(), &context, modifierVertexMapCache, materialCache, imageCache);

				mesh->ApplyGlobalScale(m_scale);
				mesh->ApplyGlobalTransform (m_globalRotation);
				cacheNode = meshCache.Insert(mesh, collGeometry);
			}
			dMesh* mesh = cacheNode->GetInfo();

			collision = NewtonCreateTreeCollision (world, NULL);
			NewtonTreeCollisionBeginBuild (collision);

			dFloat* const vertex = mesh->m_vertex;
			for (dMesh::dListNode* node = mesh->GetFirst(); node; node = node->GetNext()) {
				const dSubMesh& segment = node->GetInfo();
				for (int j = 0; j < segment.m_indexCount; j += 3) {
					dVector face[3];

					int index = segment.m_indexes[j + 0] * 3;
					face[0] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

					index = segment.m_indexes[j + 1] * 3;
					face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

					index = segment.m_indexes[j + 2] * 3;
					face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

					NewtonTreeCollisionAddFace (collision, 3, &face[0].m_x, sizeof (dVector), 0);
				}
			}

			NewtonTreeCollisionEndBuild (collision, 1);	
*/
		}


/*
	} else if (shape->getPlane()) {
		_ASSERTE (0);
//		domPlane* plane;
//		domPlane::domEquation* equation;
//		plane = shape->getPlane(); 
//		equation = plane->getEquation(); 
//		dVector surface (equation->getValue()[0], equation->getValue()[1], equation->getValue()[2], equation->getValue()[2]);
//		surface = m_axisRotation.TransformPlane(surface);
//		surface = matrix.TransformPlane (surface);
//		collision = CreatePlaneCollidion (m_world, surface );
*/
	} else {
		_ASSERTE (0);
	}


	if (node) {
		dCollisionNodeInfo* collision = (dCollisionNodeInfo*) world->GetInfoFromNode(node);

		const domRotate_Array &rotationArray = shape->getRotate_array();
		const domTranslate_Array &translationArray = shape->getTranslate_array();
		dMatrix matrix (GetIdentityMatrix());
		for (int i = 0; i < int (translationArray.getCount()); i ++) {
			const domFloat3& data = translationArray[i]->getValue();
			matrix[3][0] += dFloat (data[0]);
			matrix[3][1] += dFloat (data[1]);
			matrix[3][2] += dFloat (data[2]);
		}

		for (int i = 0; i < int (rotationArray.getCount()); i ++) {
			const domFloat4& data = rotationArray[i]->getValue();
			dFloat angle = dFloat (data[3]) * 3.1415926f / 180.0f;
			dFloat x = dFloat (data[0]);
			dFloat y = dFloat (data[1]);
			dFloat z = dFloat (data[2]);
			dVector axis (x, y, z, 0.0f);
			axis = axis.Scale (1.0f / sqrtf (axis % axis));
			dQuaternion rot (axis, angle);
			dMatrix colladaMatrix (rot, dVector (0.0f, 0.0f, 0.0f, 1.0f));
			matrix = colladaMatrix * matrix;
		}
		matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
		matrix.m_posit = matrix.m_posit.Scale(m_scale);
		collision->SetTransform(alignMatrix * matrix);
	}

	return node;
}




//NewtonCollision* CreateCollision(NewtonWorld* world, const domRigid_body::domTechnique_common::domShape_Array& shapeArray, int isDynamics,
//								 CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)

//NewtonCollision* CreateCollision(NewtonWorld* world, const domRigid_body::domTechnique_common::domShape_Array& shapeArray, int isDynamics,
//								 CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
dScene::dTreeNode* ColladaImport::LoadCollision (
	DAE* collada,
	const domRigid_body::domTechnique_common::domShape_Array& shapeArray, 
	dScene* world, 
	dScene::dTreeNode* rigidBody,
	GeometryChache& meshCache,
	int isDynamicsBody)
{
//	NewtonCollision* collision = NULL;
	dScene::dTreeNode* node = NULL;
	int count = int (shapeArray.getCount());
	if (count == 1)	{
		domRigid_body::domTechnique_common::domShape* shape = shapeArray[0];
		//collision = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
		node = LoadCollision (collada, shape, world, rigidBody, meshCache);
	} else {
		if (isDynamicsBody) {
			node = world->CreateCollisionCompoundNode (rigidBody);
//			dCollisionCompoundNodeInfo* const compound = (dCollisionCompoundNodeInfo*) world->GetInfoFromNode(node);

//			NewtonCollision** collsionArray = new NewtonCollision*[count];
//			_ASSERTE (count < sizeof (collsionArray) / sizeof (collsionArray[0]));
			for (int i = 0; i < count; i ++) {
				domRigid_body::domTechnique_common::domShape* shape = shapeArray[i];
//				collsionArray[i] = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
				LoadCollision (collada, shape, world, node, meshCache);
			}
//			collision = NewtonCreateCompoundCollision (world, count, collsionArray, 0);
//			for (int i = 0; i < count; i ++) {
//				NewtonReleaseCollision (world, collsionArray[i]);
//			}
//			delete[] collsionArray;

		} else {
			_ASSERTE (0);
//			domRigid_body::domTechnique_common::domShape* shape = shapeArray[0];
//			collision = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
		}
	}

	return node;
}



//NewtonBody* LoadRigidBody (domRigid_body* rigidBody, dModel* model, dLoaderContext* context,
//						   CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, 
//						   CollMaterialCache& materialCache, CollImageCache& imageCache)
dScene::dTreeNode* ColladaImport::LoadRigidBody (DAE* collada, domRigid_body* rigidBody, dScene* world, dScene::dTreeNode* model, GeometryChache& meshCache)
{
//		int i;
//		int j;
//		int k;
//		int isDynamicsBody;
//		dFloat Ixx;
//		dFloat Iyy;
//		dFloat Izz;
//		dFloat mass;
//		dVector com;
//		domNode* target;
//		daeElement* element;
//		NewtonBody* body;
//		dSceneNode* viualNodePtr;
//		domRigid_body* rigidBody;
//		domTargetableFloat* bodyMass;
//		domTargetableFloat3* bodyInertia;
//		dNodePhysicsRigidBody::dTreeNode* rigidBodyNode;
//		ColladaNodeTodSceneNodeMap::dTreeNode* visualNode;
//		domRigid_body::domTechnique_common::domDynamic* dynamicBody;
//		domRigid_body::domTechnique_common::domMass_frame* massFrame;
//		domInstance_rigid_body* instanceRigidBody

//		element = instanceRigidBody->getTarget().getElement();
//		target = (domNode*) element;
//		visualNode = m_nodeTodNodeMap.Find(target);
//		rigidBodyNode = m_rigidBodyCache.Find (dCRC (instanceRigidBody->getBody()));
//		_ASSERTE (rigidBodyNode);
//		rigidBody = rigidBodyNode->GetInfo();

	domRigid_body::domTechnique_common* techniqueCommon = rigidBody->getTechnique_common();
	_ASSERTE (techniqueCommon);

	int isDynamicsBody = 1;
	if (techniqueCommon->getMass()) {
		if (techniqueCommon->getMass()->getValue() == 0.0f) {
			isDynamicsBody = 0;
		}
	}
	if (isDynamicsBody && techniqueCommon->getDynamic()) {
		if (techniqueCommon->getDynamic()->getValue() == false) {
			isDynamicsBody = 0;
		}
	}

//	NewtonCollision* collision = CreateCollision(world, techniqueCommon->getShape_array(), isDynamicsBody, meshCache, modifierVertexMapCache, materialCache, imageCache);
//	NewtonBody* body = NewtonCreateBody (world, collision);
//	NewtonReleaseCollision (world, collision);
	dScene::dTreeNode* rigidBodyNode = world->CreateRigidbodyNode(model);
	LoadCollision (collada, techniqueCommon->getShape_array(), world, rigidBodyNode, meshCache, isDynamicsBody);

	dRigidbodyNodeInfo* rigidBodyInfo = (dRigidbodyNodeInfo*) world->GetInfoFromNode(rigidBodyNode);

	dFloat Ixx = 0.0f;
	dFloat Iyy = 0.0f;
	dFloat Izz = 0.0f;
	dFloat mass = 0.0f;
	dVector com (0.0f, 0.0f, 0.0f, 0.0f);
	domTargetableFloat* bodyMass = techniqueCommon->getMass();
	if (bodyMass) {
		domRigid_body::domTechnique_common::domDynamic* dynamicBody = techniqueCommon->getDynamic();
		if (dynamicBody) {
			if (dynamicBody->getValue() == true) {
				mass = dFloat (bodyMass->getValue());
			}
		}
		if (mass > 0.0f) {
			domRigid_body::domTechnique_common::domMass_frame* massFrame = techniqueCommon->getMass_frame();
			domTargetableFloat3* bodyInertia = techniqueCommon->getInertia();
			if (massFrame && bodyInertia) {
				Ixx = dFloat (bodyInertia->getValue()[0]);
				Iyy = dFloat (bodyInertia->getValue()[1]);
				Izz = dFloat (bodyInertia->getValue()[2]);
				if (massFrame->getTranslate_array().getCount()) {
					domTranslate* translation = massFrame->getTranslate_array()[0];
					com.m_x = dFloat (translation->getValue()[0]);
					com.m_y = dFloat (translation->getValue()[1]);
					com.m_z = dFloat (translation->getValue()[2]);
					com = m_globalRotation.RotateVector(com.Scale (m_scale));
				}
			} else {
				_ASSERTE (0);
//				dVector inertia;
//				NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &com[0]);	
//				Ixx = inertia.m_x * mass;
//				Iyy = inertia.m_y * mass;
//				Izz = inertia.m_z * mass;
			}
		}
	}

//	m_rigidBodyMap.Insert(body, rigidBody);

	// set user data;
//	NewtonBodySetUserData (body, visualNode ? visualNode->GetInfo() : NULL);
//	NewtonBodySetTransformCallback (body, DefaultColladaPhysicsSetTransform);
//	NewtonBodySetForceAndTorqueCallback (body, DefaultColladaApplyGravityForce);

	// by default disable collision with jointed objects
//	NewtonBodySetJointRecursiveCollision (body, 0);

	// set a destructor for this rigid body
//	NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);


	// all body have default Material ID		
//	NewtonBodySetMaterialGroupID (body, colladaMaterialIndex);


//	NewtonBodySetCentreOfMass (body, &com[0]);
//	if (mass > 0.0f) {
//		NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);
//	}

	rigidBodyInfo->SetCenterOfMass(com);
	rigidBodyInfo->SetMassMatrix(dVector (Ixx, Iyy, Izz, mass));


//		dMatrix matrix (GetIdentityMatrix());
//		if (visualNode) {
//			for (viualNodePtr = visualNode->GetInfo(); viualNodePtr; viualNodePtr = viualNodePtr->GetParent()) {
//				matrix = matrix * viualNodePtr->GetMatrix();
//			}
//			NewtonBodySetMatrix (body, &matrix[0][0]);
//		} else {
//			matrix.m_posit.m_y = - 1000.0f;
//		}


// add a moronic callada style material;
//		if (visualNode) {
//			ColladaFrictionRestituionMaterial* frictionRestituionMaterial;
//			frictionRestituionMaterial = new ColladaFrictionRestituionMaterial;
//			visualNode->GetInfo()->SetUserData (frictionRestituionMaterial);

// material for this body
//			domPhysics_material* physicMaterial;
//			domInstance_physics_material* intanceMaterial;
//			intanceMaterial = techniqueCommon->getInstance_physics_material();
//			if (intanceMaterial) {
//				const daeElement* element;
//				domPhysics_material::domTechnique_common* technique;
//				const daeURI& uri = intanceMaterial->getUrl();
//				element = uri.getElement();
//				if (element) {
//					physicMaterial = (domPhysics_material*) element;
//					technique = physicMaterial->getTechnique_common();
//					frictionRestituionMaterial->m_staticFriction = technique->getStatic_friction()->getValue();
//					frictionRestituionMaterial->m_restitution = technique->getRestitution()->getValue();
//					frictionRestituionMaterial->m_dynamicFriction = technique->getDynamic_friction()->getValue();
//				}
//			}
//		}

/*
	domTechnique *technique = FindProfileExtra (rigidBody->getExtra_array(), "Newton");
	if (technique) {
		// it is a newton collada set specific parameters 
		const daeElementRefArray& array = technique->getContents();
		for (int i = 0; i < int (array.getCount()); i ++) {
			daeElement* element = array[i];
			domAny *domExtraExtension = (domAny*)element; 

			if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingUserData")) {
				context->SetUserData(body, model, domExtraExtension->getValue());

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingSetTransformFunction")) {
				context->SetTransformCallback(body, domExtraExtension->getValue());

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingExternalForceFunction")) {
				context->SetForceAndTorqueCallback(body, domExtraExtension->getValue());

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingBodyDestructorCallback")) {
				context->SetDestructorCallback(body, domExtraExtension->getValue());

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "AutoSleepMode")) {
				if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
					NewtonBodySetAutoSleep (body, 1);
				} else {
					NewtonBodySetAutoSleep (body, 0);
				}

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalLinearDrag")) {
				dFloat drag;
				sscanf (domExtraExtension->getValue(), "%f", &drag);
				NewtonBodySetLinearDamping(body, drag);

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalAngularDrag")) {
				dFloat drag[4];
				sscanf (domExtraExtension->getValue(), "%f", &drag[0]);
				drag[1] = drag[0];
				drag[2] = drag[0];
				NewtonBodySetAngularDamping (body, &drag[0]);

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalContinueCollision")) {
				if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
					NewtonBodySetContinuousCollisionMode(body, 1);
				} else {
					NewtonBodySetContinuousCollisionMode (body, 0);
				}

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "RecursivelyCollideWithLinkedBodies")) {
				if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
					NewtonBodySetJointRecursiveCollision(body, 1);
				} else {
					NewtonBodySetJointRecursiveCollision (body, 0);
				}

			} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "UnilateralMaterialID")) {
				int materiaID;
				sscanf (domExtraExtension->getValue(), "%d", &materiaID);

				// for now since collada do not support realistic manifold materials 
				materiaID = 0;
				NewtonBodySetMaterialGroupID (body, materiaID);
			}
		}
	} else {
		// it is not a Newton Collada set the default parameters
		_ASSERTE (0);
		context->SetUserData(body, model, "default");
		context->SetTransformCallback(body, "default");
		context->SetForceAndTorqueCallback(body, "default");
		context->SetDestructorCallback(body, "default");
		NewtonBodySetAutoSleep (body, 1);

		//NewtonBodySetLinearDamping(body, drag);
		//NewtonBodySetAngularDamping (body, &drag[0]);
		//NewtonBodySetContinuousCollisionMode(body, 1);
		//NewtonBodySetContinuousCollisionMode (body, 0);
		//NewtonBodySetJointRecursiveCollision(body, 1);
		//NewtonBodySetJointRecursiveCollision (body, 0);
		//NewtonBodySetMaterialGroupID (body, materiaID);
	}
*/
	return rigidBodyNode;
}







//void LoadPhysicScene(daeDocument* document, NewtonWorld* world, dSceneModelList& sceneList, dLoaderContext* context, CollGeoCache& meshCache,
//					 ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
void ColladaImport::LoadPhysicScene (DAE* collada, dScene* world, const NodeChache& nodeCache, GeometryChache& meshCache)
{
	daeDatabase* database = collada->getDatabase();
	_ASSERTE (database);

	daeDocument *document = database->getDocument(daeUInt (0));
	_ASSERTE (document);

	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	_ASSERTE (domRoot);

//	dTree<dModel*, domNode*> modelCache; 
//	const domLibrary_visual_scenes_Array &libraryArrays = domRoot->getLibrary_visual_scenes_array ();
//	for (int j = 0; j < int (libraryArrays.getCount()); j ++) {
//		const domVisual_scene_Array& libraryArray = libraryArrays[j]->getVisual_scene_array();
//		for (int i = 0; i < int (libraryArray.getCount()); i ++) {
//			dModel* model = context->CreateModel();
//			domVisual_scene* visualScene = libraryArray[i];
//			_ASSERTE (visualScene);
//			domNode* node = LoadScene (document, model, visualScene, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
//			sceneList.AddModel(model);
//			model->Release();
//			modelCache.Insert(model, node);
//		}
//	}


	domCOLLADA::domScene *scene = domRoot->getScene();
	_ASSERTE (scene);

	// find the visual scene instance that is store in the scene node; 
	domInstanceWithExtra_Array& physicIntanceArray = scene->getInstance_physics_scene_array();
	for (int i = 0; i < int (physicIntanceArray.getCount()); i ++) {
		domInstanceWithExtra *physicIntanceScene = physicIntanceArray[i];

		// if there is a instance to a Scene then Get the Scene from the uri
		daeURI uri (physicIntanceScene->getUrl());

		daeElement* element = uri.getElement();
		_ASSERTE (element);

		domPhysics_scene* physicScene = (domPhysics_scene*) element;
		domInstance_physics_model_Array& instancePhysicsModelArray = physicScene->getInstance_physics_model_array();

		for (int j = 0; j < int (instancePhysicsModelArray.getCount()); j ++) {
			domInstance_physics_model* instancePhysicsModel = instancePhysicsModelArray[j];

			daeURI bodyUri (instancePhysicsModel->getUrl());
			element = bodyUri.getElement();
			_ASSERTE (element);
			domPhysics_model* physicModel = (domPhysics_model*) element;
			domRigid_body_Array& bodyArray = physicModel->getRigid_body_array();

			domInstance_rigid_body_Array& instanceRigidBodyArray = instancePhysicsModel->getInstance_rigid_body_array();
			for (int k = 0; k < int (instanceRigidBodyArray.getCount()); k ++) {
				domInstance_rigid_body* instanceRigidBody = instanceRigidBodyArray[k];

				domRigid_body* rigidBody = NULL;
				for (int m = 0; m < int (bodyArray.getCount()); m ++) {
					if (!strcmp (instanceRigidBody->getBody(), bodyArray[m]->getSid())) {
						rigidBody = bodyArray[m];
						break;
					}
				}

				daeURI nodeUri (instanceRigidBody->getTarget());
				element = nodeUri.getElement();
				//dModel *model = NULL;
				dScene::dTreeNode* sceneNode = NULL;
				dMatrix matrix (GetIdentityMatrix());
				if (element) {
					domNode* node = (domNode*) element;

					matrix = GetMatrix (node);
					sceneNode = nodeCache.Find(node)->GetInfo();
				}
				_ASSERTE (sceneNode);


				//NewtonBody* body = LoadRigidBody (world, rigidBody, model, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
				dScene::dTreeNode* rigidBodyNode = LoadRigidBody (collada, rigidBody, world, sceneNode, meshCache);
				dRigidbodyNodeInfo* rigidBodyInfo = (dRigidbodyNodeInfo*) world->GetInfoFromNode(rigidBodyNode);

				matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
				matrix.m_posit = matrix.m_posit.Scale (m_scale);

				//NewtonBodySetMatrix(body, &matrix[0][0]);
				//rigidBodyInfo->SetTransform(matrix);
				rigidBodyInfo->SetName(instanceRigidBody->getBody());
				
				domInstance_rigid_body::domTechnique_common* technique = instanceRigidBody->getTechnique_common();
				domInstance_rigid_body::domTechnique_common::domVelocity* initVeloc = technique->getVelocity();
				domInstance_rigid_body::domTechnique_common::domAngular_velocity* initOmega = technique->getAngular_velocity();

				dVector omega;
				dVector veloc;

				omega[0] = dFloat(initOmega->getValue()[0]);
				omega[1] = dFloat(initOmega->getValue()[1]);
				omega[2] = dFloat(initOmega->getValue()[2]);

				veloc[0] = dFloat(initVeloc->getValue()[0]);
				veloc[1] = dFloat(initVeloc->getValue()[1]);
				veloc[2] = dFloat(initVeloc->getValue()[2]);

				omega = m_globalRotation.RotateVector(omega);
				veloc = m_globalRotation.RotateVector(veloc.Scale (m_scale));

				//NewtonBodySetOmega(body, &omega[0]);
				//NewtonBodySetVelocity(body, &veloc[0]);
				rigidBodyInfo->SetVelocity(veloc);
				rigidBodyInfo->SetOmega(omega);

				// set the Body as a default Gravity 
				dVariable* var = rigidBodyInfo->CreateVariable("rigidBodyType");
				var->SetValue ("default gravity");
			}
		}
	}
}


//bool ColladaImport::Import (const char* fileName, dScene* world)
bool ColladaImport::Import (const char* const fileName, dPluginInterface* const interface)
{
_ASSERTE (0);
return 0;
/*
	DAE* dae = new DAE;
	_ASSERTE (dae);

	daeInt error = dae->load(fileName);
	_ASSERTE (error == DAE_OK);
	if (error != DAE_OK) {
		delete dae;
		return false;
	}

	m_globalRotation = dYawMatrix (-90.0f * 3.141592f / 180.0f);

	daeDatabase* database = dae->getDatabase();
	_ASSERTE (database);

	daeDocument* document = database->getDocument(daeUInt (0));
	_ASSERTE (document);

	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	_ASSERTE (domRoot);


	domAsset *asset = domRoot->getAsset();
	_ASSERTE (domRoot);

	m_scale = 1.0f;
	domAsset::domUnit* unit = asset->getUnit();
	if (unit) {
		m_scale *= dFloat (unit->getMeter());
	}

	domAsset::domUp_axis* axis = asset->getUp_axis();
	if (axis) {
		const domUpAxisType& value = axis->getValue();
		switch (value)
		{
			case UPAXISTYPE_X_UP:
			{
				m_globalRotation = dYawMatrix(3.14159265f * 0.5f) * m_globalRotation;
				break;
			}

			case UPAXISTYPE_Z_UP:
			{
				m_globalRotation = dPitchMatrix(-3.14159265f * 0.5f) * m_globalRotation;
				break;
			}

			case UPAXISTYPE_Y_UP:
			default:
			{
			}
		}
	}

	NodeChache nodeCache;
	ImageCache imageCache;
	GeometryChache meshCache;
	const domLibrary_visual_scenes_Array &libraryArrays = domRoot->getLibrary_visual_scenes_array ();
	for (int j = 0; j < int (libraryArrays.getCount()); j ++) {
		const domVisual_scene_Array& libraryArray = libraryArrays[j]->getVisual_scene_array();
		for (int i = 0; i < int (libraryArray.getCount()); i ++) {
			domVisual_scene* visualScene = libraryArray[i];
			_ASSERTE (visualScene);
			LoadVisualScene (dae, visualScene, world, meshCache, imageCache, nodeCache);
		}
	}

	// load the physics information if any
	LoadPhysicScene (dae, world, nodeCache, meshCache);

	delete dae;
	return true;
*/
}

