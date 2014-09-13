/////////////////////////////////////////////////////////////////////////////
// Name:        dLineNodeInfo.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
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


#include "dSceneStdafx.h"
#include "dScene.h"
#include "dSceneRender.h"
#include "dLineNodeInfo.h"

D_IMPLEMENT_CLASS_NODE(dLineNodeInfo);

#define D_LINE_DEAFAULT_SUBDIVITION_COUNT 100

dLineNodeInfo::dLineNodeInfo()
	:dGeometryNodeInfo (NULL)
	,m_curve()
	,m_renderSegments(D_LINE_DEAFAULT_SUBDIVITION_COUNT)
{
	SetName ("line");
}

dLineNodeInfo::dLineNodeInfo(dScene* const world)
	:dGeometryNodeInfo (world)
	,m_curve()
	,m_renderSegments(D_LINE_DEAFAULT_SUBDIVITION_COUNT)
{
	SetName ("line");
}


dLineNodeInfo::dLineNodeInfo(const dLineNodeInfo& me)
	:dGeometryNodeInfo (me)
	,m_curve(me.m_curve)
	,m_renderSegments(me.m_renderSegments)
{
	dAssert (0);
}

dLineNodeInfo::~dLineNodeInfo(void)
{
}


void dLineNodeInfo::BakeTransform (const dMatrix& transform)
{
	dVector scale; 
	dMatrix stretchMatrix;

	dMatrix matrix (transform.Inverse4x4() * m_matrix * transform);
	matrix.PolarDecomposition (m_matrix, scale, stretchMatrix);
	matrix = transform * dMatrix (dGetIdentityMatrix(), scale, stretchMatrix);

	matrix.TransformTriplex (&m_curve.GetControlPointArray()[0].m_x, sizeof (dVector), &m_curve.GetControlPointArray()[0].m_x, sizeof (dVector), m_curve.GetControlPointCount());
}


/*
void dLineNodeInfo::BeginBuild ()
{
	NewtonMeshBeginFace(m_mesh);
}

void dLineNodeInfo::AddPolygon (int pointsCount, const neMeshInfoFlatPoint* const points, int materialID)
{
	NewtonMeshAddFace(m_mesh, pointsCount, &points[0].m_x, sizeof (neMeshInfoFlatPoint), materialID);
}

void dLineNodeInfo::EndBuild ()
{
	NewtonMeshEndFace(m_mesh);
}

void dLineNodeInfo::ConvertToTriangles()
{
	NewtonMeshTriangulate (m_mesh);
}

void dLineNodeInfo::ConvertToPolygons()
{
	NewtonMeshPolygonize(m_mesh);
}
void dLineNodeInfo::RepairTJoints ()
{
	NewtonMeshFixTJoints (m_mesh);
}


void dLineNodeInfo::SmoothNormals (dFloat angleInRadiants)
{
	NewtonMeshCalculateVertexNormals(m_mesh, angleInRadiants);
}


void dLineNodeInfo::BuildFromVertexListIndexList(int faceCount, const int* const faceIndexCount, const int* faceMaterialIndex, 
	const dFloat* const vertex, int vertexStrideInBytes, const int* vertexIndex,
	const dFloat* const normal, int normalStrideInBytes, const int* normalIndex,
	const dFloat* const uv0, int uv0StrideInBytes, const int* uv0Index,
	const dFloat* const uv1, int uv1StrideInBytes, const int* uv1Index)
{
	NewtonMeshBuildFromVertexListIndexList(m_mesh, faceCount, faceIndexCount, faceMaterialIndex, 
		vertex, vertexStrideInBytes, vertexIndex,normal, normalStrideInBytes, normalIndex,
		uv0, uv0StrideInBytes, uv0Index, uv1, uv1StrideInBytes, uv1Index);
}

void dLineNodeInfo::RemoveUnusedVertices(dScene* const world, dScene::dTreeNode* const myNode)
{
	dAssert (world->GetInfoFromNode(myNode) == this);

	int vertexCount = NewtonMeshGetVertexCount(m_mesh);
	int* vertexRemapArray = new int [vertexCount];

	NewtonRemoveUnusedVertices(m_mesh, vertexRemapArray);

	for (void* ptr0 = world->GetFirstChildLink (myNode); ptr0; ptr0 = world->GetNextChildLink(myNode, ptr0)) {
		dScene::dTreeNode* node = world->GetNodeFromLink(ptr0);
		dNodeInfo* info = world->GetInfoFromNode(node);
		if (info->IsType(dGeometryNodeModifierInfo::GetRttiType())) {
			dGeometryNodeModifierInfo* const modifier = (dGeometryNodeModifierInfo*) info;
			modifier->RemoveUnusedVertices(vertexRemapArray);
		}
	}

	delete vertexRemapArray;
}



void dLineNodeInfo::CalcutateAABB (dVector& p0, dVector& p1) const
{
//	int strideInBytes = NewtonMeshGetVertexStrideInByte(m_mesh);
//	dFloat64* const vertexList = NewtonMeshGetVertexArray(m_mesh);
//	dFloat t = 1.2f;
//	for (void* face = NewtonMeshGetFirstFace (m_mesh); face; face = NewtonMeshGetNextFace (m_mesh, face)) {
//		if (!NewtonMeshIsFaceOpen (m_mesh, face)) {

//	dMatrix matrix (GetIdentityMatrix());
//	NewtonMeshCalculateOOBB(const NewtonMesh* const mesh, dFloat* const matrix, dFloat* const x, dFloat* const y, dFloat* const z);

	p0 = dVector (1.0e10f, 1.0e10f, 1.0e10f, 0.0f);
	p1 = dVector (-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);

	int strideInBytes = NewtonMeshGetVertexStrideInByte(m_mesh);
	int stride = strideInBytes / sizeof (dFloat64) ;
	dFloat64* const vertexList = NewtonMeshGetVertexArray(m_mesh);
	for (void* ptr = NewtonMeshGetFirstVertex(m_mesh); ptr; ptr = NewtonMeshGetNextVertex(m_mesh, ptr)) {
		int index = NewtonMeshGetVertexIndex (m_mesh, ptr);

		dFloat x = dFloat (vertexList[index * stride + 0]);
		dFloat y = dFloat (vertexList[index * stride + 1]);
		dFloat z = dFloat (vertexList[index * stride + 2]);
		dVector v (m_matrix.TransformVector(dVector (x, y, z, 0.0f)));

		p0[0] = dMin(v[0], p0[0]);
		p0[1] = dMin(v[1], p0[1]);
		p0[2] = dMin(v[2], p0[2]);
					   
		p1[0] = dMax(v[0], p1[0]);
		p1[1] = dMax(v[1], p1[1]);
		p1[2] = dMax(v[2], p1[2]);
	}
}


dFloat dLineNodeInfo::RayCast (const dVector& q0, const dVector& q1) const
{
	//	int vertexCount = NewtonMeshGetVertexCount(m_mesh);
	int strideInBytes = NewtonMeshGetVertexStrideInByte(m_mesh);
	dFloat64* const vertexList = NewtonMeshGetVertexArray(m_mesh);
	dFloat t = 1.2f;

	dVector p0 = m_matrix.UntransformVector(q0);
	dVector p1 = m_matrix.UntransformVector(q1);
	for (void* face = NewtonMeshGetFirstFace (m_mesh); face; face = NewtonMeshGetNextFace (m_mesh, face)) {
		if (!NewtonMeshIsFaceOpen (m_mesh, face)) {

			int indices[1024];
			int vertexCount = NewtonMeshGetFaceIndexCount (m_mesh, face);
			NewtonMeshGetFaceIndices (m_mesh, face, indices);

			dFloat t1 = dPolygonRayCast (p0, p1, vertexCount, vertexList, strideInBytes, indices);
			if (t1 < t) {
				t = t1;		
			}
		}
	}
	return t;
}


void dLineNodeInfo::DrawWireFrame(dSceneRender* const render, dScene* const scene, dScene::dTreeNode* const myNode) const
{
	dAssert (myNode == scene->Find(GetUniqueID()));
	dAssert (scene->GetInfoFromNode(myNode) == this);


	int displayList = render->GetCachedWireframeDisplayList(m_mesh);
	dAssert (displayList > 0);
	
	render->PushMatrix(&m_matrix[0][0]);
	if (GetEditorFlags() & m_selected) {
		dVector color (render->GetColor());
		render->SetColor(dVector (1.0f, 1.0f, 0.0f, 0.0f));
		render->DrawDisplayList(displayList);
		render->SetColor(color);
	} else {
		render->DrawDisplayList(displayList);
	}
	render->PopMatrix();
}
*/

void dLineNodeInfo::DrawWireFrame(dSceneRender* const render, dScene* const scene, dScene::dTreeNode* const myNode) const
{
	dAssert (myNode == scene->Find(GetUniqueID()));
	dAssert (scene->GetInfoFromNode(myNode) == this);

	//	int displayList = render->GetCachedFlatShadedDisplayList(m_mesh);
	//	dAssert (displayList > 0);
	if (m_curve.GetControlPointArray()) {

		dVector savedColor (render->GetColor());

		render->PushMatrix(&m_matrix[0][0]);
		//render->DrawDisplayList(displayList);

		render->BeginLine();
		render->SetColor(dVector (1.0f, 1.0f, 1.0f));

		dFloat scale = 1.0f / m_renderSegments;
		dVector p0 (m_curve.CurvePoint(0.0f));
		for (int i = 1; i <= m_renderSegments; i ++) {
			dFloat u = i * scale;
			dVector p1 (m_curve.CurvePoint(u));
			render->DrawLine (p0, p1);
			p0 = p1;
		}
		render->End();
		render->PopMatrix();

		render->SetColor(savedColor);
	}
}

void dLineNodeInfo::DrawFlatShaded(dSceneRender* const render, dScene* const scene, dScene::dTreeNode* const myNode) const
{
	DrawWireFrame (render, scene, myNode);
}



dCRCTYPE dLineNodeInfo::CalculateSignature() const
{
	dCRCTYPE signature = 0;
	dAssert (0);
#if 0
	int vertexCount = NewtonMeshGetVertexCount (m_mesh); 
	int vertexStride = NewtonMeshGetVertexStrideInByte(m_mesh);
	signature = dCRC64 (NewtonMeshGetVertexArray (m_mesh), vertexStride * vertexCount, signature);

	// for now just compare the vertex array, do no forget to add more text using the face winding and material indexed  


	// save the polygon array
	int faceCount = NewtonMeshGetTotalFaceCount (mesh); 
	int indexCount = NewtonMeshGetTotalIndexCount (mesh); 

	int* const faceArray = new int [faceCount];
	void** const indexArray = new void* [indexCount];
	int* const materialIndexArray = new int [faceCount];
	int* const remapedIndexArray = new int [indexCount];

	NewtonMeshGetFaces (mesh, faceArray, materialIndexArray, indexArray); 

	// save the faces vertex Count
	dIntArrayToString (faceArray, faceCount, buffer, bufferSizeInBytes);
	TiXmlElement* const polygons = new TiXmlElement ("polygons");
	rootNode->LinkEndChild(polygons);
	polygons->SetAttribute("count", faceCount);
	polygons->SetAttribute("faceIndexCount", buffer);

	dIntArrayToString (materialIndexArray, faceCount, buffer, bufferSizeInBytes);
	TiXmlElement* const faceMaterial = new TiXmlElement ("faceMaterial");
	polygons->LinkEndChild(faceMaterial);
	faceMaterial->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetVertexIndex (mesh, indexArray[i]);
		remapedIndexArray[i] = vertexIndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, bufferSizeInBytes);
	TiXmlElement* const positionIndex = new TiXmlElement ("position");
	polygons->LinkEndChild(positionIndex);
	positionIndex->SetAttribute("index", buffer);


	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(mesh, indexArray[i]);
		remapedIndexArray[i] = normalIndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, bufferSizeInBytes);
	TiXmlElement* const normalIndex = new TiXmlElement ("normal");
	polygons->LinkEndChild(normalIndex);
	normalIndex->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(mesh, indexArray[i]);
		remapedIndexArray[i] = uv0IndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, bufferSizeInBytes);
	TiXmlElement* const uv0Index = new TiXmlElement ("uv0");
	polygons->LinkEndChild(uv0Index);
	uv0Index->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(mesh, indexArray[i]);
		remapedIndexArray[i] = uv1IndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, bufferSizeInBytes);
	TiXmlElement* const uv1Index = new TiXmlElement ("uv1");
	polygons->LinkEndChild(uv1Index);
	uv1Index->SetAttribute("index", buffer);
#endif
	return signature;
}

bool dLineNodeInfo::Deserialize (const dScene* const scene, TiXmlElement* const rootNode) 
{
	dAssert (0);
	DeserialiseBase(scene, dGeometryNodeInfo, rootNode);
	//	DeserializeMesh (m_mesh, rootNode); 
	return true;
}


void dLineNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dGeometryNodeInfo, rootNode);

	TiXmlElement* const bezierCurve = new TiXmlElement ("dBezierSpline");
	rootNode->LinkEndChild (bezierCurve);

	bezierCurve->SetAttribute("degree", m_curve.GetDegree());

	int pointCount = m_curve.GetControlPointCount();
	const dVector* const controlPoints = m_curve.GetControlPointArray();
	int bufferSizeInBytes = pointCount * sizeof (dVector) * 12;
	char* const buffer = new char[bufferSizeInBytes];
	dFloatArrayToString (&controlPoints[0][0], pointCount * sizeof (dVector) / sizeof (dFloat), buffer, bufferSizeInBytes);

	TiXmlElement* const ctrlPoints = new TiXmlElement ("controlPoints");
	bezierCurve->LinkEndChild (ctrlPoints);
	ctrlPoints->SetAttribute("float4", pointCount);
	ctrlPoints->SetAttribute("floats", buffer);

	int knotCount = m_curve.GetKnotCount();
	const dFloat* const knotPoints = m_curve.GetKnotArray();
	int buffer1SizeInBytes = knotCount * sizeof (dFloat) * 12;
	char* const buffer1 = new char[buffer1SizeInBytes];
	dFloatArrayToString (knotPoints, knotCount, buffer1, buffer1SizeInBytes);

	TiXmlElement* const knotVector = new TiXmlElement ("knotVector");
	bezierCurve->LinkEndChild (knotVector);
	knotVector->SetAttribute("float", knotCount);
	knotVector->SetAttribute("floats", buffer1);

	delete[] buffer;
	delete[] buffer1;
}

