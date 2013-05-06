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

#include "Common.h"
#include "ConvexApproximation.h"

#define CONVEX_APROXIMATION_CLASS_ID Class_ID(0x3b242416, 0x1fce7cf1)
#include "resource.h"




int ConvexApproximationMouseCallBack::proc (ViewExp *vpt, int msg, int point, int flags, IPoint2 m, Matrix3& mat)
{
	if ((msg==MOUSE_POINT) || (msg==MOUSE_MOVE)) {
		m_obj->BuildMesh();

		// do not forget to select this source node again before leaving
		ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();
		theHold.Begin();
		desc->m_currentInterface->SelectNode(desc->m_sourceNode, 1);
		MSTR undostr; undostr.printf("Select");
		theHold.Accept(undostr);

		return CREATE_STOP;
	} else {
		if ((msg == MOUSE_ABORT) || (msg == RIGHT_BUTTON)) {
			return CREATE_ABORT;
		}
	}
	return CREATE_CONTINUE;

}


ConvexApproximationClassDesc::ConvexApproximationClassDesc()
	:ClassDesc2 ()
	,m_mouseCallback()
	,m_sourceNode(NULL)
	,m_currentInterface(NULL)
{
}

int ConvexApproximationClassDesc::IsPublic() 
{ 
	return TRUE; 
}

void* ConvexApproximationClassDesc::Create(BOOL loading) 
{ 
	INode* const selection = m_currentInterface->GetSelNode(0);
	if (selection) {
		m_sourceNode = selection;
	}
	return new ConvexApproximationObject(loading); 
}

BOOL ConvexApproximationClassDesc::OkToCreate(Interface *i) 
{ 
	if (i->GetSelNodeCount()!=1) return FALSE;

	ObjectState os = i->GetSelNode(0)->GetObjectRef()->Eval(i->GetTime());
	if (os.obj->SuperClassID()!=GEOMOBJECT_CLASS_ID) {
		return FALSE;
	}
	
	m_currentInterface = i;
	return TRUE;
}	



const TCHAR* ConvexApproximationClassDesc::ClassName() 
{ 
	return _T("ConvexPartition"); 
}

SClass_ID ConvexApproximationClassDesc::SuperClassID() 
{ 
	return GEOMOBJECT_CLASS_ID; 
}

Class_ID ConvexApproximationClassDesc::ClassID() 
{ 
	return CONVEX_APROXIMATION_CLASS_ID; 
}

const TCHAR* ConvexApproximationClassDesc::Category() 
{ 
	return _T ("Compound Objects");
}

const TCHAR* ConvexApproximationClassDesc::InternalName() 
{ 
	return GetString(IDS_CLASS_NAME); 
}	

HINSTANCE ConvexApproximationClassDesc::HInstance() 
{ 
	return hInstance; 
}


ClassDesc* ConvexApproximationClassDesc::GetDescriptor()
{
	static ConvexApproximationClassDesc desc;
	return &desc; 
}

DWORD WINAPI ConvexApproximationClassDesc::ReportMaxProgress(LPVOID arg)
{
/*
	int i, percent;
	Interface *ip = theUtility.ip;
	for (i = 0; i < 1000; i++) {
		percent = i/10;
		ip->ProgressUpdate(percent);
		if (ip->GetCancel()) {
			switch(MessageBox(ip->GetMAXHWnd(), _M("Really Cancel"), _M("Question"), MB_ICONQUESTION | MB_YESNO)) {
case IDYES:
	return(0);
case IDNO:
	ip->SetCancel(FALSE);
			}
		}
	}
*/
	return(0);
}

void ConvexApproximationClassDesc::ReportProgress(float progressNormalzedPercent)
{
	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();

	int progress = int (progressNormalzedPercent * 100.0f);
	if (progress != desc->m_progress) {
		desc->m_progress = progress;
		Interface* const inteface = desc->m_currentInterface;
		inteface->ProgressUpdate(progress);
	}
}


ConvexApproximationObject::ConvexApproximationObject(BOOL loading) 
	:m_createParameterUI(NULL)
	,m_maxConcavity(NULL)
	,m_maxConcavitySpinner(NULL)
	,m_maxSegments(NULL)
	,m_maxSegmentsSpinner(NULL)
	,m_currentMaxCount (32)
	,m_currentConcavity (0.01f)
	,m_UI_initalizationCompleted (false)
{
}


void ConvexApproximationObject::BeginEditParams( IObjParam *ip, ULONG flags, Animatable *prev )
{
	GeomObject::BeginEditParams(ip,flags,prev);

	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();
	_ASSERTE (ip == GetCOREInterface());

	desc->m_currentInterface = ip;
	m_createParameterUI = ip->AddRollupPage(hInstance, MAKEINTRESOURCE(IDD_CONVEX_APPROXIMATION_PANE), ConvexApproximationObject::Proc, _T("Newton World"), LPARAM (this));
}

void ConvexApproximationObject::EndEditParams( IObjParam *ip, ULONG flags,Animatable *next )
{
	GeomObject::EndEditParams(ip,flags,next);

	_ASSERTE (ip == GetCOREInterface());
	ip->DeleteRollupPage(m_createParameterUI);
}

void ConvexApproximationObject::InitUI(HWND hWnd)
{
	m_UI_initalizationCompleted = false;

	HWND maxConcavityUI = GetDlgItem(hWnd, IDC_MAX_CONCAVITY);
	HWND maxConcavitySpinnerUI = GetDlgItem(hWnd, IDC_MAX_CONCAVITY_SPINNER);
	m_maxConcavity = GetICustEdit(maxConcavityUI);
	m_maxConcavitySpinner = GetISpinner (maxConcavitySpinnerUI);
	m_maxConcavitySpinner->LinkToEdit (maxConcavityUI, EDITTYPE_POS_FLOAT);
	m_maxConcavitySpinner->SetScale (0.01f);
	m_maxConcavitySpinner->SetLimits(0.01f, 100.0f);
	m_maxConcavitySpinner->SetValue(m_currentConcavity, TRUE);


	HWND maxSegmentsUI = GetDlgItem(hWnd, IDC_MAX_SEGMENTS);
	HWND maxSegmentsSpinnerUI = GetDlgItem(hWnd, IDC_MAX_SEGMENTS_SPINNER);
	m_maxSegments = GetICustEdit(maxSegmentsUI);
	m_maxSegmentsSpinner = GetISpinner (maxSegmentsSpinnerUI);
	m_maxSegmentsSpinner->LinkToEdit (maxSegmentsUI, EDITTYPE_POS_INT);
	m_maxSegmentsSpinner->SetScale (1.0f);
	m_maxSegmentsSpinner->SetLimits(1, 256);
	m_maxSegmentsSpinner->SetValue(m_currentMaxCount, TRUE);

	m_UI_initalizationCompleted = true;
}

void ConvexApproximationObject::DestroyUI(HWND hWnd)
{
	ReleaseICustEdit (m_maxConcavity);
	ReleaseISpinner (m_maxConcavitySpinner);

	ReleaseICustEdit (m_maxSegments);
	ReleaseISpinner (m_maxSegmentsSpinner);
}


INT_PTR CALLBACK ConvexApproximationObject::Proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	ConvexApproximationObject* const me = (ConvexApproximationObject *)GetWindowLong (hWnd, GWLP_USERDATA);
	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();

	switch (msg) 
	{
		case WM_INITDIALOG:
		{
			ConvexApproximationObject* const me = (ConvexApproximationObject *)lParam;
			SetWindowLong(hWnd, GWLP_USERDATA, (LONG)me);
			me->InitUI(hWnd);
			break;
		}

		case WM_DESTROY:
		{
			//world->RigidBodyUIPane::DestroyUI(hWnd);
			me->DestroyUI(hWnd);
			break;
		}
	
		case WM_ENABLE:
		{
			EnableWindow(hWnd, (BOOL) wParam);
			break;
		}

		case CC_SPINNER_CHANGE:
		{
			if (me->m_UI_initalizationCompleted) {
				switch (LOWORD(wParam))
				{
					
					case IDC_MAX_CONCAVITY_SPINNER:
					{
						me->m_currentConcavity = me->m_maxConcavity->GetFloat();	
						break;
					}

					case IDC_MAX_SEGMENTS_SPINNER:
					{
						me->m_currentMaxCount = me->m_maxSegments->GetInt();	
					}
				}
			}

			return TRUE;
		}


		case WM_COMMAND:
		{
			if (me->m_UI_initalizationCompleted) {
				switch (LOWORD(wParam))
				{
					case IDC_MAX_CONCAVITY:
					{
						me->m_currentConcavity = me->m_maxConcavity->GetFloat();	
					}
					break;

					case IDC_MAX_SEGMENTS:
					{
						me->m_currentMaxCount = me->m_maxSegments->GetInt();	
					}
					break;
				}
			}
		}

		default:
		return FALSE;
	}
	return TRUE;
}


CreateMouseCallBack* ConvexApproximationObject::GetCreateMouseCallBack() 
{
	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();
	desc->m_mouseCallback.m_obj = this;
	return(&desc->m_mouseCallback);
}









PolyObject* ConvexApproximationObject::GetPolyObject (ObjectState* const os, int& deleteIt)
{
	PolyObject* poly = NULL;
	deleteIt = FALSE;
	Object* const obj = os->obj;
	if (obj->CanConvertToType(Class_ID(POLYOBJ_CLASS_ID, 0))) { 
		poly = (PolyObject *) obj->ConvertToType(0, Class_ID(POLYOBJ_CLASS_ID, 0));
		// Note that the TriObject should only be deleted
		// if the pointer to it is not equal to the object
		// pointer that called ConvertToType()
		if (obj != poly) {
			deleteIt = TRUE;
		}
	}
	return poly;
}


void ConvexApproximationObject::LoadGeometries (NewtonMesh* const meshOut, const dMatrix& matrix)
{
	INode* stack[1024];

	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();

	INode* const sourceNode = desc->m_sourceNode;
	_ASSERTE (sourceNode);
	stack[0] = sourceNode;
	int stackIndex = 1;


	dMatrix orthogonalRootTransform (matrix.Inverse());

	while (stackIndex) {
		stackIndex --;
		INode* const node = stack[stackIndex];

		// Get Transformation matrix at frame 0
		//dMatrix matrix (GetMatrixFromMaxMatrix (node->GetNodeTM (0)) * orthogonalRootTransform);
		dMatrix matrix (GetMatrixFromMaxMatrix (node->GetObjectTM(0)) * orthogonalRootTransform);
		
		//nodeMap.Insert(sceneNode, node);
		ObjectState os (node->EvalWorldState(0)); 

		// The obj member of ObjectState is the actual object we will export.
		if (os.obj) {

			// We look at the super class ID to determine the type of the object.
			switch(os.obj->SuperClassID()) 
			{
				case GEOMOBJECT_CLASS_ID: 
				{
					if (!node->GetBoneNodeOnOff()) {
						AddPolygonFromObject(node, &os, meshOut, matrix);
					}
					break;
				}
			}
		}

		for (int i = 0; i < node->NumberOfChildren(); i ++) {
			stack[stackIndex] = node->GetChildNode(i);
			stackIndex ++;
			_ASSERTE (stackIndex * sizeof (INode*) < sizeof (stack));	
		}
	}
}


void ConvexApproximationObject::AddPolygonFromObject(INode* const node, ObjectState* const os, NewtonMesh* const meshOut, const dMatrix& matrix)
{
	BOOL needDel = FALSE;
	PolyObject* const poly = GetPolyObject (os, needDel); 
	if (poly) {

		float polygon[32][12];
		memset (polygon, 0, sizeof (polygon));

		MNMesh& maxMesh = poly->GetMesh();
		int facesCount = maxMesh.FNum();
		int vertexCount = maxMesh.VNum();

		if (facesCount && vertexCount) {
			for (int i = 0; i < facesCount; i ++) {
				MNFace* const face = maxMesh.F(i);

				for (int j = 0; j < face->deg; j ++) {
					int index = face->vtx[j];
					Point3 p (maxMesh.P(index));
					dVector v (matrix.TransformVector(dVector (p.x, p.y, p.z, 0.0f)));
					polygon[j][0] = v.m_x;
					polygon[j][1] = v.m_y;
					polygon[j][2] = v.m_z;
				}
				NewtonMeshAddFace(meshOut, face->deg, &polygon[0][0], 12 * sizeof (float), 0);
			}
		}
	}
	if (needDel) {
		delete poly;
	}
}


void ConvexApproximationObject::BuildMesh()
{

	// since max does no provide the iNode that will own this mesh I have no choice bu to apply the root matrix to all vertex
	ConvexApproximationClassDesc* const desc = (ConvexApproximationClassDesc*) ConvexApproximationClassDesc::GetDescriptor();
	INode* const sourceNode = desc->m_sourceNode;
	//dMatrix rootMatrix1 (GetMatrixFromMaxMatrix (sourceNode->GetNodeTM (0)));
	dMatrix rootMatrix (GetMatrixFromMaxMatrix (sourceNode->GetObjectTM(0)));

	dVector scale;
	dMatrix stretchAxis;
	dMatrix orthogonalRootTransform;
	rootMatrix.PolarDecomposition (orthogonalRootTransform, scale, stretchAxis);
	orthogonalRootTransform = orthogonalRootTransform.Inverse();

	// create a Newton world, as a manager of everything Newton related stuff
	NewtonWorld* const world = NewtonCreate ();

	// create an empty mesh and load the max mesh to it
	NewtonMesh* const sourceMesh = NewtonMeshCreate (world);

	// load all faces
	NewtonMeshBeginFace(sourceMesh);
	LoadGeometries (sourceMesh, orthogonalRootTransform);
	NewtonMeshEndFace(sourceMesh);


	// make a convex approximation form this newton mesh effect
	desc->m_progress = -1;
	Interface* const inteface = desc->m_currentInterface;
	
	inteface->ProgressStart("Creation Convex approx ...", TRUE, ConvexApproximationClassDesc::ReportMaxProgress, NULL);
	NewtonMesh* approximationMesh = NewtonMeshApproximateConvexDecomposition (sourceMesh, m_currentConcavity, 0.2f, m_currentMaxCount, 1000, ConvexApproximationClassDesc::ReportProgress);
	inteface->ProgressEnd();

	NewtonMeshDestroy (sourceMesh);



	// now convert the new mesh to a max poly Object
	MNMesh& maxMesh = GetMesh();
	maxMesh.ClearAndFree();

	int faceCount = 0;
	int vertexCount = NewtonMeshGetVertexCount(approximationMesh);
	for (void* face = NewtonMeshGetFirstFace(approximationMesh); face; face = NewtonMeshGetNextFace(approximationMesh, face)) {
		if (!NewtonMeshIsFaceOpen(approximationMesh, face)) {
			faceCount ++;
		}
	}

	//maxMesh.Clear();
	maxMesh.setNumVerts(vertexCount);
	maxMesh.setNumFaces(faceCount);

	// add all vertex
	int vertexStride = NewtonMeshGetVertexStrideInByte(approximationMesh) / sizeof (dFloat64);
	dFloat64* const vertex = NewtonMeshGetVertexArray (approximationMesh); 
	for (int j = 0; j < vertexCount; j ++) {
		dVector p (orthogonalRootTransform.TransformVector(dVector (float (vertex[vertexStride * j + 0]), float (vertex[vertexStride * j + 1]), float (vertex[vertexStride * j + 2]), float(1.0f))));
		maxMesh.P(j) = Point3 (p.m_x, p.m_y, p.m_z);
	}

	// count the number of face and make a face map
	int faceIndex = 0;
	for (void* face = NewtonMeshGetFirstFace(approximationMesh); face; face = NewtonMeshGetNextFace(approximationMesh, face)) {
		if (!NewtonMeshIsFaceOpen(approximationMesh, face)) {
			int faceIndices[256];
			int indexCount = NewtonMeshGetFaceIndexCount (approximationMesh, face);

			NewtonMeshGetFaceIndices (approximationMesh, face, faceIndices);
			MNFace* const face = maxMesh.F(faceIndex);
			face->MakePoly(indexCount, faceIndices, NULL, NULL);
			face->material = 0;
			faceIndex ++;
		}
	}

	maxMesh.InvalidateGeomCache();
	maxMesh.InvalidateTopoCache();
	maxMesh.FillInMesh();
	maxMesh.AutoSmooth(45.0f * 3.1416f / 160.0f, false, false);


	NewtonMeshDestroy (approximationMesh);
	NewtonDestroy (world);
}
