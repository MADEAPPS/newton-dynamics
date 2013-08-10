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
#include "RigidBodyWorld.h"
#include "RigidBodyController.h"


#define TIMER_ID	1
#define RIGIDBOGY_WORLD_CLASS_ID Class_ID(0x6185a57, 0x3a1f2f69)


RigidBodyData::RigidBodyData()
{
	memset (this, 0, sizeof (RigidBodyData));
	m_oldControlerID = Class_ID (PRS_CONTROL_CLASS_ID, 0);
}	

RigidBodyData::~RigidBodyData()
{
	DeleteBody();
}

void RigidBodyData::CreateBody(NewtonCollision* const collision, const dVector& veloc, const dVector& omega)
{
	_ASSERTE (!m_body);
	RigidBodyWorldDesc& me = *(RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();
	
	dMatrix matrix (GetIdentityMatrix()); 
	m_body = NewtonCreateDynamicBody(me.m_newton, collision, &matrix[0][0]);

	//NewtonBodySetMassMatrix(m_body, m_mass, m_mass * m_inertia.m_x, m_mass * m_inertia.m_y, m_mass * m_inertia.m_z);
	NewtonBodySetMassProperties(m_body, m_mass, collision);
	NewtonBodySetCentreOfMass(m_body, &m_origin[0]);

	NewtonBodySetVelocity(m_body, &veloc[0]);
	NewtonBodySetOmega(m_body, &omega[0]);
	NewtonBodySetForceAndTorqueCallback(m_body, RigidBodyController::ApplyGravityForce);
}

void RigidBodyData::DeleteBody()
{
	if (m_body) {
	RigidBodyWorldDesc* const plugin = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();
		NewtonDestroyBody(m_body);
	}
	m_body = NULL;
}

void RigidBodyData::LoadCollision (void* const serializeHandle, void* buffer, int size)
{
	ULONG nwrit;
	ILoad* const iload = (ILoad*)serializeHandle;
	iload->Read(buffer, size, &nwrit);
}

void RigidBodyData::SaveCollision (void* const serializeHandle, const void* buffer, int size)
{
	ULONG nwrit;
	ISave* const isave = (ISave*)serializeHandle;
	isave->Write(buffer, size, &nwrit);
}


void RigidBodyData::Load(ILoad* const iload)
{
	ULONG nwrit;
	int revision;
	dVector veloc;
	dVector omega;
	dMatrix matrix;
	RigidBodyWorldDesc& me = *(RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	iload->Read(&revision, sizeof (revision), &nwrit);
	iload->Read(&m_oldControlerID, sizeof (m_oldControlerID), &nwrit);
	iload->Read(&m_collisionShape, sizeof (m_collisionShape), &nwrit);
	iload->Read(&m_hideGizmos, sizeof (m_hideGizmos), &nwrit);
	iload->Read(&m_mass, sizeof (m_mass), &nwrit);
	iload->Read(&m_inertia, sizeof (m_inertia), &nwrit);
	iload->Read(&m_origin, sizeof (m_origin), &nwrit);

	iload->Read(&matrix, sizeof (matrix), &nwrit);
	iload->Read(&veloc, sizeof (veloc), &nwrit);
	iload->Read(&omega, sizeof (omega), &nwrit);
	NewtonCollision* const collision = NewtonCreateCollisionFromSerialization (me.m_newton, LoadCollision, iload);


	CreateBody(collision, veloc, omega);
	NewtonBodySetMatrix(m_body, &matrix[0][0]);
	NewtonDestroyCollision(collision);
}


void RigidBodyData::Save(ISave* const isave)
{
	ULONG nwrit;
	int revision = D_FILE_REVISION;
	dVector mass;
	dVector com;
	dVector veloc;
	dVector omega;
	dMatrix matrix;

	RigidBodyWorldDesc& me = *(RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	NewtonBodyGetMatrix(m_body, &matrix[0][0]);
	NewtonBodyGetVelocity(m_body, &veloc[0]);
	NewtonBodyGetOmega(m_body, &omega[0]);

	NewtonCollision* const collision = NewtonBodyGetCollision(m_body);

	isave->Write(&revision, sizeof (revision), &nwrit);
	isave->Write(&m_oldControlerID, sizeof (m_oldControlerID), &nwrit);
	isave->Write(&m_collisionShape, sizeof (m_collisionShape), &nwrit);
	isave->Write(&m_hideGizmos, sizeof (m_hideGizmos), &nwrit);
	isave->Write(&m_mass, sizeof (m_mass), &nwrit);
	isave->Write(&m_inertia, sizeof (m_inertia), &nwrit);
	isave->Write(&m_origin, sizeof (m_origin), &nwrit);

	isave->Write(&matrix, sizeof (matrix), &nwrit);
	isave->Write(&veloc, sizeof (veloc), &nwrit);
	isave->Write(&omega, sizeof (omega), &nwrit);
	NewtonCollisionSerialize (me.m_newton, collision, SaveCollision, isave);
}


RigidBodyWorldDesc::RigidBodyWorldDesc ()
	:ClassDesc2()
	,m_updateRigidBodyMatrix(true)
	,m_minFps (120.0f)
	,m_gravity(0.0f, -9.8f, 0.0f, 0.0f)
	,m_systemMatrix (dVector (0.0f, 0.0f, 1.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f, 0.0f), dVector (0.0f, 1.0f, 0.0f, 0.0f), dVector (0.0f, 0.0f, 0.0f, 1.0f))
	,m_systemMatrixInv (m_systemMatrix.Inverse())
{
//	RegisterNotification(OnPostCloneNode, this, NOTIFY_POST_NODES_CLONED);
	RegisterNotification(OnPostLoadScene, this, NOTIFY_FILE_POST_OPEN);
	RegisterNotification(OnAddedNode, this, NOTIFY_SCENE_ADDED_NODE);
	RegisterNotification(OnPreDeleteNode, this, NOTIFY_SCENE_PRE_DELETED_NODE);

//	Interface* const inteface = GetCOREInterface();
//	RigBodyWorldUpdate* const timeCallback = this;
//	inteface->RegisterTimeChangeCallback(timeCallback);	

	// I can not create the body here because pthreads can no be destroyed at C++ start pr when calling at exit.
	// basically I need to plug this in a Max Notification functions, for now just comment this out
//	m_newton = NewtonCreate();
}

RigidBodyWorldDesc::~RigidBodyWorldDesc ()
{
	// I can no create the body here because pthreads can no be destroyed at C++ start pr when calling at exit.
	// basically I need to plug this in a Max Notification functions, for now just comment this out
//	_ASSERTE (m_newton);
//	NewtonDestroy (m_newton);

//	UnRegisterNotification(OnPostCloneNode, this, NOTIFY_POST_NODES_CLONED);
	UnRegisterNotification(OnPostLoadScene, this, NOTIFY_FILE_POST_OPEN);
	UnRegisterNotification(OnPreDeleteNode, this, NOTIFY_SCENE_PRE_DELETED_NODE);

//	Interface* const inteface = GetCOREInterface();
//	RigBodyWorldUpdate* const timeCallback = this;
//	inteface->UnRegisterTimeChangeCallback(timeCallback);	
}

int RigidBodyWorldDesc::IsPublic() 
{ 
	return TRUE; 
}

void* RigidBodyWorldDesc::Create(BOOL loading) 
{ 
	return new RigidBodyWorld;
}


const TCHAR* RigidBodyWorldDesc::ClassName() 
{ 
//	return GetString(IDS_CLASS_NAME); 
	return _T("Newton"); 
}

SClass_ID RigidBodyWorldDesc::SuperClassID() 
{ 
	return UTILITY_CLASS_ID; 
}

Class_ID RigidBodyWorldDesc::ClassID() 
{ 
	return RIGIDBOGY_WORLD_CLASS_ID; 
}

const TCHAR* RigidBodyWorldDesc::Category() 
{ 
	return GetString(IDS_CATEGORY); 
}

const TCHAR* RigidBodyWorldDesc::InternalName() 
{ 
	return _T("NewtonPlugin"); 
}	

HINSTANCE RigidBodyWorldDesc::HInstance() 
{ 
	_ASSERTE (0);
	return hInstance; 
}				

void RigidBodyWorldDesc::ResetClassParams (BOOL fileReset)
{
	ClassDesc2::ResetClassParams (fileReset);
}

ClassDesc* RigidBodyWorldDesc::GetDescriptor()
{ 
	static RigidBodyWorldDesc desc;
	return &desc; 
}

BOOL RigidBodyWorldDesc::NeedsToSave() 
{
	return TRUE; 
}

IOResult RigidBodyWorldDesc::Load(ILoad* iload)
{
	IOResult ret = ClassDesc2::Load(iload);

	ULONG retVal;
	iload->OpenChunk();
	iload->Read(&m_gravity, sizeof (m_gravity), &retVal);
	iload->Read(&m_minFps, sizeof (m_minFps), &retVal);
	iload->CloseChunk();

	return ret;
}

IOResult RigidBodyWorldDesc::Save(ISave* isave)
{
	IOResult ret = ClassDesc2::Save(isave);

	ULONG retVal;
	isave->BeginChunk(USHORT (ClassID().PartB()));
	isave->Write(&m_gravity, sizeof (m_gravity), &retVal);
	isave->Write(&m_minFps, sizeof (m_minFps), &retVal);
	isave->EndChunk();

	return ret;
}


RigidBodyController* RigidBodyWorldDesc::GetRigidBodyControl(INode* const node) const
{
	Control* const control = node->GetTMController();
	RigidBodyControllerDesc& controlDesc = *(RigidBodyControllerDesc*)RigidBodyControllerDesc::GetDescriptor();

	if (control && (control->ClassID() == controlDesc.ClassID())) {
		return (RigidBodyController*)control;
	}
	return NULL;
}


void RigidBodyWorldDesc::OnAddedNode(void* param, NotifyInfo* info)
{
	RigidBodyWorldDesc* const me = (RigidBodyWorldDesc*) param;
	INode* const node = (INode*)info->callParam;

	RigidBodyController* const myControl = me->GetRigidBodyControl(node);
	if (myControl) {
		_ASSERTE (myControl->m_undoCollision);

		TimeValue t (GetCOREInterface()->GetTime());
		Matrix3 matrix (node->GetNodeTM (t));		

		myControl->CreateBody (myControl->m_undoCollision, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (0.0f, 0.0f, 0.0f, 0.0f)); 
		NewtonBodySetUserData(myControl->m_body, node);

		node->SetNodeTM(t, matrix);
	}

}

void RigidBodyWorldDesc::OnPreDeleteNode(void* param, NotifyInfo* info)
{
	RigidBodyWorldDesc* const me = (RigidBodyWorldDesc*) param;
	INode* const node = (INode*)info->callParam;

	RigidBodyController* const myControl = me->GetRigidBodyControl(node);
	if (myControl) {
		myControl->DeleteBody();
	}
}

/*
void RigidBodyWorldDesc::OnPostCloneNode(void* param, NotifyInfo* info)
{
	struct CloneData
	{ 
		INodeTab* origNodes; 
		INodeTab* clonedNodes; 
		CloneType cloneType;
	} ;

	RigidBodyWorldDesc* const me = (RigidBodyWorldDesc*) param;
	CloneData* const data = (CloneData*)info->callParam;
	
	const INodeTab& origNodes = *data->origNodes; 
	const INodeTab& clonedNodes = *data->clonedNodes; 
	_ASSERTE (origNodes.Count() == clonedNodes.Count());

	RigidBodyControllerDesc& controlDesc = *(RigidBodyControllerDesc*)RigidBodyControllerDesc::GetDescriptor();

	TimeValue t (GetCOREInterface()->GetTime());
	for (int i = 0; i < origNodes.Count(); i ++) {
		INode* const origNode = origNodes[i];

		RigidBodyData* const origData = me->GetRigidBodyControl(origNode);
		if (origData) {

			INode* const cloneNode = clonedNodes[i];
			RigidBodyData* const cloneData = me->GetRigidBodyControl(cloneNode);
			_ASSERTE (cloneData);

			memcpy (cloneData, origData, sizeof (RigidBodyData));

			NewtonCollision* const collision = NewtonBodyGetCollision(origData->m_body);

			dMatrix matrix;
			NewtonBodyGetMatrix(origData->m_body, &matrix[0][0]);

			dVector veloc;
			NewtonBodyGetVelocity(origData->m_body, &veloc[0]);
			dVector omega;
			NewtonBodyGetOmega(origData->m_body, &omega[0]);

			cloneData->CreateBody(collision, veloc, omega);
			NewtonBodySetUserData(cloneData->m_body, cloneNode);
			Matrix3 cloneMatrix (cloneNode->GetNodeTM (t));		
			cloneNode->SetNodeTM(t, cloneMatrix);
		}
	}
}
*/

void RigidBodyWorldDesc::OnPostLoadScene (void* param, NotifyInfo* info)
{
	RigidBodyWorldDesc& me = *(RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	dList<INode*> list;
	me.GetNodeList (list);
	for (dList<INode*>::dListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
		INode* const node = ptr->GetInfo();
		RigidBodyController* const control = me.GetRigidBodyControl(node);
		if (control) {
			_ASSERTE (control->m_body);
			NewtonBodySetUserData(control->m_body, node);
		}
	}

	GetCOREInterface()->ForceCompleteRedraw(); 
}

void RigidBodyWorldDesc::GetNodeList (dList<INode*>& list)
{
	int stackIndex;
	INode* stack[4096];

	stackIndex = 1;
	Interface* const ip = GetCOREInterface();
	stack[0] = ip->GetRootNode();

	while (stackIndex) {
		stackIndex --;
		INode* const node = stack[stackIndex];
		list.Append(node); 

		for (int i = 0; i < node->NumberOfChildren(); i ++) {
			stack[stackIndex] = node->GetChildNode(i);
			stackIndex ++;
			_ASSERTE (stackIndex * sizeof (INode*) < sizeof (stack));	
		}
	}
}



RigidBodyWorld::RigidBodyWorld()
	:UtilityObj()
	,m_selectionChange(true)
	,m_newtonBodyUI(NULL)
	,m_newtonWorldUI(NULL)
{
}

RigidBodyWorld::~RigidBodyWorld()
{
}

SClass_ID RigidBodyWorld::SuperClassID()
{
	return RigidBodyWorldDesc::GetDescriptor()->SuperClassID();
}

Class_ID RigidBodyWorld::ClassID()
{
	return RigidBodyWorldDesc::GetDescriptor()->ClassID();
}


void RigidBodyWorld::DeleteThis ()
{
}


void RigidBodyWorld::InitUI(HWND hWnd)
{
	RigidBodyWorldDesc* const desc = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	HWND minFps = GetDlgItem(hWnd, IDC_MINUMIN_SIMULATION_RATE);
	HWND gravity_x = GetDlgItem(hWnd, IDC_GRAVITY_X);
	HWND gravity_y = GetDlgItem(hWnd, IDC_GRAVITY_Y);
	HWND gravity_z = GetDlgItem(hWnd, IDC_GRAVITY_Z);

	m_minFps = GetICustEdit(minFps);
	m_gravity[0] = GetICustEdit(gravity_x);
	m_gravity[1] = GetICustEdit(gravity_y);
	m_gravity[2] = GetICustEdit(gravity_z);


	float scale = 1.0f / float (GetMasterScale(UNITS_METERS));
	dVector gravity = desc->m_systemMatrixInv.RotateVector(desc->m_gravity.Scale (scale));

	m_minFps->SetText(desc->m_minFps);
	m_gravity[0]->SetText(gravity.m_x, 1);
	m_gravity[1]->SetText(gravity.m_y, 1);
	m_gravity[2]->SetText(gravity.m_z, 1);

	RegisterNotification(OnUndoRedo, this, NOTIFY_SCENE_UNDO);
	RegisterNotification (OnUndoRedo, this, NOTIFY_SCENE_REDO);
}

void RigidBodyWorld::DestroyUI(HWND hWnd)
{
	ReleaseICustEdit (m_minFps);
	ReleaseICustEdit (m_gravity[0]);
	ReleaseICustEdit (m_gravity[1]);
	ReleaseICustEdit (m_gravity[2]);

	UnRegisterNotification(OnUndoRedo, this, NOTIFY_SCENE_UNDO);
	UnRegisterNotification (OnUndoRedo, this, NOTIFY_SCENE_REDO);

}


void RigidBodyWorld::BeginEditParams (Interface *ip, IUtil *iu)
{
	_ASSERTE (ip == GetCOREInterface());
	m_newtonWorldUI = ip->AddRollupPage(hInstance, MAKEINTRESOURCE(IDD_NEWTON_WORLD_PANE), RigidBodyWorld::Proc, _T("Newton World"), LPARAM (this));
	m_newtonBodyUI = ip->AddRollupPage(hInstance, MAKEINTRESOURCE(IDD_NEWTON_BODY_PANE), RigidBodyUIPane::Proc, _T("RigidBodies properties"), LPARAM (this));
	SelectionSetChanged (ip, iu);
}

void RigidBodyWorld::EndEditParams (Interface *ip, IUtil *iu)
{
//	m_iu = NULL;
	_ASSERTE (ip == GetCOREInterface());
	ip->DeleteRollupPage(m_newtonWorldUI);
	ip->DeleteRollupPage(m_newtonBodyUI);
}



INT_PTR CALLBACK RigidBodyWorld::Proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	RigidBodyWorld* const world = (RigidBodyWorld *)GetWindowLong (hWnd, GWLP_USERDATA);
	RigidBodyWorldDesc* const desc = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	switch (msg) 
	{
		case WM_INITDIALOG:
		{
			
			RigidBodyWorld* const world = (RigidBodyWorld *)lParam;
			SetWindowLong(hWnd, GWLP_USERDATA, (LONG)world);

			world->m_myWindow = hWnd;
			world->RigidBodyWorld::InitUI(hWnd);
			break;
		}

		case WM_DESTROY:
		{
			world->StopsSimulation ();
			world->RigidBodyWorld::DestroyUI(hWnd);
			break;
		}

		case WM_ENABLE:
		{
			//EnableWindow(obj->m_worldPaneHandle, (BOOL) wParam);
			break;
		}

		case WM_TIMER:
		{
			world->UpdatePhysics ();
			break;
		}


		case WM_COMMAND:
		{
			switch (LOWORD(wParam)) 
			{
				case IDC_MAKE_RIGIDBODY:
				{
					world->StopsSimulation ();
					Interface* const ip = GetCOREInterface();
					int selectionCount = ip->GetSelNodeCount();
					for (int i = 0; i < selectionCount; i ++) {
						INode* const node = ip->GetSelNode(i);
						Object* const obj = node->GetObjOrWSMRef();
						if (obj) {
							world->AttachRigiBodyController (node);
						}
					}
					world->UpdateViewPorts();
					break;
				}

				case IDC_DELETE_RIGIDBODY:
				{
					world->StopsSimulation ();
					Interface* const ip = GetCOREInterface();
					int selectionCount = ip->GetSelNodeCount();
					for (int i = 0; i < selectionCount; i ++) {
						INode* const node = ip->GetSelNode(i);
						world->DetachRigiBodyController (node);
					}
					world->UpdateViewPorts ();

					break;
				}

				case IDC_SHOW_GIZMOS:
				{
					world->StopsSimulation ();
					for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; body = NewtonWorldGetNextBody(desc->m_newton, body)) {
						INode* const node = (INode*)NewtonBodyGetUserData(body);
						RigidBodyController* const bodyInfo = (RigidBodyController*)desc->GetRigidBodyControl(node);
						_ASSERTE (bodyInfo);
						bodyInfo->m_hideGizmos = FALSE;
					}
					world->UpdateViewPorts();
					break;
				}

				case IDC_HIDE_GIZMOS:
				{
					world->StopsSimulation ();
					for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; body = NewtonWorldGetNextBody(desc->m_newton, body)) {
						INode* const node = (INode*)NewtonBodyGetUserData(body);
						RigidBodyController* const bodyInfo = (RigidBodyController*)desc->GetRigidBodyControl(node);
						_ASSERTE (bodyInfo);
						bodyInfo->m_hideGizmos = TRUE;
					}
					world->UpdateViewPorts();
					break;
				}

				case IDC_SELECT_ALL:
				{
					world->StopsSimulation ();
					world->m_selectionChange = false;
					Interface* const ip = GetCOREInterface();
					ip->ClearNodeSelection(FALSE);
					for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; body = NewtonWorldGetNextBody(desc->m_newton, body)) {
						INode* const node = (INode*)NewtonBodyGetUserData(body);
						ip->SelectNode(node, 0); 
					}
					world->m_selectionChange = true;
					world->SelectionSetChanged (ip, NULL);
					world->UpdateViewPorts();
					break;
				}
				

				case IDC_REMOVE_ALL:
				{
					world->StopsSimulation ();
					Interface* const ip = GetCOREInterface();
					ip->ClearNodeSelection(FALSE);
					for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; ) {
						INode* const node = (INode*)NewtonBodyGetUserData(body);
						body = NewtonWorldGetNextBody(desc->m_newton, body);
						world->DetachRigiBodyController (node);
					}
					world->UpdateViewPorts();
					break;
				}

				case IDC_MINUMIN_SIMULATION_RATE:
				{
					world->StopsSimulation ();
					desc->m_minFps = world->m_minFps->GetFloat();
					break;
				}

				case IDC_GRAVITY_X:
				case IDC_GRAVITY_Y:
				case IDC_GRAVITY_Z:
				{
					world->StopsSimulation ();
					dVector gravity (world->m_gravity[0]->GetFloat(), world->m_gravity[1]->GetFloat(), world->m_gravity[2]->GetFloat(), 0.0f);
					//world->m_gravity[0]->SetText(gravity.m_x, 1);
					//world->m_gravity[1]->SetText(gravity.m_y, 1);
					//world->m_gravity[2]->SetText(gravity.m_z, 1);
					desc->m_gravity = desc->m_systemMatrix.RotateVector(gravity.Scale(float (GetMasterScale(UNITS_METERS))));
					break;
				}


				case IDC_PREVIEW_WORLD:
				{
					if (IsDlgButtonChecked(hWnd, IDC_PREVIEW_WORLD) == BST_CHECKED) {
						world->Undo();
						unsigned timeOut = unsigned (1000.0f / desc->m_minFps);
						SetTimer(hWnd, TIMER_ID, timeOut, NULL);
					} else {
						world->StopsSimulation ();
					}

					break;
				}

				case IDC_STEP_WORLD:
				{
					world->StopsSimulation ();
					world->Undo();
					world->UpdatePhysics ();
					break;
				}
			}

			break;
		}

		default:
		return FALSE;
	}
	return TRUE;
}


void RigidBodyWorld::SelectionSetChanged (Interface *ip, IUtil *iu)
{
	if (m_selectionChange) {
		RigidBodyUIPane::SelectionSetChanged();
//		m_currentSelection.RemoveAll();
//		int selectionCount = m_ip->GetSelNodeCount();
//		for (int i = 0; i < selectionCount; i ++) {
//			INode* const node = m_ip->GetSelNode(i);
//			m_currentSelection.Append(node);
//		}
//		NewtonRigidBodyInfoPane::SelectionHasChanged();
	}
}




void RigidBodyWorld::AttachRigiBodyController (INode* const node)
{
	RigidBodyControllerDesc& desc = *(RigidBodyControllerDesc*)RigidBodyControllerDesc::GetDescriptor();
	Control* const control = node->GetTMController();
	if (control->ClassID() != desc.ClassID()) {
		Matrix3 matrix (node->GetNodeTM (GetCOREInterface()->GetTime()));		

		RigidBodyData data;
		data.m_oldControlerID = control->ClassID();

		// this create the matrix controller but for some reason that I cannot explain I can no move the body with the navigation 
		RigidBodyController* const rigidBodyController = (RigidBodyController*)CreateInstance(desc.SuperClassID(), desc.ClassID());
		_ASSERTE (rigidBodyController);
		rigidBodyController->Init (data, node);

		node->SetTMController (rigidBodyController);
		_ASSERTE (node->GetTMController());
		_ASSERTE (node->GetTMController() == rigidBodyController);

		node->SetNodeTM(GetCOREInterface()->GetTime(), matrix);
	}
}


void RigidBodyWorld::DetachRigiBodyController (INode* const node)
{
	RigidBodyControllerDesc& desc = *(RigidBodyControllerDesc*)RigidBodyControllerDesc::GetDescriptor();
	RigidBodyController* const rigidBodyController = (RigidBodyController*) node->GetTMController();
	if (rigidBodyController->ClassID() == desc.ClassID()) {

		Matrix3 matrix (node->GetNodeTM (GetCOREInterface()->GetTime()));		
		Control* const control = (Control*) CreateInstance (desc.SuperClassID(), rigidBodyController->m_oldControlerID);
		_ASSERTE (control);

		node->SetTMController (control);
		_ASSERTE (node->GetTMController());
		_ASSERTE (node->GetTMController() == control);
		node->SetNodeTM(GetCOREInterface()->GetTime(), matrix);
	}
}


void RigidBodyWorld::UpdateViewPorts ()
{
	//GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
	GetCOREInterface()->ForceCompleteRedraw(); 
}

void RigidBodyWorld::UpdatePhysics ()
{
	RigidBodyWorldDesc* const desc = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	float timestep = 1.0f / desc->m_minFps;
	if (timestep > 1.0f / 60.0f) {
		timestep = 1.0f / 60.0f;
	}


	desc->m_updateRigidBodyMatrix = false;
	NewtonUpdate(desc->m_newton, timestep);
	TimeValue t (GetCOREInterface()->GetTime());

	float scale = 1.0f / float (GetMasterScale(UNITS_METERS));
	for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; body = NewtonWorldGetNextBody(desc->m_newton, body))	{
		dMatrix matrix;
		INode* const node = (INode*)NewtonBodyGetUserData(body);
		NewtonBodyGetMatrix(body, &matrix[0][0]);

		matrix = desc->m_systemMatrix * matrix * desc->m_systemMatrixInv;
		matrix.m_posit = matrix.m_posit.Scale (scale);

		Matrix3 maxMatrix (GetMatrixFromdMatrix (matrix));
		node->SetNodeTM(t, maxMatrix);
	}
	
	UpdateViewPorts ();

	desc->m_updateRigidBodyMatrix = true;
}

void RigidBodyWorld::StopsSimulation ()
{
	KillTimer(m_myWindow, TIMER_ID);
	Sleep (100);
	if (IsDlgButtonChecked(m_myWindow, IDC_PREVIEW_WORLD) == BST_CHECKED) {
		CheckDlgButton(m_myWindow, IDC_PREVIEW_WORLD, BST_UNCHECKED);
	}
}

void RigidBodyWorld::OnUndoRedo(void* param, NotifyInfo* info)
{
	RigidBodyWorld* const me = (RigidBodyWorld*) param;
	me->StopsSimulation();

}

void RigidBodyWorld::Undo() const
{
	TimeValue t (GetCOREInterface()->GetTime());

	theHold.Begin();  
	RigidBodyWorldDesc* const desc = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();
	for (NewtonBody* body = NewtonWorldGetFirstBody(desc->m_newton); body; body = NewtonWorldGetNextBody(desc->m_newton, body))	{
		INode* const node = (INode*)NewtonBodyGetUserData(body);

		RigidBodyController* const control = desc->GetRigidBodyControl(node);
		control->SaveUndoState(t);
	}
	theHold.Accept ("newton Undo");
}