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
#include "RigidBodyUIPane.h"
#include "RigidBodyController.h"



RigidBodyUIPane::RigidBodyUIPane()
{
}

RigidBodyUIPane::~RigidBodyUIPane()
{
}


void RigidBodyUIPane::InitUI(HWND hWnd)
{
	HWND massControlHandle = GetDlgItem(hWnd, IDC_RIGID_BODY_MASS);
	HWND massSpinControlHandle = GetDlgItem(hWnd, IDC_RIGID_BODY_MASS_PINNER);
	HWND collisionShapeHandle = GetDlgItem(hWnd, IDC_RIGID_BODY_COLLISION);

	m_massEdit = GetICustEdit(massControlHandle);
	m_massSpinner = GetISpinner (massSpinControlHandle);
	m_massSpinner->LinkToEdit (massControlHandle, EDITTYPE_POS_FLOAT);
	m_massSpinner->SetScale (0.1f);
	m_massSpinner->SetLimits(0.0f, 1000.0f);
/*
	SetCollisionData (collisionShapeHandle, m_box, "box");
	SetCollisionData (collisionShapeHandle, m_sphere, "sphere");
	SetCollisionData (collisionShapeHandle, m_cylinder, "cylinder");
	SetCollisionData (collisionShapeHandle, m_convexHull, "convex hull");
	SetCollisionData (collisionShapeHandle, m_collisionTree, "collision tree");

	//SendMessage (collisionShapeHandle, CB_SELECTSTRING, 0, LPARAM ("convex hull"));
	SendMessage (collisionShapeHandle, CB_SELECTSTRING, 0, LPARAM ("box"));
	m_curCollsionShape = m_box;

	PhysicsPluginObject& me = *(PhysicsPluginObject*)this;
	dList<INode*> nodeList;
	me.GetNodeList (nodeList);
	for (dList<INode*>::dListNode* ptr = nodeList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		INode* const node = ptr->GetInfo();
		AppDataChunk* const rigidBodyChunk = node->GetAppDataChunk(GetRigidBodyID(), node->SuperClassID(), 0); 
		if (rigidBodyChunk) {
			RidBodyData* const bodyData = (RidBodyData*) rigidBodyChunk->data;
			bodyData->m_body = NULL;
			AddNodeToWorld (node);
		}
	}
*/
}

void RigidBodyUIPane::DestroyUI(HWND hWnd)
{

	ReleaseICustEdit (m_massEdit);
	ReleaseISpinner (m_massSpinner);
}





INT_PTR CALLBACK RigidBodyUIPane::Proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	RigidBodyWorld* const world = (RigidBodyWorld *)GetWindowLong (hWnd, GWLP_USERDATA);
	RigidBodyWorldDesc* const desc = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();

	switch (msg) 
	{
		case WM_INITDIALOG:
		{
			RigidBodyWorld* const world = (RigidBodyWorld *)lParam;
			SetWindowLong(hWnd, GWLP_USERDATA, (LONG)world);
			world->RigidBodyUIPane::InitUI(hWnd);
			break;
		}
		
		case WM_DESTROY:
		{
			world->RigidBodyUIPane::DestroyUI(hWnd);
			break;
		}

		case WM_ENABLE:
		{
			//EnableWindow(obj->m_worldPaneHandle, (BOOL) wParam);
			break;
		}

		case CC_SPINNER_CHANGE:
		{
			switch (LOWORD(wParam))
			{
				case IDC_RIGID_BODY_MASS_PINNER:
				{
					MCHAR text [256];
					world->m_massEdit->GetText (text, sizeof (text));	
					if (text[0] != '?') {
						float mass = world->m_massEdit->GetFloat();
						world->m_massEdit->SetText(mass, 1);
						world->SetSelectionMass (mass);
					}
					break;
				}
			}
			return TRUE;
		}


		case WM_COMMAND:
		{

			case IDC_RIGID_BODY_MASS:
			{
				MCHAR text [256];
				world->m_massEdit->GetText (text, sizeof (text));	
				if (text[0] != '?') {
					float mass = world->m_massEdit->GetFloat();
					world->SetSelectionMass (mass);
				}
				break;
			}


			switch (LOWORD(wParam)) 
			{
				case IDC_HIDE_GIZMO:
				{
					Interface* const ip = GetCOREInterface();
					int selectionCount = ip->GetSelNodeCount();
					for (int i = 0; i < selectionCount; i ++) {
						INode* const node = ip->GetSelNode(i);
						RigidBodyController* const bodyInfo = desc->GetRigidBodyControl(node);
						if (bodyInfo) {
							bodyInfo->m_hideGizmos = (IsDlgButtonChecked(hWnd, IDC_HIDE_GIZMO) == BST_CHECKED) ? TRUE : FALSE;
						}
					}

					world->UpdateViewPorts();
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


void RigidBodyUIPane::SetSelectionMass (dFloat mass)
{
	RigidBodyWorldDesc* const plugin = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();
	Interface* const ip = GetCOREInterface();
	int selectionCount = ip->GetSelNodeCount();
	for (int i = 0; i < selectionCount; i ++) {
		INode* const node = ip->GetSelNode(i);
		RigidBodyController* const bodyInfo = (RigidBodyController*)plugin->GetRigidBodyControl(node);
		if (bodyInfo) {
			bodyInfo->m_mass = mass;
			_ASSERTE (bodyInfo->m_body);

//			dVector inertia (bodyInfo->m_inertia.Scale (mass));
//			NewtonBodySetMassMatrix(bodyInfo->m_body, mass, inertia.m_x, inertia.m_y, inertia.m_z);

			NewtonCollisionInfoRecord info;
			NewtonCollision* const collision = NewtonBodyGetCollision(bodyInfo->m_body);
			NewtonCollisionGetInfo (collision, &info);

			switch (info.m_collisionType)
			{
				case SERIALIZE_ID_BOX:
				case SERIALIZE_ID_CONE:
				case SERIALIZE_ID_SPHERE:
				case SERIALIZE_ID_CAPSULE:
				case SERIALIZE_ID_CYLINDER:
				case SERIALIZE_ID_COMPOUND:
				case SERIALIZE_ID_CONVEXHULL:
				case SERIALIZE_ID_CHAMFERCYLINDER:
				{
					NewtonConvexCollisionCalculateInertialMatrix (collision, &bodyInfo->m_inertia[0], &bodyInfo->m_origin[0]);	

					NewtonBodySetCentreOfMass(bodyInfo->m_body, &bodyInfo->m_origin[0]);
					NewtonBodySetMassProperties(bodyInfo->m_body, bodyInfo->m_mass, collision);
				}
			}
		}
	}
}

void RigidBodyUIPane::SelectionSetChanged ()
{
	RigidBodyWorldDesc* const plugin = (RigidBodyWorldDesc*) RigidBodyWorldDesc::GetDescriptor();
	Interface* const ip = GetCOREInterface();
	int selectionCount = ip->GetSelNodeCount();

	RigidBodyWorld& me = *(RigidBodyWorld*)this;

	dList<RigidBodyController*> selectionList;
	for (int i = 0; i < selectionCount; i ++) {
		INode* const node = ip->GetSelNode(i);
		RigidBodyController* const info = plugin->GetRigidBodyControl(node);
		if (info) {
			selectionList.Append (info);
		}
	}

	if (selectionList.GetCount()) {
		dList<RigidBodyController*>::dListNode* first = selectionList.GetFirst(); 
		bool showMass = true;
		RigidBodyController* const control = first->GetInfo();
		for (dList<RigidBodyController*>::dListNode* ptr = first->GetNext(); ptr; ptr = ptr->GetNext()) {
			RigidBodyController* const control1 = ptr->GetInfo();
			if (control->m_mass != control1->m_mass) {
				showMass = false;
			}
			
		}

		if (showMass) {
			m_massEdit->SetText(control->m_mass);
		} else {
			m_massEdit->SetText("??????");
		}
		SendMessage (me.m_newtonBodyUI, WM_COMMAND, 0, 0);
		
	} 
}