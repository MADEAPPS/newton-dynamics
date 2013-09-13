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
#include "iparamm2.h"
#include "Simpobj.h"

class IObjParam;
class ConvexApproximationObject;



class ConvexApproximationMouseCallBack: public CreateMouseCallBack
{
	public:
	ConvexApproximationMouseCallBack()
	{
	}
	virtual ~ConvexApproximationMouseCallBack() 
	{
	}
	
	virtual int override(int mode) { return mode; }
	virtual BOOL StartNewCreation() { return TRUE; }
	virtual BOOL TolerateOrthoMode() {	return FALSE; }
	virtual int proc (ViewExp *vpt, int msg, int point, int flags, IPoint2 m, Matrix3& mat);

	ConvexApproximationObject* m_obj;
};


class ConvexApproximationClassDesc: public ClassDesc2 
{
	public:
	ConvexApproximationClassDesc();
	int	IsPublic();
	void* Create(BOOL loading = FALSE);
	const TCHAR* ClassName();
	SClass_ID SuperClassID();
	Class_ID ClassID();
	const TCHAR* Category();
	const TCHAR* InternalName();
	HINSTANCE HInstance();

	BOOL OkToCreate(Interface *i);
	static ClassDesc* GetDescriptor();

	static DWORD WINAPI ReportMaxProgress(LPVOID arg);
	static bool ReportProgress(float progressNormalzedPercent);

	int m_progress; 
	ConvexApproximationMouseCallBack m_mouseCallback;
	INode* m_sourceNode;
	Interface* m_currentInterface;
};


class ConvexApproximationObject : public PolyObject
{
	public:
	ConvexApproximationObject(BOOL loading);

	static INT_PTR CALLBACK Proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
	void BeginEditParams( IObjParam *ip, ULONG flags,Animatable *prev);
	void EndEditParams( IObjParam *ip, ULONG flags,Animatable *next);

	void InitUI(HWND hWnd);
	void DestroyUI(HWND hWnd);
	virtual CreateMouseCallBack* GetCreateMouseCallBack();

	void BuildMesh();

	void LoadGeometries (NewtonMesh* const meshOut, const dMatrix& matrix);
	PolyObject* GetPolyObject (ObjectState* const os, int& deleteIt);
	void AddPolygonFromObject(INode* const node, ObjectState* const os, NewtonMesh* const meshOut, const dMatrix& matrix);

	HWND m_createParameterUI;

	ICustEdit* m_maxConcavity;
	ISpinnerControl* m_maxConcavitySpinner;

	ICustEdit* m_maxSegments;
	ISpinnerControl* m_maxSegmentsSpinner;

	int m_currentMaxCount;
	float m_currentConcavity;

	bool m_UI_initalizationCompleted;
	
};				
