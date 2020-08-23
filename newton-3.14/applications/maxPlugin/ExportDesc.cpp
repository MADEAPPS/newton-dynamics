/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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
#include "Export.h"
#include "options.h"
#include "ExportDesc.h"


#define EXPORT_DESC_CLASS_ID Class_ID(0x69f52725, 0x75583868)


int ExportAlchemediaDesc::IsPublic() 
{ 
	return TRUE; 
}

void* ExportAlchemediaDesc::Create(BOOL loading) 
{ 
	return new AlchemediaMaxExport(); 
}

const TCHAR* ExportAlchemediaDesc::ClassName() 
{ 
	return GetString(IDS_CLASS_NAME); 
}

SClass_ID ExportAlchemediaDesc::SuperClassID() 
{ 
	return SCENE_EXPORT_CLASS_ID; 
}

Class_ID ExportAlchemediaDesc::ClassID() 
{ 
	return EXPORT_DESC_CLASS_ID; 
}

const TCHAR* ExportAlchemediaDesc::Category() 
{ 
	return GetString(IDS_CATEGORY); 
}

const TCHAR* ExportAlchemediaDesc::InternalName() 
{ 
	return _T("NewtonMaxExport"); 
}	

HINSTANCE ExportAlchemediaDesc::HInstance() 
{ 
	return hInstance; 
}			

ClassDesc* ExportAlchemediaDesc::GetDescriptor() 
{
	static ExportAlchemediaDesc desc;
	return &desc;
}







//--- ExportDesc -------------------------------------------------------
AlchemediaMaxExport::AlchemediaMaxExport()
{

}

AlchemediaMaxExport::~AlchemediaMaxExport() 
{

}

int AlchemediaMaxExport::ExtCount()
{
	//TODO: Returns the number of file name extensions supported by the plug-in.
	return 1;
}

const TCHAR *AlchemediaMaxExport::Ext(int n)
{		
	//TODO: Return the 'i-th' file name extension (i.e. "3DS").
/*
	switch (n) 
	{	
		case 0:
			return _T("mdl");

		case 1:
			return _T("msh");

		case 2:
			return _T("skel");

		case 3:
			return _T("ani");
	}
*/
	return _T(D_FILE_EXT); 
}

const TCHAR *AlchemediaMaxExport::LongDesc()
{
	//TODO: Return long ASCII description (i.e. "Targa 2.0 Image File")
	return _T(D_LONG_DESCRIPTION);
}

const TCHAR *AlchemediaMaxExport::ShortDesc() 
{			
	//TODO: Return short ASCII description (i.e. "Targa")
	return _T(D_SHORT_DESCRIPTION);
}

const TCHAR *AlchemediaMaxExport::AuthorName()
{			
	//TODO: Return ASCII Author name
	return _T(D_AUTHOR_NAME);
}

const TCHAR *AlchemediaMaxExport::CopyrightMessage() 
{	
	// Return ASCII Copyright message
	return _T("");
}

const TCHAR *AlchemediaMaxExport::OtherMessage1() 
{		
	//TODO: Return Other message #1 if any
	return _T("");
}

const TCHAR *AlchemediaMaxExport::OtherMessage2() 
{		
	//TODO: Return other message #2 in any
	return _T("");
}

unsigned int AlchemediaMaxExport::Version()
{				
	//TODO: Return Version number * 100 (i.e. v3.01 = 301)
	return 100;
}

void AlchemediaMaxExport::ShowAbout(HWND hWnd)
{			
	// Optional
}


int AlchemediaMaxExport::DoExport(const TCHAR *name,ExpInterface *ei,Interface *i, BOOL suppressPrompts, DWORD options)
{
	m_options.Load();
	if(!suppressPrompts)	{
		if (!DialogBoxParam(hInstance, MAKEINTRESOURCE(IDD_EXPORT_DIALOG), GetActiveWindow(), (DLGPROC) ExportDescOptionsDlgProc, (LPARAM)this)) {
			return TRUE;
		}
	}

	Export exportMesh (name, ei, i, m_options);

	m_options.Save();
	return TRUE;

}

SClass_ID AlchemediaMaxExport::SuperClassID()
{
	return ExportAlchemediaDesc::GetDescriptor()->SuperClassID(); 
}

Class_ID AlchemediaMaxExport::ClassID()
{
	return ExportAlchemediaDesc::GetDescriptor()->ClassID(); 
}


BOOL CALLBACK AlchemediaMaxExport::ExportDescOptionsDlgProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam) 
{
	switch(message) 
	{
		case WM_INITDIALOG:
		{
			AlchemediaMaxExport* const scene = (AlchemediaMaxExport *)lParam;
			SetWindowLong(hWnd, GWLP_USERDATA, (LONG)scene);

			CenterWindow(hWnd,GetParent(hWnd));

			CheckDlgButton(hWnd, IDC_EXPORT_MESH,	   scene->m_options.m_exportMesh); 
			CheckDlgButton(hWnd, IDC_EXPORT_MODEL,	   scene->m_options.m_exportModel); 
			CheckDlgButton(hWnd, IDC_EXPORT_SKELETON,  scene->m_options.m_exportSkeleton); 
			CheckDlgButton(hWnd, IDC_EXPORT_ANIMATION, scene->m_options.m_exportAnimation); 
			break;
		}

		case WM_COMMAND:
		{
			AlchemediaMaxExport* const scene = (AlchemediaMaxExport *)GetWindowLong (hWnd, GWLP_USERDATA);
			switch (LOWORD(wParam)) 
			{
				case IDC_EXPORT_MODEL:
				{
					scene->m_options.m_exportModel = IsDlgButtonChecked(hWnd, IDC_EXPORT_MODEL) ? 1 : 0;
					break;
				}

				case IDC_EXPORT_MESH:
				{
					scene->m_options.m_exportMesh = IsDlgButtonChecked(hWnd, IDC_EXPORT_MESH) ? 1 : 0;
					break;
				}

				case IDC_EXPORT_SKELETON:
				{
					scene->m_options.m_exportSkeleton = IsDlgButtonChecked(hWnd, IDC_EXPORT_SKELETON) ? 1 : 0;
					break;
				}


				case IDC_EXPORT_ANIMATION:
				{
					scene->m_options.m_exportAnimation = IsDlgButtonChecked(hWnd, IDC_EXPORT_ANIMATION) ? 1 : 0;
					break;
				}

				case IDOK:
				{
					EndDialog(hWnd, 1);
					break;
				}

				case IDCANCEL:
				{
					EndDialog(hWnd, 0);
					break;
				}
			}
			break;
		}

		case WM_CLOSE:
		{
			EndDialog(hWnd, 0);
			return TRUE;
		}
	}

	return FALSE;
}

