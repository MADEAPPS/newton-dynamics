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
#include "Import.h"
#include "ImportDesc.h"


#define IMPORT_DESC_CLASS_ID Class_ID(0x38350d9d, 0x5a825aa9)

int ImportAlchemediaDesc::IsPublic() 
{ 
	return TRUE; 
}

void* ImportAlchemediaDesc::Create(BOOL loading) 
{ 
	return new AlchemediaMaxImport(); 
}

const TCHAR* ImportAlchemediaDesc::ClassName() 
{ 
	return GetString(IDS_CLASS_NAME); 
}

SClass_ID ImportAlchemediaDesc::SuperClassID() 
{ 
	return SCENE_IMPORT_CLASS_ID; 
}

Class_ID ImportAlchemediaDesc::ClassID() 
{ 
	return IMPORT_DESC_CLASS_ID; 
}

const TCHAR* ImportAlchemediaDesc::Category() 
{ 
	return GetString(IDS_CATEGORY); 
}

const TCHAR* ImportAlchemediaDesc::InternalName() 
{ 
	return _T("NewtonMaxImport"); 
}	

HINSTANCE ImportAlchemediaDesc::HInstance() 
{ 
	return hInstance; 
}


ClassDesc* ImportAlchemediaDesc::GetDescriptor() 
{
	static ImportAlchemediaDesc desc;
	return &desc; 
}



//--- AlchemediaMaxImport -------------------------------------------------------
AlchemediaMaxImport::AlchemediaMaxImport()
{

}

AlchemediaMaxImport::~AlchemediaMaxImport() 
{
}


SClass_ID AlchemediaMaxImport::SuperClassID()
{
	return ImportAlchemediaDesc::GetDescriptor()->SuperClassID(); 
}

Class_ID AlchemediaMaxImport::ClassID()
{
	return ImportAlchemediaDesc::GetDescriptor()->ClassID(); 
}


int AlchemediaMaxImport::ExtCount()
{
	//TODO: Returns the number of file name extensions supported by the plug-in.
	return 1;
}

const TCHAR *AlchemediaMaxImport::Ext(int n)
{		
	//TODO: Return the 'i-th' file name extension (i.e. "3DS").
	return _T(D_FILE_EXT);
}

const TCHAR *AlchemediaMaxImport::LongDesc()
{
	//TODO: Return long ASCII description (i.e. "Targa 2.0 Image File")
	return _T(D_LONG_DESCRIPTION);
}
	
const TCHAR *AlchemediaMaxImport::ShortDesc() 
{			
	//TODO: Return short ASCII description (i.e. "Targa")
	return _T(D_SHORT_DESCRIPTION);
}

const TCHAR *AlchemediaMaxImport::AuthorName()
{			
	//TODO: Return ASCII Author name
	return _T(D_AUTHOR_NAME);
}

const TCHAR *AlchemediaMaxImport::CopyrightMessage() 
{	
	// Return ASCII Copyright message
	return _T("");
}

const TCHAR *AlchemediaMaxImport::OtherMessage1() 
{		
	//TODO: Return Other message #1 if any
	return _T("");
}

const TCHAR *AlchemediaMaxImport::OtherMessage2() 
{		
	//TODO: Return other message #2 in any
	return _T("");
}

unsigned int AlchemediaMaxImport::Version()
{				
	//TODO: Return Version number * 100 (i.e. v3.01 = 301)
	return 100;
}

void AlchemediaMaxImport::ShowAbout(HWND hWnd)
{			
	// Optional
}

int AlchemediaMaxImport::DoImport(const TCHAR *filename, ImpInterface *i, Interface *gi, BOOL suppressPrompts)
{
	int ret;
	Import import (filename, gi, i);
	ret = import.m_succes;
	return ret;

}
	

