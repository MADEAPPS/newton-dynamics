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
#include "options.h"



Options::Options()
{
	m_exportMesh = 1;
	m_exportMesh = 1;
	m_exportAnimation = 1;
	m_exportSkeleton = 1;
}

void Options::Load ()
{
	char* ptr;
	char pathName[256];
	const TiXmlElement* root;

	GetModuleFileName(NULL, pathName, sizeof (pathName));
	ptr = strrchr (pathName, '\\');
	*ptr = 0;
	strcat (pathName, "\\plugins\\NewtonOptions.xml");
	
	TiXmlDocument doc (pathName);
	doc.LoadFile();
	
	root = doc.RootElement();
	if (root){
		int exportModel;
		int exportMesh;
		int exportAnim;
		int exportSkel;

		if (root->Attribute ("exportModel", &exportModel)) {
			m_exportModel = exportModel;
		}

		if (root->Attribute ("exportMesh", &exportMesh)) {
			m_exportMesh = exportMesh;
		}
		if (root->Attribute ("exportAnimation", &exportAnim)) {
			m_exportAnimation = exportAnim;
		}

		if (root->Attribute ("exportSkeleton", &exportSkel)) {
			m_exportSkeleton = exportSkel;
		}

	}
}

void Options::Save ()
{
	char* ptr;
	char pathName[256];
	TiXmlDeclaration* decl;
	TiXmlElement* options;

	GetModuleFileName(NULL, pathName, sizeof (pathName));
	ptr = strrchr (pathName, '\\');
	*ptr = 0;
	strcat (pathName, "\\plugins\\NewtonOptions.xml");

	TiXmlDocument out (pathName);
	decl = new TiXmlDeclaration( "1.0", "", "" );

	out.LinkEndChild( decl );

	options = new TiXmlElement( "Options" );
	options->SetAttribute("exportModel", m_exportModel);
	options->SetAttribute("exportMesh", m_exportMesh);
	options->SetAttribute("exportAnimation", m_exportAnimation);
	options->SetAttribute("exportSkeleton", m_exportSkeleton);
	out.LinkEndChild(options);

	out.SaveFile (pathName);
}


