#ifndef __MS_PLUGIN_IMPL_H__
#define __MS_PLUGIN_IMPL_H__



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif



#include "resource.h"
#include "msPlugIn.h"



/////////////////////////////////////////////////////////////////////////////
// CMsPlugInApp
//
//

class CMsPlugInApp : public CWinApp
{
public:
	CMsPlugInApp();

	//{{AFX_VIRTUAL(CMsPlugInApp)
	//}}AFX_VIRTUAL

	//{{AFX_MSG(CMsPlugInApp)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};



/////////////////////////////////////////////////////////////////////////////
// cPlugIn
//
//

struct msModel;
class cPlugIn : public cMsPlugIn
{
    char szTitle[64];

public:
	cPlugIn ();
    virtual ~cPlugIn ();

public:
    int             GetType ();
    const char *    GetTitle ();
    int             Execute (msModel* pModel);
};



/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ fügt unmittelbar vor der vorhergehenden Zeile zusätzliche Deklarationen ein.

#endif // __MS_PLUGIN_IMPL_H__