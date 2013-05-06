#include "stdafx.h"
#include "resource.h"
#include "DlgMessage.h"



#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



//======================================================================
// constructor
//======================================================================
cDlgMessage::cDlgMessage(CWnd* pParent /*=NULL*/)
	: CDialog(IDD_MESSAGE, pParent)
{
	//{{AFX_DATA_INIT(cDlgMessage)
	//}}AFX_DATA_INIT
}

//======================================================================
// DoDataExchange
//======================================================================
void cDlgMessage::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(cDlgMessage)
	//}}AFX_DATA_MAP
}



//======================================================================
// SetPosition
//======================================================================
BEGIN_MESSAGE_MAP(cDlgMessage, CDialog)
	//{{AFX_MSG_MAP(cDlgMessage)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()



//======================================================================
// OnInitDialog
//======================================================================
BOOL cDlgMessage::OnInitDialog() 
{
	CDialog::OnInitDialog();
    CDialog::CenterWindow();
	
	return TRUE;
}

//======================================================================
// SetTitle
//======================================================================
void cDlgMessage::SetTitle (CString sTitle)
{
    SetWindowText (sTitle);
}

//======================================================================
// SetMessage
//======================================================================
void cDlgMessage::SetMessage (CString sMessage)
{
    CWnd *pWnd = GetDlgItem (IDC_STMESSAGE);
    pWnd->SetWindowText (sMessage);
}

//======================================================================
// SetRange
//======================================================================
void cDlgMessage::SetRange (int nRange)
{
    CProgressCtrl *pPC = (CProgressCtrl *) GetDlgItem (IDC_PROGRESS);
    pPC->SetRange (0, nRange);
}

//======================================================================
// SetPosition
//======================================================================
void cDlgMessage::SetPosition (int nPosition)
{
    CProgressCtrl *pPC = (CProgressCtrl *) GetDlgItem (IDC_PROGRESS);
    pPC->SetPos (nPosition);
}
