#include "stdafx.h"
#include "resource.h"
#include "DlgOptions.h"



#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



//======================================================================
// constructor
//======================================================================
cDlgOptions::cDlgOptions(CWnd* pParent /*=NULL*/)
	: CDialog(IDD_OPTIONS, pParent)
{
	//{{AFX_DATA_INIT(cDlgOptions)
	//}}AFX_DATA_INIT
}

//======================================================================
// DoDataExchange
//======================================================================
void cDlgOptions::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(cDlgOptions)
	//}}AFX_DATA_MAP
}



//======================================================================
// Message Map
//======================================================================
BEGIN_MESSAGE_MAP(cDlgOptions, CDialog)
	//{{AFX_MSG_MAP(cDlgOptions)
    ON_BN_CLICKED(IDC_BTBROWSE,     OnBrowse)
    ON_BN_CLICKED(IDC_BTMATERIALS,  OnMaterials)
    ON_BN_CLICKED(IDC_BTBONES,      OnBones)
	ON_WM_CLOSE()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()



//======================================================================
// OnInitDialog
//======================================================================
BOOL cDlgOptions::OnInitDialog() 
{
	CDialog::OnInitDialog();
    CDialog::CenterWindow();

    m_nFlags = eMeshes | eMaterials | eBones | eKeyFrames;
    m_sPathName.Empty();

    CheckDlgButton (IDC_BTMESHES, TRUE);
    CheckDlgButton (IDC_BTMATERIALS, TRUE);
    CheckDlgButton (IDC_BTNOTRANSPARENCY, FALSE);
    CheckDlgButton (IDC_BTBONES, TRUE);
    CheckDlgButton (IDC_BTKEYFRAMES, TRUE);
	
	return TRUE;
}

//======================================================================
// OnClose
//======================================================================
void cDlgOptions::OnClose() 
{
	OnCancel ();
}

//======================================================================
// OnCancel
//======================================================================
void cDlgOptions::OnCancel() 
{
	CDialog::OnCancel();
}

//======================================================================
// OnOK
//======================================================================
void cDlgOptions::OnOK() 
{
    GetDlgItemText (IDC_FILENAME, m_sPathName);

    m_nFlags = 0;
    if (IsDlgButtonChecked (IDC_BTMESHES))
        m_nFlags |= eMeshes;
    if (IsDlgButtonChecked (IDC_BTMATERIALS))
        m_nFlags |= eMaterials;
    if (IsDlgButtonChecked (IDC_BTNOTRANSPARENCY))
        m_nFlags |= eNoTransparency;
    if (IsDlgButtonChecked (IDC_BTBONES))
        m_nFlags |= eBones;
    if (IsDlgButtonChecked (IDC_BTKEYFRAMES))
        m_nFlags |= eKeyFrames;

	CDialog::OnOK();
}

//======================================================================
// OnBrowse
//======================================================================
void cDlgOptions::OnBrowse() 
{
    //
    // choose filename
    //
    CFileDialog fileDlg (TRUE, "txt", NULL, OFN_HIDEREADONLY | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST,
                         "MilkShape 3D ASCII Files (*.txt)|*.txt|All Files (*.*)|*.*||");
    if (!fileDlg.DoModal ())
        m_sPathName.Empty ();
    else
        m_sPathName = fileDlg.GetPathName ();

    SetDlgItemText (IDC_FILENAME, m_sPathName);
}

//======================================================================
// OnBones
//======================================================================
void cDlgOptions::OnBones() 
{
    if (IsDlgButtonChecked (IDC_BTBONES))
    {
        GetDlgItem (IDC_BTKEYFRAMES)->EnableWindow (TRUE);
    }
    else
    {
        GetDlgItem (IDC_BTKEYFRAMES)->EnableWindow (FALSE);
        CheckDlgButton (IDC_BTKEYFRAMES, FALSE);
    }
}

//======================================================================
// OnMaterials
//======================================================================
void cDlgOptions::OnMaterials() 
{
    if (IsDlgButtonChecked (IDC_BTMATERIALS))
    {
        GetDlgItem (IDC_BTNOTRANSPARENCY)->EnableWindow (TRUE);
    }
    else
    {
        GetDlgItem (IDC_BTNOTRANSPARENCY)->EnableWindow (FALSE);
        CheckDlgButton (IDC_BTNOTRANSPARENCY, FALSE);
    }
}

//======================================================================
// GetPathName
//======================================================================
CString cDlgOptions::GetPathName() 
{
    return m_sPathName;
}

//======================================================================
// GetPathName
//======================================================================
int cDlgOptions::GetOptionFlags() 
{
    return m_nFlags;
}
