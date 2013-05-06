#ifndef __DLG_OPTIONS_H__
#define __DLG_OPTIONS_H__



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


//
// option flags
//
enum {
    eMeshes         = 1,
    eMaterials      = 2,
    eBones          = 4,
    eKeyFrames      = 8,
    eNoTransparency = 16
};



class cDlgOptions : public CDialog
{
private:
    int     m_nFlags;
    CString m_sPathName;

public:
	cDlgOptions(CWnd* pParent = NULL);

public:
    CString GetPathName ();
    int GetOptionFlags ();

	//{{AFX_VIRTUAL(cDlgOptions)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);
	//}}AFX_VIRTUAL

protected:

	//{{AFX_MSG(cDlgOptions)
	virtual BOOL OnInitDialog();
	afx_msg void OnClose();
	virtual void OnCancel();
	virtual void OnOK();
    virtual void OnBrowse();
    virtual void OnMaterials();
    virtual void OnBones();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}

#endif // __DLG_OPTIONS_H__
