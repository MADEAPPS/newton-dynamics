#ifndef __DLG_MESSAGE_H__
#define __DLG_MESSAGE_H__



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



class cDlgMessage : public CDialog
{
public:
	cDlgMessage(CWnd* pParent = NULL);

public:
    void SetTitle (CString sTitle);
    void SetMessage (CString sMessage);
    void SetRange (int nRange);
    void SetPosition (int nPosition);

	//{{AFX_VIRTUAL(cDlgMessage)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);
	//}}AFX_VIRTUAL

protected:

	//{{AFX_MSG(cDlgMessage)
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}

#endif // __DLG_MESSAGE_H__
