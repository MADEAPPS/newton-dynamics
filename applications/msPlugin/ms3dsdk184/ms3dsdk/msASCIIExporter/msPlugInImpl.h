#ifndef __MS_PLUGIN_IMPL_H__
#define __MS_PLUGIN_IMPL_H__



#include "msPlugIn.h"



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



#endif // __MS_PLUGIN_IMPL_H__