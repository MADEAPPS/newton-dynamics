/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeDom.h>
#include <dae/daeMetaElement.h>
#include <dom.h>

daeMetaElement* initializeDomMeta(DAE& dae)
{
	registerDomTypes(dae);
	return registerDomElements(dae);
}
