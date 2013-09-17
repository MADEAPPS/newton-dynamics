/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGl_hook_abstract.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGl_hook_abstract::create(DAE& dae)
{
	domGl_hook_abstractRef ref = new domGl_hook_abstract(dae);
	return ref;
}


daeMetaElement *
domGl_hook_abstract::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gl_hook_abstract" );
	meta->registerClass(domGl_hook_abstract::create);

	meta->setIsAbstract( true );

	meta->setElementSize(sizeof(domGl_hook_abstract));
	meta->validate();

	return meta;
}

