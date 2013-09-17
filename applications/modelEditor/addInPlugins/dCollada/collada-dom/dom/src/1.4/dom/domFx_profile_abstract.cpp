/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_profile_abstract.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_profile_abstract::create(DAE& dae)
{
	domFx_profile_abstractRef ref = new domFx_profile_abstract(dae);
	return ref;
}


daeMetaElement *
domFx_profile_abstract::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_profile_abstract" );
	meta->registerClass(domFx_profile_abstract::create);

	meta->setIsAbstract( true );

	meta->setElementSize(sizeof(domFx_profile_abstract));
	meta->validate();

	return meta;
}

