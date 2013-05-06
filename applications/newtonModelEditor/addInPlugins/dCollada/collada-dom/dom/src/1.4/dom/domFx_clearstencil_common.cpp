/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_clearstencil_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_clearstencil_common::create(DAE& dae)
{
	domFx_clearstencil_commonRef ref = new domFx_clearstencil_common(dae);
	return ref;
}


daeMetaElement *
domFx_clearstencil_common::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_clearstencil_common" );
	meta->registerClass(domFx_clearstencil_common::create);

	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsByte"));
		ma->setOffset( daeOffsetOf( domFx_clearstencil_common , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( dae.getAtomicTypes().get("xsNonNegativeInteger"));
		ma->setOffset( daeOffsetOf( domFx_clearstencil_common , attrIndex ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_clearstencil_common));
	meta->validate();

	return meta;
}

