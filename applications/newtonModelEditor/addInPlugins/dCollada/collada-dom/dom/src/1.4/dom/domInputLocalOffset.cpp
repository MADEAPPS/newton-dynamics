/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInputLocalOffset.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInputLocalOffset::create(DAE& dae)
{
	domInputLocalOffsetRef ref = new domInputLocalOffset(dae);
	return ref;
}


daeMetaElement *
domInputLocalOffset::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "InputLocalOffset" );
	meta->registerClass(domInputLocalOffset::create);


	//	Add attribute: offset
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "offset" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domInputLocalOffset , attrOffset ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domInputLocalOffset , attrSemantic ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: source
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( dae.getAtomicTypes().get("URIFragmentType"));
		ma->setOffset( daeOffsetOf( domInputLocalOffset , attrSource ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: set
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "set" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domInputLocalOffset , attrSet ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInputLocalOffset));
	meta->validate();

	return meta;
}

