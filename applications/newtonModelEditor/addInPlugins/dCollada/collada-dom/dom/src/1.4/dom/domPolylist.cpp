/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domPolylist.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domPolylist::create(DAE& dae)
{
	domPolylistRef ref = new domPolylist(dae);
	return ref;
}


daeMetaElement *
domPolylist::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "polylist" );
	meta->registerClass(domPolylist::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domPolylist,elemInput_array) );
	mea->setElementType( domInputLocalOffset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "vcount" );
	mea->setOffset( daeOffsetOf(domPolylist,elemVcount) );
	mea->setElementType( domPolylist::domVcount::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "p" );
	mea->setOffset( daeOffsetOf(domPolylist,elemP) );
	mea->setElementType( domP::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domPolylist,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolylist , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: count
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domPolylist , attrCount ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: material
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "material" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolylist , attrMaterial ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domPolylist));
	meta->validate();

	return meta;
}

daeElementRef
domPolylist::domVcount::create(DAE& dae)
{
	domPolylist::domVcountRef ref = new domPolylist::domVcount(dae);
	return ref;
}


daeMetaElement *
domPolylist::domVcount::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "vcount" );
	meta->registerClass(domPolylist::domVcount::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("ListOfUInts"));
		ma->setOffset( daeOffsetOf( domPolylist::domVcount , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domPolylist::domVcount));
	meta->validate();

	return meta;
}

