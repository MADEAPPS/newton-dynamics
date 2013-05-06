/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCylinder.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCylinder::create(DAE& dae)
{
	domCylinderRef ref = new domCylinder(dae);
	return ref;
}


daeMetaElement *
domCylinder::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cylinder" );
	meta->registerClass(domCylinder::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "height" );
	mea->setOffset( daeOffsetOf(domCylinder,elemHeight) );
	mea->setElementType( domCylinder::domHeight::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "radius" );
	mea->setOffset( daeOffsetOf(domCylinder,elemRadius) );
	mea->setElementType( domCylinder::domRadius::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCylinder,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCylinder));
	meta->validate();

	return meta;
}

daeElementRef
domCylinder::domHeight::create(DAE& dae)
{
	domCylinder::domHeightRef ref = new domCylinder::domHeight(dae);
	return ref;
}


daeMetaElement *
domCylinder::domHeight::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "height" );
	meta->registerClass(domCylinder::domHeight::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCylinder::domHeight , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCylinder::domHeight));
	meta->validate();

	return meta;
}

daeElementRef
domCylinder::domRadius::create(DAE& dae)
{
	domCylinder::domRadiusRef ref = new domCylinder::domRadius(dae);
	return ref;
}


daeMetaElement *
domCylinder::domRadius::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "radius" );
	meta->registerClass(domCylinder::domRadius::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCylinder::domRadius , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCylinder::domRadius));
	meta->validate();

	return meta;
}

