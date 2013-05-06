/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSpline.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSpline::create(DAE& dae)
{
	domSplineRef ref = new domSpline(dae);
	return ref;
}


daeMetaElement *
domSpline::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "spline" );
	meta->registerClass(domSpline::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domSpline,elemSource_array) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "control_vertices" );
	mea->setOffset( daeOffsetOf(domSpline,elemControl_vertices) );
	mea->setElementType( domSpline::domControl_vertices::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSpline,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: closed
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed" );
		ma->setType( dae.getAtomicTypes().get("Bool"));
		ma->setOffset( daeOffsetOf( domSpline , attrClosed ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSpline));
	meta->validate();

	return meta;
}

daeElementRef
domSpline::domControl_vertices::create(DAE& dae)
{
	domSpline::domControl_verticesRef ref = new domSpline::domControl_vertices(dae);
	return ref;
}


daeMetaElement *
domSpline::domControl_vertices::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "control_vertices" );
	meta->registerClass(domSpline::domControl_vertices::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domSpline::domControl_vertices,elemInput_array) );
	mea->setElementType( domInputLocal::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSpline::domControl_vertices,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domSpline::domControl_vertices));
	meta->validate();

	return meta;
}

