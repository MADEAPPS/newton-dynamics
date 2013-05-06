/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domMorph.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domMorph::create(DAE& dae)
{
	domMorphRef ref = new domMorph(dae);
	return ref;
}


daeMetaElement *
domMorph::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "morph" );
	meta->registerClass(domMorph::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 2, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domMorph,elemSource_array) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "targets" );
	mea->setOffset( daeOffsetOf(domMorph,elemTargets) );
	mea->setElementType( domMorph::domTargets::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domMorph,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: method
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "method" );
		ma->setType( dae.getAtomicTypes().get("MorphMethodType"));
		ma->setOffset( daeOffsetOf( domMorph , attrMethod ));
		ma->setContainer( meta );
		ma->setDefaultString( "NORMALIZED");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: source
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domMorph , attrSource ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domMorph));
	meta->validate();

	return meta;
}

daeElementRef
domMorph::domTargets::create(DAE& dae)
{
	domMorph::domTargetsRef ref = new domMorph::domTargets(dae);
	return ref;
}


daeMetaElement *
domMorph::domTargets::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "targets" );
	meta->registerClass(domMorph::domTargets::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 2, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domMorph::domTargets,elemInput_array) );
	mea->setElementType( domInputLocal::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domMorph::domTargets,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domMorph::domTargets));
	meta->validate();

	return meta;
}

