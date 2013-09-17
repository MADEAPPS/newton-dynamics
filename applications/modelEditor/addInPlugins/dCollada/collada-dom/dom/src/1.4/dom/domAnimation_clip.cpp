/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domAnimation_clip.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domAnimation_clip::create(DAE& dae)
{
	domAnimation_clipRef ref = new domAnimation_clip(dae);
	return ref;
}


daeMetaElement *
domAnimation_clip::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "animation_clip" );
	meta->registerClass(domAnimation_clip::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domAnimation_clip,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 1, -1 );
	mea->setName( "instance_animation" );
	mea->setOffset( daeOffsetOf(domAnimation_clip,elemInstance_animation_array) );
	mea->setElementType( domInstanceWithExtra::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domAnimation_clip,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: start
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "start" );
		ma->setType( dae.getAtomicTypes().get("xsDouble"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrStart ));
		ma->setContainer( meta );
		ma->setDefaultString( "0.0");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: end
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "end" );
		ma->setType( dae.getAtomicTypes().get("xsDouble"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrEnd ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAnimation_clip));
	meta->validate();

	return meta;
}

