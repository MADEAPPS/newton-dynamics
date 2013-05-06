/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCg_sampler3D.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_sampler3D::create(DAE& dae)
{
	domCg_sampler3DRef ref = new domCg_sampler3D(dae);
	return ref;
}


daeMetaElement *
domCg_sampler3D::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cg_sampler3D" );
	meta->registerClass(domCg_sampler3D::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemSource) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "wrap_s" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemWrap_s) );
	mea->setElementType( domWrap_s::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "wrap_t" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemWrap_t) );
	mea->setElementType( domWrap_t::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "wrap_p" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemWrap_p) );
	mea->setElementType( domWrap_p::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "minfilter" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemMinfilter) );
	mea->setElementType( domMinfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "magfilter" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemMagfilter) );
	mea->setElementType( domMagfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "mipfilter" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemMipfilter) );
	mea->setElementType( domMipfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "border_color" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemBorder_color) );
	mea->setElementType( domBorder_color::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "mipmap_maxlevel" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemMipmap_maxlevel) );
	mea->setElementType( domMipmap_maxlevel::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "mipmap_bias" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemMipmap_bias) );
	mea->setElementType( domMipmap_bias::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 10, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCg_sampler3D,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 10 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 10 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCg_sampler3D));
	meta->validate();

	return meta;
}

