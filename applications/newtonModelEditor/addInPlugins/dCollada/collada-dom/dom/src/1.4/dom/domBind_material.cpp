/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domBind_material.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domBind_material::create(DAE& dae)
{
	domBind_materialRef ref = new domBind_material(dae);
	return ref;
}


daeMetaElement *
domBind_material::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_material" );
	meta->registerClass(domBind_material::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domBind_material,elemParam_array) );
	mea->setElementType( domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domBind_material,elemTechnique_common) );
	mea->setElementType( domBind_material::domTechnique_common::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domBind_material,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domBind_material,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domBind_material));
	meta->validate();

	return meta;
}

daeElementRef
domBind_material::domTechnique_common::create(DAE& dae)
{
	domBind_material::domTechnique_commonRef ref = new domBind_material::domTechnique_common(dae);
	return ref;
}


daeMetaElement *
domBind_material::domTechnique_common::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique_common" );
	meta->registerClass(domBind_material::domTechnique_common::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "instance_material" );
	mea->setOffset( daeOffsetOf(domBind_material::domTechnique_common,elemInstance_material_array) );
	mea->setElementType( domInstance_material::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domBind_material::domTechnique_common));
	meta->validate();

	return meta;
}

