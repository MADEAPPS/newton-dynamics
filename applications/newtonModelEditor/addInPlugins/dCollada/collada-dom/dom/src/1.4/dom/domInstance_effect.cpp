/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInstance_effect.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_effect::create(DAE& dae)
{
	domInstance_effectRef ref = new domInstance_effect(dae);
	return ref;
}


daeMetaElement *
domInstance_effect::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_effect" );
	meta->registerClass(domInstance_effect::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "technique_hint" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemTechnique_hint_array) );
	mea->setElementType( domInstance_effect::domTechnique_hint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemSetparam_array) );
	mea->setElementType( domInstance_effect::domSetparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: url
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrUrl ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_effect));
	meta->validate();

	return meta;
}

daeElementRef
domInstance_effect::domTechnique_hint::create(DAE& dae)
{
	domInstance_effect::domTechnique_hintRef ref = new domInstance_effect::domTechnique_hint(dae);
	return ref;
}


daeMetaElement *
domInstance_effect::domTechnique_hint::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique_hint" );
	meta->registerClass(domInstance_effect::domTechnique_hint::create);

	meta->setIsInnerClass( true );

	//	Add attribute: platform
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrPlatform ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: profile
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "profile" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrProfile ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_effect::domTechnique_hint));
	meta->validate();

	return meta;
}

daeElementRef
domInstance_effect::domSetparam::create(DAE& dae)
{
	domInstance_effect::domSetparamRef ref = new domInstance_effect::domSetparam(dae);
	return ref;
}


daeMetaElement *
domInstance_effect::domSetparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "setparam" );
	meta->registerClass(domInstance_effect::domSetparam::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fx_basic_type_common" );
	mea->setOffset( daeOffsetOf(domInstance_effect::domSetparam,elemFx_basic_type_common) );
	mea->setElementType( domFx_basic_type_common::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domSetparam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_effect::domSetparam));
	meta->validate();

	return meta;
}

