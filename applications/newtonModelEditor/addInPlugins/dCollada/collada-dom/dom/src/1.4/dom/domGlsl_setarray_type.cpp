/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGlsl_setarray_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGlsl_setarray_type::create(DAE& dae)
{
	domGlsl_setarray_typeRef ref = new domGlsl_setarray_type(dae);
	return ref;
}


daeMetaElement *
domGlsl_setarray_type::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "glsl_setarray_type" );
	meta->registerClass(domGlsl_setarray_type::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "glsl_param_type" );
	mea->setOffset( daeOffsetOf(domGlsl_setarray_type,elemGlsl_param_type_array) );
	mea->setElementType( domGlsl_param_type::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domGlsl_setarray_type,elemArray_array) );
	mea->setElementType( domGlsl_setarray_type::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGlsl_setarray_type,_contents));
	meta->addContentsOrder(daeOffsetOf(domGlsl_setarray_type,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGlsl_setarray_type,_CMData), 1);
	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsPositiveInteger"));
		ma->setOffset( daeOffsetOf( domGlsl_setarray_type , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_setarray_type));
	meta->validate();

	return meta;
}

