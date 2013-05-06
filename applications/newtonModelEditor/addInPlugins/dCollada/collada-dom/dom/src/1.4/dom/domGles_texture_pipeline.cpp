/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texture_pipeline.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texture_pipeline::create(DAE& dae)
{
	domGles_texture_pipelineRef ref = new domGles_texture_pipeline(dae);
	return ref;
}


daeMetaElement *
domGles_texture_pipeline::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texture_pipeline" );
	meta->registerClass(domGles_texture_pipeline::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "texcombiner" );
	mea->setOffset( daeOffsetOf(domGles_texture_pipeline,elemTexcombiner_array) );
	mea->setElementType( domGles_texcombiner_command_type::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "texenv" );
	mea->setOffset( daeOffsetOf(domGles_texture_pipeline,elemTexenv_array) );
	mea->setElementType( domGles_texenv_command_type::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGles_texture_pipeline,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles_texture_pipeline,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles_texture_pipeline,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles_texture_pipeline,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_pipeline , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_texture_pipeline));
	meta->validate();

	return meta;
}

