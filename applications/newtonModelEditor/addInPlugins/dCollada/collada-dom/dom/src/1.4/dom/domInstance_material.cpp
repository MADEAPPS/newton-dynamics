/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInstance_material.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_material::create(DAE& dae)
{
	domInstance_materialRef ref = new domInstance_material(dae);
	return ref;
}


daeMetaElement *
domInstance_material::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_material" );
	meta->registerClass(domInstance_material::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "bind" );
	mea->setOffset( daeOffsetOf(domInstance_material,elemBind_array) );
	mea->setElementType( domInstance_material::domBind::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "bind_vertex_input" );
	mea->setOffset( daeOffsetOf(domInstance_material,elemBind_vertex_input_array) );
	mea->setElementType( domInstance_material::domBind_vertex_input::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_material,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: target
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrTarget ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_material));
	meta->validate();

	return meta;
}

daeElementRef
domInstance_material::domBind::create(DAE& dae)
{
	domInstance_material::domBindRef ref = new domInstance_material::domBind(dae);
	return ref;
}


daeMetaElement *
domInstance_material::domBind::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind" );
	meta->registerClass(domInstance_material::domBind::create);

	meta->setIsInnerClass( true );

	//	Add attribute: semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind , attrSemantic ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: target
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind , attrTarget ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_material::domBind));
	meta->validate();

	return meta;
}

daeElementRef
domInstance_material::domBind_vertex_input::create(DAE& dae)
{
	domInstance_material::domBind_vertex_inputRef ref = new domInstance_material::domBind_vertex_input(dae);
	return ref;
}


daeMetaElement *
domInstance_material::domBind_vertex_input::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_vertex_input" );
	meta->registerClass(domInstance_material::domBind_vertex_input::create);

	meta->setIsInnerClass( true );

	//	Add attribute: semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind_vertex_input , attrSemantic ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: input_semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "input_semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind_vertex_input , attrInput_semantic ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: input_set
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "input_set" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind_vertex_input , attrInput_set ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_material::domBind_vertex_input));
	meta->validate();

	return meta;
}

