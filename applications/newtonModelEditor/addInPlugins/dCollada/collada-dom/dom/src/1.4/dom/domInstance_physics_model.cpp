/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInstance_physics_model.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_physics_model::create(DAE& dae)
{
	domInstance_physics_modelRef ref = new domInstance_physics_model(dae);
	return ref;
}


daeMetaElement *
domInstance_physics_model::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_physics_model" );
	meta->registerClass(domInstance_physics_model::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "instance_force_field" );
	mea->setOffset( daeOffsetOf(domInstance_physics_model,elemInstance_force_field_array) );
	mea->setElementType( domInstance_force_field::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "instance_rigid_body" );
	mea->setOffset( daeOffsetOf(domInstance_physics_model,elemInstance_rigid_body_array) );
	mea->setElementType( domInstance_rigid_body::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "instance_rigid_constraint" );
	mea->setOffset( daeOffsetOf(domInstance_physics_model,elemInstance_rigid_constraint_array) );
	mea->setElementType( domInstance_rigid_constraint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_physics_model,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: url
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrUrl ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: parent
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "parent" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrParent ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_physics_model));
	meta->validate();

	return meta;
}

