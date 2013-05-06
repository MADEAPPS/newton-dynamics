/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texcombiner_argumentAlpha_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texcombiner_argumentAlpha_type::create(DAE& dae)
{
	domGles_texcombiner_argumentAlpha_typeRef ref = new domGles_texcombiner_argumentAlpha_type(dae);
	return ref;
}


daeMetaElement *
domGles_texcombiner_argumentAlpha_type::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texcombiner_argumentAlpha_type" );
	meta->registerClass(domGles_texcombiner_argumentAlpha_type::create);


	//	Add attribute: source
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( dae.getAtomicTypes().get("Gles_texcombiner_source_enums"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrSource ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: operand
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "operand" );
		ma->setType( dae.getAtomicTypes().get("Gles_texcombiner_operandAlpha_enums"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrOperand ));
		ma->setContainer( meta );
		ma->setDefaultString( "SRC_ALPHA");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: unit
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "unit" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrUnit ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_texcombiner_argumentAlpha_type));
	meta->validate();

	return meta;
}

