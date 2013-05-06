/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_surface_init_from_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_init_from_common::create(DAE& dae)
{
	domFx_surface_init_from_commonRef ref = new domFx_surface_init_from_common(dae);
	return ref;
}


daeMetaElement *
domFx_surface_init_from_common::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_surface_init_from_common" );
	meta->registerClass(domFx_surface_init_from_common::create);

	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: mip
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrMip ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: slice
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "slice" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrSlice ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: face
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "face" );
		ma->setType( dae.getAtomicTypes().get("Fx_surface_face_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrFace ));
		ma->setContainer( meta );
		ma->setDefaultString( "POSITIVE_X");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_surface_init_from_common));
	meta->validate();

	return meta;
}

