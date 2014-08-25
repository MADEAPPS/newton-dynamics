# - Find TinyXML
# Find the native TinyXML includes and library
#
#   TINYXML_FOUND       - True if TinyXML found.
#   TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
#   TINYXML_LIBRARIES   - List of libraries when using TinyXML.
#

IF( TinyXML_INCLUDE_DIR )
    # Already in cache, be silent
    SET( TinyXML_FIND_QUIETLY TRUE )
ENDIF( TinyXML_INCLUDE_DIR )

FIND_PATH( TinyXML_INCLUDE_DIR "tinyxml.h" PATH_SUFFIXES "tinyxml" )

FIND_LIBRARY( TinyXML_LIBRARIES NAMES "tinyxml" PATH_SUFFIXES "tinyxml" )

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( "FindPackageHandleStandardArgs" )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "TinyXML" DEFAULT_MSG TinyXML_INCLUDE_DIR TinyXML_LIBRARIES )

MARK_AS_ADVANCED( TinyXML_INCLUDE_DIR TinyXML_LIBRARIES )
