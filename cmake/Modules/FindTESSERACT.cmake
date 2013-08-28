# cmake macro to test if we use TESSERACT
#
#  TESSERACT_FOUND - system has TESSERACT
#  TESSERACT_INCLUDE_DIR - the TESSERACT include directory
#  TESSERACT_LIBRARIES - The libraries needed to use TESSERACT

find_path(TESSERACT_INCLUDE_DIR tesseract/baseapi.h
  PATHS
  /usr/include
  /usr/local/include)

FIND_LIBRARY(TESSERACT_LIBRARY NAMES tesseract_full tesseract libtesseract
   PATHS
   /usr/lib
   /usr/local/lib
)

if(TESSERACT_INCLUDE_DIR AND TESSERACT_LIBRARY)
   set(TESSERACT_FOUND TRUE)
   set(TESSERACT_LIBRARIES ${TESSERACT_LIBRARY})
else()
   set(TESSERACT_FOUND FALSE)
endif()

if (TESSERACT_FOUND)
   if (NOT Tesseract_FIND_QUIETLY)
      message(STATUS "Found Tesseract: ${TESSERACT_LIBRARIES}")
   endif (NOT Tesseract_FIND_QUIETLY)
else (TESSERACT_FOUND)
   if (NOT Tesseract_FIND_QUIETLY)

 message(STATUS "Tesseract not found")

   endif (NOT Tesseract_FIND_QUIETLY)
endif (TESSERACT_FOUND)

MARK_AS_ADVANCED(TESSERACT_INCLUDE_DIR TESSERACT_LIBRARIES TESSERACT_LIBRARY)
