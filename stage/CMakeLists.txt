SET( STAGE_LIBRARY_DIRS ../../stg/lib64 )
SET( STAGE_INCLUDE_DIRS ../../stg/include/Stage-4.1 )
SET( RAPI_DIR ../../autolab-rapi )
SET( EXECUTABLE_NAME orbittarget )

ADD_DEFINITIONS(-DSTAGE)
    INCLUDE_DIRECTORIES(
      ${RAPI_DIR}/librapicore
      ${RAPI_DIR}/librapichatterbox
      ${RAPI_DIR}/librapistage
      ${STAGE_INCLUDE_DIRS}
      ${GLIB_INCLUDE_DIRS}
      ../
    )

    LINK_DIRECTORIES(
      ${STAGE_LIBRARY_DIRS}
      ${RAPI_DIR}/librapicore
      ${RAPI_DIR}/librapichatterbox
      ${RAPI_DIR}/librapistage
    )

    #build a shared library
    ADD_LIBRARY( ${EXECUTABLE_NAME} MODULE
      main.cpp
      ../orbittarget.cpp
    )

    TARGET_LINK_LIBRARIES( ${EXECUTABLE_NAME}
      ${STAGE_LIBRARIES}
      ${OPENGL_LIBRARIES}
      autolab-rapi
      autolab-rapi-stage
    )
#ENDIF(QT_FOUND)

  # delete the "lib" prefix from the plugin libraries
  SET_TARGET_PROPERTIES( ${EXECUTABLE_NAME} PROPERTIES PREFIX "" )

  INSTALL(TARGETS ${EXECUTABLE_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
  )
