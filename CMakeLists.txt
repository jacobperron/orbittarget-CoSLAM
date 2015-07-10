cmake_minimum_required(VERSION 2.6)

# Change this directory to point to your RAPI library
SET (RAPI_DIR ../../autolab-rapi)

SET (EXECUTABLE_NAME orbittarget)

INCLUDE_DIRECTORIES(
  ${RAPI_DIR}/librapicore
  ${RAPI_DIR}/librapichatterbox
)

LINK_DIRECTORIES(
  ${RAPI_DIR}/librapicore
  ${RAPI_DIR}/librapichatterbox
)

ADD_EXECUTABLE(${EXECUTABLE_NAME}
   main.cpp
   orbittarget.cpp
)

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
  autolab-rapi
  autolab-rapi-chatterbox
)

