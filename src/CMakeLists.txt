# Copyright 2013, Francois Keith, JRL, CNRS/AIST

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Add the library to wrap the controller of HRP4.

SET(CONTROLLER_NAME expression_graph)
ADD_LIBRARY(${CONTROLLER_NAME}
  SHARED
  feature-expressionGraph.cpp
  feature-expressionFullGraph.cpp
  feature-expr-graph-abstract.cpp
  feature-point-to-point.cpp
  feature-point-to-surface.cpp
  feature-point-to-line-distance.cpp
  helper.cpp
)


# Link the dynamic library containing the SoT with its dependencies.
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")
#external package in ros packages
ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "expressiongraph")
ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "expressiongraph_tf")


IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
ENDIF(UNIX AND NOT APPLE)


INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${CONTROLLER_NAME})
SET(NEW_ENTITY_CLASS ${ENTITIES})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/expression_graph/${PYTHON_LIBRARY_NAME}"
  ${CONTROLLER_NAME}
  feature-expressionGraph-${PYTHON_LIBRARY_NAME}-wrap
  feature-expressionFullGraph-${PYTHON_LIBRARY_NAME}-wrap
  )


# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/expression_graph/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/expression_graph
  )

