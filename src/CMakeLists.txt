# Copyright 2013, Francois Keith, JRL, CNRS/AIST

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Add the features
SET(CONTROLLER_NAME expression_graph)
ADD_LIBRARY(${CONTROLLER_NAME}
  SHARED
  feature-expr-graph-abstract.cpp

  feature-line-to-line-distance.cpp
  feature-point-to-point-distance.cpp
  feature-point-to-line-distance.cpp
  feature-point-to-surface-distance.cpp
  feature-point-to-point.cpp
  feature-point-to-line-projection.cpp

  feature-plane-to-plane-angle.cpp
  feature-plane-to-versor-angle.cpp
  feature-versor-to-versor-angle.cpp
  feature-point-toward-angle.cpp
  helper.cpp
)


# Link the dynamic library containing the SoT with its dependencies.
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")
#external package in ros packages
ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "expressiongraph")


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
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/expression_graph/types.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/expression_graph/functions.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/expression_graph
  )

