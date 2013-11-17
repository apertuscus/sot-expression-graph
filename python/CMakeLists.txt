INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES test_exp_graph.py test_exp_graph_full.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

