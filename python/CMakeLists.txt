INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES test_exp_graph.py 
        test_exp_graph_full.py
        test_distance.py
        test_point_to_point.py
        test_point_to_point_fixed.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

