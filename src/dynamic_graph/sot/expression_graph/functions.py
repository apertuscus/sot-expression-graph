from dynamic_graph import plug
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.types import *


""" 
plug operational point to a feature

opmodif: operational point of the robot
index: ...
feature: the destination feature
"""
def plugFeatureToRobotOpPoint(feature, index, opmodif):
	plug(opmodif.signal('position'), feature.signal('w_T_o'+index))
	try:
		plug(opmodif.jacobian,feature.signal('w_J_o'+index))
	except :
		plug(opmodif.Jq,feature.signal('w_J_o'+index))


def plugFeatureToOpPoint(feature, index, op):
  try:
	  plug(op.ref_frame, feature.signal('w_T_o'+index))
  except :
	  feature.signal('w_T_o'+index).value = op.ref_frame


def plugOperationalPoint(feature, index, op):
	try:
		plugFeatureToRobotOpPoint(feature, index, op.ref_frame)
	except:
		plugFeatureToOpPoint(feature, index, op)



def createFeatureAnglePlaneVersor(name, elmt1, elmt2):
	assert elmt1.typename == 'plane'
	assert elmt2.typename == 'versor'

	feature = FeatureAngleBtwPlaneAndVersor(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	feature.norm1.value  = elmt1.normal
	feature.p1.value     = elmt1.position

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	feature.v2.value = elmt2.versor

	feature.reference.value = 0

	# Define a task
	task = Task('task'+name)
	task.controlGain.value = 1
	task.add(feature.name)
	return(task, feature)


def createFeatureAngleVersorVersor(name, elmt1, elmt2):
	assert elmt1.typename == 'versor'
	assert elmt2.typename == 'versor'

	feature = FeatureVersorToVersor(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	feature.v1.value = elmt1.versor

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	feature.v2.value = elmt2.versor

	feature.reference.value = 0

	# Define a task
	task = Task('task'+name)
	task.controlGain.value = 1
	task.add(feature.name)
	return(task, feature)


def createFeaturePointToPoint(name, elmt1, elmt2):
	assert elmt1.typename == 'point'
	assert elmt2.typename == 'point'

	feature = FeaturePointToPoint(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	feature.p1.value = elmt1.position

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	feature.p2.value = elmt2.position

	# Define a task
	task = Task('task'+name)
	task.controlGain.value = 1
	task.add(feature.name)
	return(task, feature)


"""
Main method to create the task and the feature
"""
def createTaskAndFeature(name, elmt1, elmt2, operation):
	if operation == 'distance':
		# line / line
		if   elmt1.typename == 'line' and elmt2.typename == 'line':
			return createFeatureDistanceLineLine(name, elmt1, elmt2)
		# point / surface
		elif elmt1.typename == 'point' and elmt2.typename == 'surface':
			return createFeatureDistanceSurfacePoint(name, elmt2, elmt1)
		elif elmt1.typename == 'surface' and elmt2.typename == 'point':
			return createFeatureDistanceSurfacePoint(name, elmt1, elmt2)
		# point / line
		elif elmt1.typename == 'line' and elmt2.typename == 'point':
			return createFeatureDistanceLinePoint(name, elmt1, elmt2)
		elif elmt1.typename == 'point' and elmt2.typename == 'line':
			return createFeatureDistanceLinePoint(name, elmt2, elmt1)
		# point / point
		elif elmt1.typename == 'point' and elmt2.typename == 'point':
			return createFeatureDistancePointPoint(name, elmt1, elmt2)

		# not handled.
		else :
			print 'Unable to compute the angle between ' + elmt1.typename + '/' + elmt2.typename 

	elif operation == 'angle':
		# plane / plane
		if   elmt1.typename == 'plane' and elmt2.typename == 'plane':
			return createFeatureAngleBtwPlaneAndPlane(name, elmt1, elmt2)
		# plane / versor
		elif elmt1.typename == 'plane' and elmt2.typename == 'versor':
			return createFeatureAnglePlaneVersor(name, elmt1, elmt2)
		elif elmt1.typename == 'versor' and elmt2.typename == 'plane':
			return createFeatureAnglePlaneVersor(name, elmt1, elmt2)
		# versor / versor
		elif elmt1.typename == 'versor' and elmt2.typename == 'versor':
			return createFeatureAngleVersorVersor(name, elmt1, elmt2)
		else:
			print 'Unable to compute the angle between ' + elmt1.typename  + '/' + elmt2.typename 

	else:
			print "pouet"

