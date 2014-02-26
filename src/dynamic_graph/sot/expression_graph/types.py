from dynamic_graph import plug
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.core import *

class BaseElement:
	name = None
	ref_frame =None
	typename   = ""

	# Database of fixed elemts of the environment
	#  Can contain transformations (numerical values) or output signals of transformation
	frames = {}
	frames['ground'] = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))

	def __init__(self, name, robot, ref_frame_name):
		try :
			self.ref_frame = robot.frames[ref_frame_name]
		except:
			try:
				self.ref_frame = robot.features[ref_frame_name]
			except:
				self.ref_frame = self.frames[ref_frame_name]
		self.name = name


class PointElement(BaseElement):
	position = None
	typename   = "point"

	def __init__(self, name, robot, ref_frame_name, position = (0,0,0) ):
		BaseElement.__init__(self,name, robot, ref_frame_name)
		self.position = position


class VersorElement(BaseElement):
	versor = None
	typename   = "versor" 

	def __init__(self, name, robot, ref_frame_name, versor):
		BaseElement.__init__(self, name, robot, ref_frame_name)
		normedVersor = versor #/ norm(versor) # TODO
		self.versor = normedVersor


class LineElement(BaseElement):
	normal = (0,0,1)
	typename   = "line"

	def __init__(self, name, robot, ref_frame_name, normal):
		BaseElement.__init__(self,name, robot, ref_frame_name)
		self.normal = normal



class PlaneElement(BaseElement):
	normal = None
	position = None
	typename   = "plane"

	def __init__(self, name, robot, ref_frame_name, normal, position = (0,0,0) ):
		BaseElement.__init__(self,name, robot, ref_frame_name)
		self.normal = normal
		self.position = position


