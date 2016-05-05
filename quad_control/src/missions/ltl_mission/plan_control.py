import pickle

class ControlAutomaton:
	def __init__(self,env,plan):
		self.plan = plan
		self.env = env
		self.current_state = None
		self.control = nx.get_node_attributes(self.plan,"apply_control")
		self.period = 0.1

	def save(self,name):
		pickle.dump( self, open( name, "wb" ) )

	def load(self,name):
		pickle.load( self, open( name, "rb" ) )

	def init_state(self,position):
		cell = self.env.get_cell(position)
		for n in self.plan.graph['initial']:
			if cell==n[0]:
				return n

	def get_next_state(self,position):
		cell = self.env.get_cell(position)
		for n in self.plan.successors(self.current_state):
			if cell==n[0]:
				return n

	def get_control(self,position):
		self.current_state = self.get_next_state(position)
		
		if self.current_state:
			u = self.control[self.current_state]
			return u
		else:
			print "No fts state!!"
			return None