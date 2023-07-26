from .controller import Controller


class Utils:

	@staticmethod
	def constrain(v, r):
		if v > r[1]:
			return r[1]
		elif v < r[0]:
			return r[0]
		else:
			return v
		
		# return max(min(v, r[1]), r[0])

	@staticmethod
	def remap(v, from_range, to_range):
		if from_range[0] == from_range[1]:
			if v < from_range[0]:
				v_new = to_range[0]
			elif v > from_range[0]:
				v_new = to_range[1]
			else:
				v_new = (to_range[0] + to_range[1]) / 2

		elif to_range[0] == to_range[1]:
			v_new = to_range[0]
		else:
			v_new = min(
				to_range[0],
				max(
				to_range[1],
				((((v - from_range[0]) * (to_range[1] - to_range[0])) / (from_range[1] - from_range[0])) + to_range[0])))
		
		return int(round(v_new))

	@staticmethod
	def scale_range(r, by):
		rmin, rmax = r
		rmid = (rmin + rmax) / 2
		padding = by * (rmax - rmid)
		new_rmin = int(round(rmid - padding))
		new_rmax = int(round(rmid + padding))
		return [new_rmin, new_rmax]


class Servo:

	MASTER = Controller()

	def __init__(self, channel, irom, srom):
		Servo.MASTER.set_speed(channel, 0)
		Servo.MASTER.set_acceleration(channel, 0)
		
		self._CHANNEL = channel

		self.rom = {
			"input": irom,
			"max": srom,
			"current": srom,
			"delta": 1.0
		}

		self.target = {
			"input": 0,
			"current": 0,
		}

		self.acceleration = 256

	def set_target(self, t):
		t_mapped = Utils.remap(t, from_range=self.rom["input"], to_range=self.rom["current"])
		Servo.MASTER.set_target(self._CHANNEL, t_mapped)
		self.target["input"] = t
		self.target["current"] = t_mapped
		return t_mapped

	def get_position(self):
		return Servo.MASTER.get_position(self._CHANNEL)

	def adjust_srom(self, by):
		self.set_srom(self.rom["delta"] + by)

	def set_srom(self, delta):
		delta = round(delta, 5)
		delta = Utils.constrain(delta, [0.0, 1.0])
		new_srom = Utils.scale_range(self.rom["max"], by=delta)
		Servo.MASTER.set_range(self._CHANNEL, *new_srom)

		current_position = self.get_position()
		new_position = Utils.remap(current_position, from_range=self.rom["current"], to_range=new_srom)
		Servo.MASTER.set_target(self._CHANNEL, new_position)
		self.target["current"] = new_position		

		self.rom["current"] = new_srom
		self.rom["delta"] = delta

	def adjust_acceleration(self, by):
		self.set_acceleration(self.acceleration + by)

	def set_acceleration(self, a):
		a = int(round(a))
		a = Utils.constrain(a, [1, 256])
		self.acceleration = a
		if a == 256:
			a = 0

		Servo.MASTER.set_acceleration(self._CHANNEL, a)

	def disable(self):
		Servo.MASTER.disable(self._CHANNEL)
		self.target["input"] = 0
		self.target["current"] = 0


