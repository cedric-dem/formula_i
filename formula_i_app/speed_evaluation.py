import matplotlib.pyplot as plt
import math

def get_real_acceleration():
	time_to_reach_each_speed = [  # according to that video https://www.youtube.com/watch?v=D4hyohfeP8Q
		[0, 0],
		[100, 1.98],
		[200, 3.9],
		[250, 5.9],
		[300, 8.6],
		[350, 12.9],
		[370, 17],
	]

	x = [time_to_reach_each_speed[i][1] for i in range(len(time_to_reach_each_speed))]
	y = [time_to_reach_each_speed[i][0] for i in range(len(time_to_reach_each_speed))]
	return x, y


def get_fake_y(i):
	if i != 0:  # thanks to chatgpt for solving the equation for a,b and c to match the hardcoded list
		result = -390.5 * math.exp(-0.173 * i) + 390.5
		result = min(370, result)
	else:
		result = 0
	return result

def get_fake_acceleration():
	max_length = 19
	pps = 10
	fake_x = [(i / pps) for i in range(pps * max_length)]
	fake_y = [get_fake_y(i / pps) for i in range(pps * max_length)]
	return (fake_x, fake_y)

x, y = get_real_acceleration()
fake_x, fake_y = get_fake_acceleration()

plt.plot(x, y, color = "red", marker = "o", label = "real acceleration")
plt.plot(fake_x, fake_y, color = "blue", marker = "none", label = "fake acceleration")

plt.title("Speed of F1")
plt.xlabel("time in seconds")
plt.ylabel("Speed in kmh")
plt.legend()
plt.grid(True)

plt.show()
