from formula_i.misc import *
from formula_i.car import Car

def main():
	initialize_simulation()
	setup_environment()
	car = Car()
	car.apply_steering(0.0)

	try:
		run_simulation(car)
	finally:
		p.disconnect()

if __name__ == "__main__":
	main()
