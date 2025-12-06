from drones import *
# structure is drone(id, position)
drone1 = drone(1, [0, 0])
drone2 = drone(2, [5, 2])
drone3 = drone(3, [2, 9])
# simple program that moves all the drones and makes them transmit their maps to each other. 
# visualizes the map of drone 1 after each round as a window. you can exit the window and the 
# program will continue. after 10 rounds the final maps of all drones are printed and compared.
# maps will be shown after each round.
for i in range(10):
    drone2.move()
    drone2.transmit([drone1,drone3])
    drone3.move()
    drone3.transmit([drone1,drone2])
    drone1.move()
    drone1.transmit([drone2,drone3])
    visualize_map(drone1.map)
print(drone1.map)
print(drone2.map)
print(drone3.map)

# this test checks if the maps are synchronized after multiple transmissions
assert np.array_equal(drone2.map, drone3.map)
assert np.array_equal(drone1.map, drone2.map) 






