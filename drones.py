import numpy as np
import matplotlib.pyplot as plt

UNEXPLORED = 0 # value in the map that indicates box is unexplored
EXPLORED = 1   # value in the map that indicates box is explored
DRONE = 2     # value in the map that indicates box is occupied by a drone
MAPSIZE = (10, 10)   # size of the map

class drone:
    """A simple drone class that can move, transmit and receive maps."""
    def __init__(self, id, position):
        """
            Initialize the drone with an ID and starting position.
            The drone starts with an empty map.
            Parameters:
                id: unique identifier for the drone
                position: starting position of the drone as [x, y]
                map: 2D numpy array representing the drone's local map
                num_receives: number of times the drone has received a map
                num_transmits: number of times the drone has transmitted its map
        """
        self.id = id
        self.position = position

        self.map = np.zeros(MAPSIZE)
        self.map[position[0], position[1]] = DRONE
        
        self.num_recieves = 0
        self.num_transmits = 0

    def move(self):
        '''
        Move the drone to a new position on the map that is unexplored if possible.
        The drone updates its map to mark the previous position as explored.
        If the drone is unable to find a new unexplored position after 6 attempts, it will
        move to an explored position if available, otherwise it stays in place. It updates 
        its map accordingly.
        '''
        # Simple random move
        map_shape = self.map.shape
        move_finished = False
        # check random moves until one of them is unexplored and within bounds
        count = 0
        backup = None
        while not move_finished:
            count += 1
            movex = np.random.choice([-1, 0, 1])
            movey = np.random.choice([-1, 0, 1])
            if 0 <= self.position[0] + movex < map_shape[0] and 0 <= self.position[1] + movey < map_shape[1]:
                if count < 6: # try to find random unexplored move first within 1 block distance
                    if self.map[self.position[0] + movex, self.position[1] + movey] == UNEXPLORED:
                        self.map[self.position[0], self.position[1]] = EXPLORED
                        self.position[0] += movex
                        self.position[1] += movey
                        self.map[self.position[0], self.position[1]] = DRONE
                        move_finished = True
                    elif self.map[self.position[0] + movex, self.position[1] + movey] == EXPLORED:
                        backup = (movex, movey)
                else:
                    if backup is not None:    # if no unexplored move found, move to explored if possible
                        movex, movey = backup
                        self.map[self.position[0], self.position[1]] = EXPLORED
                        self.position[0] += movex
                        self.position[1] += movey
                        self.map[self.position[0], self.position[1]] = DRONE
                        move_finished = True
                    else:
                        self.map[self.position[0], self.position[1]] = DRONE
                        move_finished = True  # no move possible, stay in place  
            
    def transmit(self, drones):
        '''
        Transmit the drone's map to other drones.
        Args:
            drones: drone objects to transmit the map to. must either be a 
            single drone object or a list of drone objects.
        '''
        # transmit map to other drones
        if type(drones) is not list:
            drones = [drones]
        for drone in drones:
            if drone.id != self.id:
                drone.receive(self.map)
            self.num_transmits += 1

    def receive(self, other):
        '''
        Receive a map from another drone and merge it with the drone's own map.
        If both maps indicate a drone in the same position, that position is marked
        with a special value (4). Explored areas and other drone positions are merged
        accordingly.

        Args:
            other: 2D numpy array representing the received map from another drone.        
        '''
        #remove old drone positions
        self.map[self.map >= DRONE] = EXPLORED

        self.num_recieves += 1
        
        A = self.map
        B = other
        # merge received map with own map
        merged = np.zeros_like(A)

        both_drone = (A == DRONE) & (B == DRONE)
        any_drone  = (A == DRONE) | (B == DRONE)
        any_exp    = (A == EXPLORED) | (B == EXPLORED)

        merged[both_drone] = 4
        merged[any_drone & (~both_drone)] = 2
        merged[any_exp & (~any_drone)] = 1

        self.map = merged
    
    
def visualize_map(drone_map):
    '''
    Visualize the drone's map using matplotlib.
    Args:
        drone_map: 2D numpy array representing the drone's map.
    '''
    plt.imshow(drone_map, cmap='gray_r')
    plt.show()  