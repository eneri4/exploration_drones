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

        self.local_map = np.zeros(MAPSIZE) # Updated when communication over (tracks its own move only)
        self.local_map[position[0], position[1]] = DRONE 

        self.global_map = np.zeros(MAPSIZE) # Updates while communication taking place 
        self.global_map[position[0], position[1]] = DRONE
        
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
        # Remove old drone positions
        self.local_map[self.local_map >= DRONE] = EXPLORED

        # Simple random move
        map_shape = self.local_map.shape
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
                    if self.local_map[self.position[0] + movex, self.position[1] + movey] == UNEXPLORED:
                        self.local_map[self.position[0], self.position[1]] = EXPLORED
                        self.position[0] += movex
                        self.position[1] += movey
                        self.local_map[self.position[0], self.position[1]] = DRONE
                        move_finished = True
                    elif self.local_map[self.position[0] + movex, self.position[1] + movey] == EXPLORED:
                        backup = (movex, movey)
                else:
                    if backup is not None:    # if no unexplored move found, move to explored if possible
                        movex, movey = backup
                        self.local_map[self.position[0], self.position[1]] = EXPLORED
                        self.position[0] += movex
                        self.position[1] += movey
                        self.local_map[self.position[0], self.position[1]] = DRONE
                        move_finished = True
                    else:
                        self.local_map[self.position[0], self.position[1]] = DRONE
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
                drone.receive(self.local_map)
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

        self.num_recieves += 1
        
        A = self.local_map
        B = other

        # merge received map with own map
        merged = np.zeros_like(A)

        both_drone = (A == DRONE) & (B == DRONE)
        any_drone  = (A == DRONE) | (B == DRONE)
        any_exp    = (A == EXPLORED) | (B == EXPLORED)

        merged[both_drone] = 4
        merged[any_drone & (~both_drone)] = 2
        merged[any_exp & (~any_drone)] = 1

        self.global_map = merged

    def finalize_communication(self):
        """Update local map to global map for the next round."""
        self.local_map = self.global_map.copy()
    
    @staticmethod
    def generate_bibd_7_3_1():
        """Return the 7 blocks of the (7,3,1) BIBD."""
        base = {0, 1, 3}
        blocks = []
        for i in range(7):
            block = sorted({(x + i) % 7 for x in base})
            blocks.append(block)
        return blocks


        
    @staticmethod
    def visualize_map(drones, visited, round_number, transmitter_drone = None, reciver_drone = None):
        """Displays the global map using RGB colors."""
        size = len(visited)

        # Make an RGB image (height × width × 3)
        rgb = np.zeros((size, size, 3), dtype=float)

        # Color definitions (R, G, B) in 0–1 range
        COLOR_UNEXPLORED = np.array([1.0, 1.0, 1.0])   # white
        COLOR_EXPLORED   = np.array([0.6, 0.6, 0.6])   # light gray
        COLOR_DRONE      = np.array([0.0, 0.3, 1.0])   # blue
        COLOR_RECEIVER   = np.array([0.0, 1.0, 0.0])   # green
        COLOR_TRANSMITTER= np.array([1.0, 0.0, 0.0])   # red

        # Fill unexplored / explored layers
        rgb[visited == 0] = COLOR_UNEXPLORED
        rgb[visited == 1] = COLOR_EXPLORED
        # Mark drone positions in RGB array
        for d in drones:
            x, y = d.position
            rgb[y, x] = COLOR_DRONE
        # Highlight transmitter and receiver drones if provided
        if transmitter_drone is not None:   
            x, y = transmitter_drone.position
            rgb[y, x] = COLOR_TRANSMITTER   
        if reciver_drone is not None:
            x, y = reciver_drone.position
            rgb[y, x] = COLOR_RECEIVER
        # Display
        plt.clf()
        plt.imshow(rgb)
        plt.title(f"Round {round_number}")
        plt.pause(0.1)