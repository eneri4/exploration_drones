import numpy as np
import matplotlib.pyplot as plt

UNEXPLORED = 0 # value in the map that indicates box is unexplored
EXPLORED = 1   # value in the map that indicates box is explored
DRONE = 2     # value in the map that indicates box is occupied by a drone
MAPSIZE = (10, 10)   # size of the map
img = 0  
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

    def move_bfs(self):
        '''
        Move the drone to a new position on the map that is unexplored if possible.
        The drone updates its map to mark the previous position as explored. uses bredth first search to find the next unexplored position.
         
        '''
        dist = 1
        positions_checked = [(0, 0)]
        found = False
        while not found and dist <= 5:
            positions = [x - dist for x in range(2*dist + 1)]
            for i in positions:
                for j in positions:
                    if not (i, j) in positions_checked and 0 <= self.position[0] + i < MAPSIZE[0] and 0 <= self.position[1] + j < MAPSIZE[1]:
                        positions_checked.append((i, j))
                        if self.local_map[int(self.position[0] + i), int(self.position[1] + j)] == UNEXPLORED:
                            position = [i,j]
                            if abs(i) > 1:
                                position[0] = int(position[0]/abs(position[0]))
                            if abs(j) > 1:
                                position[1] = int(position[1]/abs(position[1]))
                            if not self.local_map[int(self.position[0] + position[0]), int(self.position[1] + position[1])] == DRONE:
                                found = True
                                self.local_map[int(self.position[0]), int(self.position[1])] = EXPLORED
                                self.local_map[int(self.position[0] + position[0]), int(self.position[1] + position[1])] = DRONE
                                self.position[0] += position[0]
                                self.position[1] += position[1]
                                print(position)

                                #print(int(self.position[0] + position[0]), int(self.position[1] + position[1]), position)
                                break
            dist = dist + 1
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
            moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            movex, movey = moves[np.random.randint(0, len(moves))]
            moves.remove((movex, movey))
            if 0 <= self.position[0] + movex < map_shape[0] and 0 <= self.position[1] + movey < map_shape[1]:
                if count <= 8: # try to find random unexplored move first within 1 block distance
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
        """Finalize communication by updating the local map with the global map."""
        self.local_map = self.global_map.copy()
    
    @staticmethod
    def generate_bibd_7_3_1():
        """Generate a Balanced Incomplete Block Design (BIBD) with parameters (7, 3, 1).
        Returns:
            blocks: list of blocks, where each block is a list of integers representing the positions of the drone positions in the block.
        """
        base = {0, 1, 3}
        blocks = []
        for i in range(7):
            block = sorted({(x + i) % 7 for x in base})
            blocks.append(block)
        return blocks


  
    @staticmethod
    def visualize_map(drones, visited, round_number, transmitter_drone = None, reciver_drone = None):
        """
        Visualize the current state of the map for a given round number.
        Args:
            drones: list of drone objects
            visited: 2D numpy array representing the global map
            round_number: current round number
            transmitter_drone: optional, the drone that transmitted its map
            reciver_drone: optional, the drone that received the map
        """
        # Get size of the map
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
        global img
        plt.clf()
        plt.imshow(rgb)
        plt.title(f"Round {round_number}")
        plt.pause(0.1)
        img += 1
