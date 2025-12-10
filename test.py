import numpy as np
from drones import drone
import matplotlib.pyplot as plt
import time

def simulate(drones, T, mode="full", bibd_blocks=None):
    plt.ion()

    size = 10
    visited = np.zeros((size, size), dtype=int)

    for t in range(T):

        # Communication phase
        if mode == "full":
            drone.full_communication(drones)
        else:
            block = bibd_blocks[t % len(bibd_blocks)]
            drone.bibd_communicate(drones, block)

        # Update local maps to global
        for d in drones:
            d.finalize_communication()

        # Mark current location as visited before they move
        for d in drones:
            x, y = d.position
            visited[y][x] = 1

        # Print number of unexplored cells
        unexplored_count = np.sum(visited == 0)
        print(f"\nUnexplored cells remaining: {unexplored_count}")

        # Move
        for d in drones:
            d.move()

        # Visualize
        drone.visualize_map(drones, visited, t)

    plt.ioff()
    plt.show()


def main():
    # Choose number of drones for simulation
    N = 7

    # Choose which communication mode to use
    print("Choose communication mode:")
    print("1. full (all drones communicate each round)")
    print("2. BIBD (7,3,1) block design communication")

    choice = input("Enter 1 or 2: ").strip()

    if choice == "1":
        mode = "full"
        bibd_blocks = None
    elif choice == "2":
        mode = "BIBD"
        bibd_blocks = drone.generate_bibd_7_3_1()
    else:
        print("Invalid input. Defaulting to full communication.")
        mode = "full"
        bibd_blocks = None

    # Initialize drones location randomly
    drones = []
    for i in range(N):
        x = np.random.randint(0, 10)
        y = np.random.randint(0, 10)
        drones.append(drone(i, [x, y]))

    # Run simulation
    simulate(drones, T=20, mode=mode, bibd_blocks=bibd_blocks)

    # Print results
    print("\n--- FINAL COMMUNICATION COUNTS ---")
    for d in drones:
        print(f"Drone {d.id}: received={d.num_recieves}, transmitted={d.num_transmits}")

    


if __name__ == "__main__":
    main()
