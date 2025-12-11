import numpy as np
from drones import drone
import matplotlib.pyplot as plt
import time

def simulate(drones, T, mode="full", bibd_blocks=None, visualize=True):
    plt.ion()

    size = 10
    visited = np.zeros((size, size), dtype=int)
    t = 0
    while visited.min() == 0 and t < T:
        t = t + 1
        # Communication phase
        if mode == "full":
            """All drones on at the same time. Each communicates its local map to every other drone."""
            for d in drones:
                for other in drones:
                    if d.id != other.id:
                        d.transmit(other)
                        if visualize:
                            drone.visualize_map(drones, visited, t, transmitter_drone=d, reciver_drone=other)
        else:
            block = bibd_blocks[t % len(bibd_blocks)]
            """Perform pairwise communication only among drones in this BIBD block."""
            active = [drones[i] for i in block]
            for d in active:
                for other in active:
                    if other.id != d.id:
                        d.transmit(other)
                        if visualize:
                            drone.visualize_map(drones, visited, t, transmitter_drone=d, reciver_drone=other)


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
    num_simulations = 100
    num_transmits = 0
    num_receives = 0
    for i in range(num_simulations):
        drones = []
        for i in range(N):
            x = np.random.randint(0, 10)
            y = np.random.randint(0, 10)
            drones.append(drone(i, [x, y]))

        # Run simulation
        simulate(drones, T=10000, mode=mode, bibd_blocks=bibd_blocks, visualize=False)

        # Print results
        print("\n--- FINAL COMMUNICATION COUNTS ---")
        transmit_avg = 0
        receive_avg = 0
        for d in drones:
            print(f"Drone {d.id}: received={d.num_recieves}, transmitted={d.num_transmits}")
            transmit_avg += d.num_transmits
            receive_avg += d.num_recieves
        num_transmits += transmit_avg / N
        num_receives += receive_avg / N
    print(f"\nAverage transmits per drone over {num_simulations} simulations: {num_transmits / num_simulations}")
    print(f"Average receives per drone over {num_simulations} simulations: {num_receives / num_simulations}")


if __name__ == "__main__":
    main()
