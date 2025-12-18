import math
import matplotlib.pyplot as plt


# Energy consumption (per communication time period)
send = 1.4 #Watts
receive = 1
communicate = send + receive
idle = 0.83
sleep = 0.13 

# def calc_rb(v, k, lamb=1):
#     r = lamb*(v-1)/(k-1) # every drone is in r block
#     b = v*r/k # number of blocks
#     return r, b

def calc_BIBD_energy(v, k, lamb=1):
    '''
    Calculates the total energy cost for BIBD communication method
    
    :param v: total number of drones
    :param k: number of drones in a group
    :param lamb: times of communication between each pair (assuming always = 1)
    '''
    r = lamb*(v-1)/(k-1) # every drone is in r block
    b = v*r/k # number of blocks
    t = math.comb(k, 2)*2*b # times of communication in total
    E = (communicate + (k-2)*idle + (v-k)*sleep)*t   # TODO: implement lamb as a variable
    return E

def calc_non_BIBD_energy(v, k, lamb=1):
    '''
    Calculates the total energy cost for non-BIBD communication method

    :param v: total number of drones
    :param k: number of drones in a group
    :param lamb: times of communication between each pair (assuming always = 1)
    '''
    t = math.comb(v, 2)*2 # times of communication in total (=(v choose 2)*2)
    E = (communicate + (v-2)*idle)*t    # TODO: implement lamb as a variable
    return E

# BIBD_energy = calc_BIBD_energy(7, 3, 1)
# No_BIBD_energy = calc_non_BIBD_energy(7, 3, 1)

# print(BIBD_energy)
# print(No_BIBD_energy)

# Example difference sets (v, k, lamb)
sample = [
    (7, 3, 1),
    (13, 4, 1), 
    (21, 5, 1),
    (28, 4, 1),
    (31, 6, 1),
    (43, 7, 1)
    (46, 6, 1),
]
same_k = [
    (7, 3, 1), 
    (9, 3, 1),
    (15, 3, 1),
    (21, 3, 1),
    (27, 3, 1),
    (33, 3, 1)
]

sets = same_k

# Calculate energies
BIBD_energies = [calc_BIBD_energy(v, k, lamb) for v, k, lamb in sets]
nonBIBD_energies = [calc_non_BIBD_energy(v, k, lamb) for v, k, lamb in sets]
labels = [f'v={v}, k={k}' for v, k, _ in sets]

# Plotting
x = range(len(sets))
width = 0.35

fig, ax = plt.subplots(figsize=(8,5))
bars1 = ax.bar([i - width/2 for i in x], BIBD_energies, width, label='BIBD')
bars2 = ax.bar([i + width/2 for i in x], nonBIBD_energies, width, label='Non-BIBD')


# Add numbers on top of bars
for bar in bars1 + bars2:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, height, f'{int(height)}', 
            ha='center', va='bottom', fontsize=9)
    
ax.set_ylabel('Total Energy (W)')
ax.set_title('BIBD vs Non-BIBD Energy Comparison')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()
plt.show()