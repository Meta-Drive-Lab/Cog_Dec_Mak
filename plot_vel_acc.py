import matplotlib.pyplot as plt

# list for vel and acc
y1 = []
y2 = []

with open('record_all.txt', 'r') as file:

    for line in file:

        values = line.strip().split(', ')

        if len(values) >= 2:

            velocity = float(values[0]) # velocity
            acceleration = float(values[1]) # acceleration
            y1.append(velocity)
            y2.append(acceleration)


# figure
fig, ax1 = plt.subplots(figsize=(8,3))

# plot velocity
ax1.plot(y1, marker='o', linestyle='-', color='blue', label='Velocity')
ax1.set_xlabel('Time step', fontweight ='bold', fontsize = 13)

ax1.set_ylabel('Velocity(m/s)', color='blue', fontweight ='bold', fontsize = 13)
ax1.tick_params(axis='y', labelcolor='blue')


# plot acceleration
ax2 = ax1.twinx()
ax2.plot(y2, marker='s', linestyle='--', color='r', label='Acceleration')
ax2.set_ylabel('Accerleration(m/s$^2$)', color='r', fontweight ='bold', fontsize = 13)
ax2.tick_params(axis='y', labelcolor='r')

plt.title('')
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize = 12)


plt.subplots_adjust(bottom=0.2) 

plt.show()
