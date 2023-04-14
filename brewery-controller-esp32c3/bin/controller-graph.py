import matplotlib.pyplot as plt
import matplotlib.animation as animation

temp_color = '#4287f5'
setpoint_color = '#a142f5'
power_color = '#4da341'

fig, ax = plt.subplots(2, 1, sharey=False, sharex=False)
fig.set_figwidth(12)
fig.set_figheight(12)
ax_main = ax[0]
ax_zoom = ax[1]
ax_main_power = ax_main.twinx()
ax_zoom_power = ax_zoom.twinx()

ax_main.get_xaxis().set_visible(False)
ax_zoom.get_xaxis().set_visible(False)

zoom = 5
xs = []
ys1 = []
ys2 = []
ys3 = []

def animate(i, xs, ys1, ys2, ys3):
    # Read temperature (Celsius) from TMP102
    line = input()

    if line.startswith('I DATA '):
        data = line[5:].split(',')
        temperature = float(data[0])
        setpoint = float(data[1])
        power = float(data[2])

        # Add x and y to lists
        xs.append(len(xs) - 1)
        ys1.append(temperature)
        ys2.append(setpoint)
        ys3.append(power)

    ax_main.clear()
    ax_main.set_ylabel('Temperature (°F)')
    ax_main.set_ylim(59, 221)
    ax_main.set_yticks(range(60, 221, 10))
    ax_main.plot(xs, ys1, color=temp_color)
    ax_main.plot(xs, ys2, color=setpoint_color)

    ax_main_power.clear()
    ax_main_power.set_ylim(-1, 101)
    ax_main_power.set_yticks(range(0, 101, 5))
    ax_main_power.plot(xs, ys3, color=power_color)

    ax_zoom.clear()
    ax_zoom.plot(xs[-zoom:], ys1[-zoom:], color=temp_color)
    ax_zoom.plot(xs[-zoom:], ys2[-zoom:], color=setpoint_color)

    ax_zoom_power.clear()
    ax_zoom_power.plot(xs[-zoom:], ys3[-zoom:], color=power_color)

    # Format plot
    plt.tight_layout()

ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys1, ys2, ys3), interval=100, save_count=30)
plt.show()
