"""
  Authors:
  Manuel Córdoba Ramos (University of Malaga)
  Alejandro Daniel Rodríguez Pijuan (University of Malaga)
  Date: 18/01/25

  This code represents the data obtained by a radar, equipped in a car created for the subject
  Industrial Informatics 2024/25, of the B.S. Industrial Electronics Engineering at the University of Malaga.

  The executed code subscribes to an MQTT topic from which it receives a string of 8 comma-separated values,
  which it will represent in a real-time polar plot.
  At the end of the code execution, it will display a summary figure of the time evolution of the received data,
  and will also display them on the terminal.

  For more information, visit the GitHub repository: https://github.com/Manuel-C-R/ESP32-CAR
"""


import paho.mqtt.client as mqtt
import numpy as np
import matplotlib.pyplot as plt
import time


# Stores the time instant at which the file is executed
first_time = time.time()


# Arrays in which information is stored
times = []  # Stores the time instants at which the information arrives
d_0 = []    # Stores distances in centimeters in the 0 degrees direction
d_45 = []   # Stores distances in centimeters in the 45 degrees direction
d_90 = []   # Stores distances in centimeters in the 90 degrees direction
d_135 = []  # Stores distances in centimeters in the 135 degrees direction
d_180 = []  # Stores distances in centimeters in the 180 degrees direction
d_225 = []  # Stores distances in centimeters in the 225 degrees direction
d_270 = []  # Stores distances in centimeters in the 270 degrees direction
d_315 = []  # Stores distances in centimeters in the 315 degrees direction


# Initialization of radar values
data = np.zeros(8)


# on_message () is executed when a new array of data is received through MQTT
def on_message(client, userdata, msg):
    global data

    # The message is a string with 8 numbers separated by commas
    new_data = np.array([float(x) for x in msg.payload.decode().split(',')])
    data = new_data

    # The values received are displayed on the terminal
    print("Data received:", data)

    # The received data is stored in its respective array,
    # together with the time instant with respect to the start of code execution.
    times.append(time.time()-first_time)
    d_0.append(data[0])
    d_45.append(data[1])
    d_90.append(data[2])
    d_135.append(data[3])
    d_180.append(data[4])
    d_225.append(data[5])
    d_270.append(data[6])
    d_315.append(data[7])

    # The radar graphic is redrawn
    update_radar()


# MQTT connection parameters
MQTT_BROKER = "mqtt.eclipseprojects.io"
MQTT_PORT = 1883
MQTT_TOPIC = "MCR/radar/data"

# MQTT client configuration
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Subscription to the MQTT topic
client.subscribe(MQTT_TOPIC)


# Create the figure and polar graph
fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'})
ax.set_yticks([0, 50, 100, 150, 200, 250, 300, 350, 400])  # Value markers
ax.set_yticklabels([])

# Adjust the angle so that 0 degrees is at the top of the screen.
ax.set_theta_zero_location('N')

# Title is established
ax.set_title("ESP32-Car Distance Radar", color='black', fontsize=16, va='bottom')


# update_radar() is executed to update the graph
# displaying the radar with the new values received.
def update_radar():


    ax.clear()

    # Angles for the 8 positions (uniformly distributed in a circle)
    angles = np.linspace(0, 2 * np.pi, len(data), endpoint=False).tolist()

    # The first value is at the top (0 degrees)
    angles = np.roll(angles, -2)

    # The values are assigned to the angles
    values = data.tolist()

    # Add the first angle at the end to close the circle.
    values.append(values[0])
    angles = np.append(angles, angles[0])

    # Plot the radar
    ax.plot(angles, values, color='b', linewidth=2.5, linestyle='solid')
    ax.fill(angles, values, color='b', alpha=0.2)

    # Radar settings: no labels on the Y-axis and display the indexes
    ax.set_yticklabels([])
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels([f'{i*45} \u00b0 ({values[i]}cm)' for i in range(len(data))])

    # Add the legend of the values 100, 200, 300, 400 on the radial axis.
    ax.set_yticks([0,50,100,150,200,250,300,350,400])  # Value positions
    ax.set_yticklabels(['','','100','','200','', '300','', '400cm'])  # Corresponding labels

    # Fixed scale at 400
    ax.set_ylim(0, 400)

    ax.set_title("ESP32-Car Distance Radar", color='black', fontsize=16, va='bottom')

    # Redraw the figure
    fig.canvas.draw()


# Start MQTT thread to receive messages
client.loop_start()


# Show the figure
plt.show()


# When the radar is closed, a graph with the time evolution of each distance is displayed.
print(f"Times: {times}")
print(f"0\u00b0   : {d_0}")
print(f"45\u00b0  : {d_45}")
print(f"90\u00b0  : {d_90}")
print(f"135\u00b0 : {d_135}")
print(f"180\u00b0 : {d_180}")
print(f"225\u00b0 : {d_225}")
print(f"270\u00b0 : {d_270}")
print(f"315\u00b0 : {d_315}")

# Create the figure and axes
plt.figure(figsize=(10, 6))

# Plot each data set
plt.plot(times, d_0, label='0°')
plt.plot(times, d_45, label='45°')
plt.plot(times, d_90, label='90°')
plt.plot(times, d_135, label='135°')
plt.plot(times, d_180, label='180°')
plt.plot(times, d_225, label='225°')
plt.plot(times, d_270, label='270°')
plt.plot(times, d_315, label='315°')

# Add title and labels
plt.title('Temporal evolution of distances', fontsize=14)
plt.xlabel('Time [s]', fontsize=12)
plt.ylabel('Distance [cm]', fontsize=12)

# Configure axis range
plt.ylim(0, 400)  # Y range

# Add legend
plt.legend()

# Show the figure
plt.grid(True)
plt.show()