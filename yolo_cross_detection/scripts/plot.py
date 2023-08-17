import matplotlib.pyplot as plt
import pandas

fig = plt.figure()
df = pandas.read_csv('~/Downloads/3d.csv')
setpoint_x = df['setpoint_x']
setpoint_y = df['setpoint_y']
setpoint_z = df['setpoint_z']
cross_x = df['cross_x']
cross_y = df['cross_y']
cross_z=  df['cross_z']
drone_x = df['drone_x']
drone_y = df['drone_y']
drone_z = df['drone_z']

# ax = fig.add_subplot(111,projection='3d')

# ax.plot(setpoint_x,setpoint_y,setpoint_z, label='setpoint')
# ax.plot(drone_x, drone_y, drone_z, label='drone')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
plt.plot(setpoint_y)
plt.plot(drone_y)
plt.legend()
plt.show()