import matplotlib.pyplot as plt
import pandas

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

df = pandas.read_csv('~/Downloads/3d.csv')

setpoint_x = df['setpoint_x']
setpoint_y = df['setpoint_y']
setpoint_z = df['setpoint_z']
cross_x = df['cross_x']
cross_y=df['cross_y']
cross_z=df['cross_z']
ax.scatter(setpoint_x,setpoint_y,setpoint_z, label='setpoint')
ax.scatter(cross_x, cross_y, cross_z, label='cross')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.legend()
plt.show()