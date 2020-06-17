
#%matplotlib notebook
import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv('/home/ros/thesis/ur5_ws/src/ur5_training/data/ur5-1/5.1/data.csv')

df.head()
print (df)
df.plot(x ='time', y='reward', kind = 'line')


plt.plot(df['time'], df['reward'])


plt.plot(df['time'], df['reward'])
plt.savefig('/home/ros/thesis/ur5_ws/src/ur5_training/data/learning_curve.png')
plt.show()
