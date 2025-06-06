import matplotlib.pyplot as plt
import numpy as np

# Sample data
data1 = np.random.normal(0, 1, 100)
data2 = np.random.normal(1, 2, 100)
data3 = np.random.normal(2, 1.5, 100)

data = [data1, data2, data3]

plt.figure(figsize=(10, 6))

# Custom widths and positions
widths = [0.3, 0.5, 0.7]
positions = [1, 2, 3.5]

box = plt.boxplot(data, 
                  notch=False,
                  vert=True,
                  patch_artist=False,
                  widths=0.7,
                  #positions=positions,
                  showmeans=True,
                  #showcaps=True,
                  #showbox=True,
                  showfliers=True,
                  #boxprops=dict(facecolor="lightblue", color="blue"),
                  whiskerprops=dict(color="orange"),
                  capprops=dict(color="green"),
                  medianprops=dict(color="red"),
                  #meanprops=dict(marker='D', markerfacecolor='purple', markersize=10),
                  meanprops=dict(marker='D', markersize=1),
                  flierprops=dict(marker='o', color='yellow', alpha=0.5))

plt.grid(True)
plt.title("Customized Boxplot with Different Widths and Positions")
plt.xlabel("Data Sets")
plt.ylabel("Values")
plt.xticks([1, 2, 3], ['Data1', 'Data2', 'Data3'])
plt.show()

