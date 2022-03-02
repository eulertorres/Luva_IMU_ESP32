# importing libraries
import matplotlib.pyplot as plt
import numpy as np
import math
peso = 50

# Get the angles from 0 to 2 pie (360 degree) in narray object
X = np.array([3.7, 3.8, 3.9, 4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5.0, 5.1, 5.2])
X1 = np.array([3.7, 3.8, 3.9, 4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5.0, 5.1, 5.2])
# Using built-in trigonometric function we can directly plot
# the given cosine wave for the given angles
Y = np.ones(len(X))*peso
  
fig, ax = plt.subplots(figsize=(10, 6))

ax.set(title = "Peso x força",
       xlabel = "Valor de força [1/R]", 
       ylabel = "Peso [g])")

ax.plot(X,Y,marker = 'o')

ax.grid()

plt.show()
