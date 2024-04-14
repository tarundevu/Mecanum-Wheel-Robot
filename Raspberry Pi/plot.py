import math
import time
import matplotlib.pyplot as plt
import testrig
import numpy as np

array = [(0,0),(1,1),(3,4),(6,6)]

x_val = list()
y_val = list()
for x,y in array:
  
  x_val.append(x)
  y_val.append(y)

# print(new_list)
# plt.ion()
# plt.plot(x_val,y_val,'bo')
# plt.axis((0, 180, 0, 120))
# plt.show()
# plt.plot(x, y, 'bo')[0]
def main():
    plt.ion() 
    plt.axis((0, 180, 0, 120))
    plt.show()
    # x = np.arange(-10, 11)
 
    # Plot different mathematical functions
    while True:
        x=int(input("Press [enter]x to continue."))
        y=int(input("Press [enter]y to continue."))
        x_val.append(x)
        y_val.append(y)
        plt.plot(x_val, y_val, 'bo')
        # plt.draw()
        plt.pause(0.5)
        
 
if __name__ == '__main__':
    main()