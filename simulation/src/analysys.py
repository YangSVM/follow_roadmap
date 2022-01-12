#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt



if __name__ == '__main__':

    # 读原始的文件
    poses=np.loadtxt('poses2.txt')


    headings_np = np.array(poses)
    np.savetxt('poses3.txt', headings_np)

    plt.plot(headings_np[:, 0] ,headings_np[:, 1] , 'r-')
    plt.show()