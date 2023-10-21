import numpy as np
# import matplotlib.pyplot as plt
# from copy import deepcopy
# import math

'''use x and y calculate grid size'''
def state_to_idx(x,y,grid_size):
    return (x * grid_size) + y

'''use grid size calculate x and y'''
def idx_to_state(idx, grid_size):
    x,y = divmod(idx, grid_size) # return div and remainder
    return x,y

'''Assign (index = idx) element = 1, others = 0'''
def onehot_state(idx, length, height):
    state = np.zeros(length*height)
    state[idx] = 1
    return state

'''Return the non-zero element in s'''
def from_onehot(s):
    for i in range(len(s)):
        if s[i] != 0:
            return i

def generate_B_matrices(length,height):
    '''
    Construct B matrix
    The third dimension stands for {0:B_L, 1:B_r, 2:B_up, 3:B_down}
    Details as the calculation procedure below
    '''
    N = length * height
    N_size = (N,N)
    B_up = np.zeros(N_size)
    B_down = np.zeros(N_size)
    B_L = np.zeros(N_size)
    B_R = np.zeros(N_size)
    B_up_right = np.zeros(N_size)
    B_down_right = np.zeros(N_size)
    B_up_left = np.zeros(N_size)
    B_down_left = np.zeros(N_size)
    B = np.zeros((N,N,8))   #create N(N*4) array
    
    for i in range(N):
        for j in range(N):
            start_x, start_y = idx_to_state(i, length)
            end_x, end_y = idx_to_state(j, length)
        
            '''left matrix'''
            if start_y == 0:
                if start_y == end_y and start_x == end_x:
                    B_L[i,j] = 1
            if start_y != 0:
                if end_y == start_y - 1 and start_x == end_x:
                    B_L[i,j] = 1
        
            '''right matrix'''
            if start_y == length-1:
                if start_y == end_y and start_x == end_x:
                    B_R[i,j] = 1
            if start_y != length -1:
                if end_y == start_y + 1 and start_x == end_x:
                    B_R[i,j] = 1
        
            '''up matrix'''
            if start_x ==0:
                if start_x == end_x and start_y == end_y:
                    B_up[i,j] = 1
            if start_x != 0:
                if end_x == start_x - 1 and start_y == end_y:
                    B_up[i,j] = 1
        
            '''down matrix'''
            if start_x == height-1:
                if start_x == end_x and start_y == end_y:
                    B_down[i,j] = 1
            if start_x != height -1:
                if end_x == start_x + 1 and start_y == end_y:
                    B_down[i,j] = 1
                    
            '''up-right matrix'''
            if start_x == 0 or start_y == length-1:
                pass
            else:
                if end_x == start_x - 1 and end_y == start_y + 1:
                    B_up_right[i,j] = 1
    
            '''up-left matrix'''
            if start_x == 0 or start_y == 0:
                pass
            else:
                if end_x == start_x - 1 and end_y == start_y - 1:
                    B_up_left[i,j] = 1
    
            '''down-right matrix'''
            if start_x == height-1 or start_y == length-1:
                pass
            else:
                if end_x == start_x + 1 and end_y == start_y + 1:
                    B_down_right[i,j] = 1
    
            '''down-left matrix'''
            if start_x == height-1 or start_y == 0:
                pass
            else:
                if end_x == start_x + 1 and end_y == start_y - 1:
                    B_down_left[i,j] = 1
            
    B[:,:,0] = B_L.T
    B[:,:,1] = B_R.T
    B[:,:,2] = B_up.T
    B[:,:,3] = B_down.T
    B[:,:,4] = B_up_right.T
    B[:,:,5] = B_up_left.T
    B[:,:,6] = B_down_right.T
    B[:,:,7] = B_down_left.T
    
    return B
        
'''log stable (confusing)'''
def log_stable(x):
    return np.log(x + 1e-5)

'''Calculate H_A as the entropy (as the information gain)'''
def entropy(A):
    H_A = - (A * log_stable(A)).sum(axis=0)
    return H_A





