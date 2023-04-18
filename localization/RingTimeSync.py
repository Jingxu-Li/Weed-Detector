"""TimeSync but in ringbuffer implementation

Almost done, made it more logical and clear than the 
former version; although the comparison is done, the 
position pointer will move an extra unit in some special 
datasets (e.g., the first set), will fix it soon.

Updated date: 18/04/23 by ychen441
"""


import sys
import numpy as np
from RingBufferFuncs import RingBuffer

# Hand-written data for test
#t_std = np.array([1, 2, 3, 4, 8, 10])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.4, 4.8, 5, 10.05, 10.5])

t_std = np.array([1, 2, 3, 4, 8, 10])
t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.3, 4.8, 5, 9.95, 10.05])

#t_std = np.array([1, 2, 3, 4, 8, 10])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.3, 4.8, 5, 9.95, 9.98])

#t_std = np.array([1, 2, 3, 4, 8, 10, 11, 12, 13, 14])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9])


data_unsync = np.array([[3, 0, 0], [3, 0, 0], [1, 0, 0], [1, 0, 0], [27, 0, 0],
                 [26, 0, 0], [50, 0, 0], [2, 0, 0], [1, 0, 0], [5, 0, 0], [1, 0, 0], [22, 0, 0], [20, 0, 0]])

std_size = 5  # std_buffer size
unsync_size = 5  # unsync_buffer size
std_buffer = RingBuffer(std_size)  
unsync_buffer = RingBuffer(unsync_size)

counter_i = 0  # std_buffer data-appending pointer
counter_j = 0  # unsync_buffer data-appending pointer
counter_data = 0  # unsync_data fetching pointer
pos_i = 0  # std_buffer position pointer
pos_j_front = 0  # unsync_buffer pos front pointer
pos_j_back = 0  # unsync_buffer pos back pointer

threshold = 0.1  # Threshold for checking data loss
false_counter = 0
tolerance = 3  # Max out-of-bound acceptance
sync = []  # Store synced data

for i in range(0, len(t_std)):  # loop for std num times minially

    if i < len(t_std):

        std_buffer.append(t_std[counter_i])  # Append t_std[0], pos_i = 0

        print("counter_i appended", counter_i)

        while True:

            if counter_j < len(t_unsync) - 1:

                if pos_j_front == 0 and  pos_j_back == 0:  # Append extra t_unsync for comparison

                    unsync_buffer.append(t_unsync[counter_j])  # Append t_unsyc[0]
                    counter_j += 1  # unsync pointer moves
                    pos_j_back += 1  # unsync pos back moves
                    if pos_j_back > unsync_size - 1:
                        pos_j_back = 0  # Pop the oldest one when full
                    unsync_buffer.append(t_unsync[counter_j])

                    print("checkpoint 1", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    continue  # Jump out the initialisation and start comparisons

                if unsync_buffer.data[pos_j_front] > std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] > std_buffer.data[pos_i]:  
                    # Both unsync larger than std, std moves
                    print("checkpoint 2", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    pos_i += 1
                    if pos_i > std_size - 1:
                        pos_i = 0
                    counter_i += 1
                    
                    break

                if unsync_buffer.data[pos_j_front] < std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] > std_buffer.data[pos_i]:
                    # Possibly can be interpolated, check threshold
                    # Parameters for interpolation
                    front_diff = std_buffer.data[pos_i] - unsync_buffer.data[pos_j_front]
                    back_diff = unsync_buffer.data[pos_j_back] - std_buffer.data[pos_i]
                    diff = unsync_buffer.data[pos_j_back] - unsync_buffer.data[pos_j_front]
                    front_scale = back_diff / diff
                    back_scale = front_diff / diff

                    if front_diff < threshold and back_diff < threshold:
                        print("checkpoint 3", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        # Able to be interpolated
                        # count_data should be zero initially
                        sync_x = front_scale * data_unsync[counter_data][0] + back_scale * data_unsync[counter_data+1][0]
                        sync_y = front_scale * data_unsync[counter_data][1] + back_scale * data_unsync[counter_data+1][1]
                        sync_z = front_scale * data_unsync[counter_data][2] + back_scale * data_unsync[counter_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointers
                        pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        pos_j_front += 1
                        pos_j_back += 1
                        if pos_j_front > unsync_size - 1:
                            pos_j_front = 0
                        if pos_j_back > unsync_size - 1:
                            pos_j_back = 0
                        # Move data pointer
                        counter_i += 1
                        counter_j += 1
                        counter_data += 1
                        unsync_buffer.append(t_unsync[counter_j])  # Append new t_unsync
                        
                        break  # Jump out, append new t_std, and start new comparisons

                    if front_diff < threshold and back_diff > threshold:
                        print("checkpoint 4", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        # Interpolation solely depends on the non-out-of-bound front
                        sync_x = data_unsync[counter_data][0]
                        sync_y = data_unsync[counter_data][1]
                        sync_z = data_unsync[counter_data][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointers
                        pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        pos_j_front += 1
                        pos_j_back += 1
                        if pos_j_front > unsync_size - 1:
                            pos_j_front = 0
                        if pos_j_back > unsync_size - 1:
                            pos_j_back = 0
                        # Move data pointer
                        counter_i += 1
                        counter_j += 1
                        counter_data += 1
                        unsync_buffer.append(t_unsync[counter_j])
                        
                        break

                    if front_diff > threshold and back_diff < threshold:
                        print("checkpoint 5", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        # Interpolation solely depends on the non-out-of-bound back
                        sync_x = data_unsync[counter_data+1][0]
                        sync_y = data_unsync[counter_data+1][1]
                        sync_z = data_unsync[counter_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointers
                        pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        pos_j_front += 1
                        pos_j_back += 1
                        if pos_j_front > unsync_size - 1:
                            pos_j_front = 0
                        if pos_j_back > unsync_size - 1:
                            pos_j_back = 0
                        # Move data pointer
                        counter_i += 1
                        counter_j += 1
                        counter_data += 1
                        unsync_buffer.append(t_unsync[counter_j])  # Append new t_unsync
                        
                        break

                    if front_diff >= threshold and back_diff >= threshold:
                        print("checkpoint 6", unsync_buffer.data[pos_j_front], 
                                unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        # Log a potential data loss to the false_counter
                        false_counter += 1
                        if false_counter == tolerance:
                            print("checkpoint6")
                            print(sync)
                            print(false_counter)
                            print(pos_j_front, pos_j_back, pos_i)
                            print(unsync_buffer.data, std_buffer.data)
                            sys.exit('imu data loss.')
                        else:  # t_std moves and check if the error appears again
                            # Move data pointer
                            counter_i += 1
                            # Move position pointer
                            pos_i += 1
                            if pos_i > std_size - 1:
                                pos_i = 0
                            
                            break

                if unsync_buffer.data[pos_j_front] < std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] < std_buffer.data[pos_i]:
                    print("checkpoint 7", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    # Both unsync smaller than std, unsync moves
                    # Move position pointer
                    pos_j_front += 1
                    pos_j_back += 1
                    if pos_j_front > unsync_size - 1:
                        pos_j_front = 0
                    if pos_j_back > unsync_size - 1:
                        pos_j_back = 0
                    # Move data pointer
                    counter_j += 1
                    counter_data += 1
                    unsync_buffer.append(t_unsync[counter_j])  # Append a new t_unsync to the buffer
                    

            elif counter_j >= len(t_unsync) - 1:
                # unsync_buffer reaches the end with pos_front at [len-2] and pos_back at [len-1]
                # Freeze t_unsync position when reaching the end
                unsync_front = unsync_buffer.data[pos_j_front]
                unsync_back = unsync_buffer.data[pos_j_back]
                
                
                if unsync_front > std_buffer.data[pos_i] and unsync_back > std_buffer.data[pos_i]:
                    print("checkpoint 8", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    # t_std moves, unsync's frozen
                    # Move position pointer
                    """pos_i += 1
                    if pos_i > std_size - 1:
                        pos_i = 0
                    # Move data pointer
                    counter_i += 1"""
                    
                    break  # Jump out and append a new t_std

                if unsync_front < std_buffer.data[pos_i] and unsync_back > std_buffer.data[pos_i]:
                    # May need interpolation
                    # Prepare parameters
                    front_diff = std_buffer.data[pos_i] - unsync_front
                    back_diff = unsync_back - std_buffer.data[pos_i]
                    diff = unsync_back -unsync_front
                    front_scale = back_diff / diff
                    back_scale = front_diff / diff

                    if front_diff < threshold and back_diff < threshold:
                        print("checkpoint 9", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = front_scale * data_unsync[counter_data-1][0] + back_scale * data_unsync[counter_data][0]
                        sync_y = front_scale * data_unsync[counter_data-1][1] + back_scale * data_unsync[counter_data][1]
                        sync_z = front_scale * data_unsync[counter_data-1][2] + back_scale * data_unsync[counter_data][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointer
                        """pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        # Move data pointer
                        counter_i += 1"""
                        
                        break

                    if front_diff < threshold and back_diff > threshold:
                        print("checkpoint 10", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[counter_data-1][0]
                        sync_y = data_unsync[counter_data-1][1]
                        sync_z = data_unsync[counter_data-1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointer
                        """pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        # Move data pointer
                        counter_i += 1"""
                        
                        break

                    if front_diff > threshold and back_diff < threshold:
                        print("checkpoint 11", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[counter_data][0]
                        sync_y = data_unsync[counter_data][1]
                        sync_z = data_unsync[counter_data][2]
                        sync.append([sync_x, sync_y, sync_z])
                        # Move position pointer
                        """pos_i += 1
                        if pos_i > std_size - 1:
                            pos_i = 0
                        # Move data pointer
                        counter_i += 1"""
                        
                        break

                    if front_diff >= threshold and back_diff >= threshold:
                        print("checkpoint 12", unsync_buffer.data[pos_j_front], 
                                unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        false_counter += 1
                        if false_counter == tolerance:
                            print("checkpoint12")
                            print(sync)
                            print(false_counter)
                            print(pos_j_front, pos_j_back, pos_i)
                            print(unsync_buffer.data, std_buffer.data)
                            sys.exit('imu data loss.')
                        else:  
                            # t_std moves and check if the error appears again
                            # Move data pointer
                            """counter_i += 1
                            # Move position pointer
                            pos_i += 1
                            if pos_i > std_size - 1:
                                pos_i = 0"""
                            
                            break

                if unsync_front < std_buffer.data[pos_i] and unsync_back < std_buffer.data[pos_i]:
                    # Both unsync smaller than t_std, no need for further comparisons temporally, break
                    print("checkpoint13")
                    print(sync)
                    print(false_counter)
                    print(pos_j_front, pos_j_back, pos_i)
                    print(unsync_buffer.data, std_buffer.data)
                    sys.exit('Finish computing.')

    elif i >= len(t_std):
        # t_std reaches the end, freeze t_std, pos_i should be [len-1]
        std = std_buffer.data[pos_i]
        print("inside", std)

        while True:

            if unsync_buffer.data[pos_j_front] > std and unsync_buffer.data[pos_j_back] > std:
                # the last t_std smaller than current t_unsync, stop computing
                print("checkpoint14")
                print(sync)
                print(false_counter)
                print(pos_j_front, pos_j_back, pos_i, i)
                print(unsync_buffer.data, std_buffer.data)

                sys.exit('Finish computing.')

            if unsync_buffer.data[pos_j_front] < std and unsync_buffer.data[pos_j_back] > std:
                front_diff = std - unsync_buffer.data[pos_j_front]
                back_diff = unsync_buffer.data[pos_j_back] - std
                diff = unsync_buffer.data[pos_j_back] - unsync_buffer.data[pos_j_front]
                front_scale = back_diff / diff
                back_scale = front_diff / diff

                if front_diff < threshold and back_diff < threshold:
                    print("checkpoint 15", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    sync_x = front_scale * data_unsync[counter_data][0] + back_scale * data_unsync[counter_data+1][0]
                    sync_y = front_scale * data_unsync[counter_data][1] + back_scale * data_unsync[counter_data+1][1]
                    sync_z = front_scale * data_unsync[counter_data][2] + back_scale * data_unsync[counter_data+1][2]
                    sync.append([sync_x, sync_y, sync_z])
                    
                    break  # Already given eq-value to the last std, no need for further computations

                if front_diff < threshold and back_diff > threshold:
                    print("checkpoint 16", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    sync_x = data_unsync[counter_data][0]
                    sync_y = data_unsync[counter_data][1]
                    sync_z = data_unsync[counter_data][2]
                    sync.append([sync_x, sync_y, sync_z])
                    
                    break

                if front_diff < threshold and back_diff > threshold:
                    print("checkpoint 17", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    sync_x = data_unsync[counter_data+1][0]
                    sync_y = data_unsync[counter_data+1][1]
                    sync_z = data_unsync[counter_data+1][2]
                    sync.append([sync_x, sync_y, sync_z])
                    
                    break

                if front_diff >= threshold and back_diff >= threshold:
                    print("checkpoint 18", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    false_counter += 1
                    if false_counter == tolerance:
                        print("checkpoint18")
                        print(sync)
                        print(false_counter)
                        print(pos_j_front, pos_j_back, pos_i)
                        print(unsync_buffer.data, std_buffer.data)
                        sys.exit('imu data loss.')
                    else:
                        
                        break

            if unsync_buffer.data[pos_j_front] < std and unsync_buffer.data[pos_j_back] < std:
                print("checkpoint 19", unsync_buffer.data[pos_j_front], 
                        unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                # Move unsync until we can get a comparison or run out of data
                if counter_j < len(t_unsync) - 1:
                    pos_j_front += 1
                    pos_j_back += 1
                    if pos_j_front > unsync_size - 1:
                        pos_j_front = 0
                    if pos_j_back > unsync_size - 1:
                        pos_j_back = 0
                    # Move data pointer
                    counter_j += 1
                    counter_data += 1
                    unsync_buffer.append(t_unsync[counter_j])  # Append the last t_unsync to the buffer
                    
                
                elif counter_j >= len(t_unsync) - 1:
                    # Freeze the last unsync ,make a final comparison
                    front = unsync_buffer.data[pos_j_front]
                    back = unsync_buffer.data[pos_j_back]

                    if front > std and back > std:
                        print("checkpoint 20", unsync_buffer.data[pos_j_front], 
                            unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        break

                    if front < std and back > std:
                        front_diff = std - front
                        back_diff = std - back
                        diff = back - front
                        front_scale = back_diff / diff
                        back_scale = front_diff / diff

                        if front_diff < threshold and back_diff < threshold:
                            print("checkpoint 20", unsync_buffer.data[pos_j_front], 
                                unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                            sync_x = front_scale * data_unsync[counter_data-1][0] + back_scale * data_unsync[counter_data][0]
                            sync_y = front_scale * data_unsync[counter_data-1][1] + back_scale * data_unsync[counter_data][1]
                            sync_z = front_scale * data_unsync[counter_data-1][2] + back_scale * data_unsync[counter_data][2]
                            sync.append([sync_x, sync_y, sync_z])
                            
                            break

                        if front_diff < threshold and back_diff > threshold:
                            print("checkpoint 21", unsync_buffer.data[pos_j_front], 
                                unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                            sync_x = data_unsync[counter_data-1][0]
                            sync_y = data_unsync[counter_data-1][1]
                            sync_z = data_unsync[counter_data-1][2]
                            sync.append([sync_x, sync_y, sync_z])
                            
                            break
                            
                        if front_diff > threshold and back_diff < threshold:
                            print("checkpoint 22", unsync_buffer.data[pos_j_front], 
                                unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                            sync_x = data_unsync[counter_data][0]
                            sync_y = data_unsync[counter_data][1]
                            sync_z = data_unsync[counter_data][2]
                            sync.append([sync_x, sync_y, sync_z])
                            
                            break

                        if front_diff >= threshold and back_diff >= threshold:
                            print("checkpoint 23", unsync_buffer.data[pos_j_front], 
                                    unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                            false_counter += 1
                            if false_counter == tolerance:
                                print("checkpoint23")
                                print(sync)
                                print(false_counter)
                                print(pos_j_front, pos_j_back, pos_i)
                                print(unsync_buffer.data, std_buffer.data)
                                sys.exit('imu data loss.')
                            else:
                                
                                break

                    if front < std and back < std:
                        print("checkpoint24")
                        print(sync)
                        print(false_counter)
                        print(pos_j_front, pos_j_back, pos_i)
                        print(unsync_buffer.data, std_buffer.data)
                        sys.exit('Finish computing.')


print(sync)
print(false_counter)
print(pos_j_front, pos_j_back, pos_i)
print(unsync_buffer.data, std_buffer.data)





                







            












     
            
    

