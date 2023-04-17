"""TimeSync but in ringbuffer implementation

Errors still exists in some special scenarios, to be debugged.

Updated date: 14/04/23 by ychen441
"""

import sys
import numpy as np
from RingBufferFuncs import RingBuffer

# Hand-calculated data for test
#t_std = np.array([1, 2, 3, 4, 8, 10])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.4, 4.8, 5, 10.1, 10.5])

#t_std = np.array([1, 2, 3, 4, 8, 10])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.4, 4.8, 5, 9.95, 10.05])

#t_std = np.array([1, 2, 3, 4, 8, 10])
#t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9, 4.2, 4.4, 4.8, 5, 9.95, 9.98])

t_std = np.array([1, 2, 3, 4, 8, 10, 11, 12, 13, 14])
t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.2, 3.9])





data_unsync = np.array([[3, 0, 0], [3, 0, 0], [1, 0, 0], [1, 0, 0], [27, 0, 0],
                 [26, 0, 0], [50, 0, 0], [2, 0, 0], [1, 0, 0], [5, 0, 0], [1, 0, 0], [22, 0, 0], [20, 0, 0]])

std_size = 5  # Size of t_std buffer
unsync_size = 5  # Size of t_unsync buffer
std_buffer = RingBuffer(std_size)
unsync_buffer = RingBuffer(unsync_size)

#count_i = 0
end_counter = 0
count_j = 0  # Pointer for fetching t_unsync
count_data = 0  # Pointer for fetching data_unsync
pos_i = 0  # Position pointer for std_buffer
pos_j_front = 0  # Front pointer for unsync_buffer
pos_j_back = 0  # Back pointer for unsync_buffer  

sync = []  # Store synced data
threshold = 0.1
false_counter = 0  # Out-of-bound counter
false_max = 3
loop_counter = 0

for i in range(0, len(t_std)+1):
    

    if i < len(t_std):
        
        std_buffer.append(t_std[i])

    # Freeze t_std position if reach the end   
    else:
        
        std = std_buffer.data[pos_i]
        
        #for j in range(count_j, len(t_unsync)+1):
        while True:
            count_data += 1
            if count_data > len(t_unsync) - 2:
                count_data = len(t_unsync) - 2
            
            
            if unsync_buffer.data[pos_j_front] > std and unsync_buffer.data[pos_j_back] > std:
                print("checkpoint21", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                break
            if unsync_buffer.data[pos_j_front] < std and unsync_buffer.data[pos_j_back] > std:
                front_diff = std - unsync_buffer.data[pos_j_front]
                back_diff = unsync_buffer.data[pos_j_back] - std
                diff = unsync_buffer.data[pos_j_back] - unsync_buffer.data[pos_j_front]
                front_scale = back_diff / diff
                back_scale = front_diff / diff

                if front_diff < threshold and back_diff < threshold:
                    print("checkpoint22", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    sync_x = front_scale * data_unsync[count_data][0] + back_scale * data_unsync[count_data+1][0]
                    sync_y = front_scale * data_unsync[count_data][1] + back_scale * data_unsync[count_data+1][1]
                    sync_z = front_scale * data_unsync[count_data][2] + back_scale * data_unsync[count_data+1][2]
                    sync.append([sync_x, sync_y, sync_z])
                    break

                if front_diff > threshold  and back_diff < threshold:
                    print("checkpoint23", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    sync_x = data_unsync[count_data+1][0]
                    sync_y = data_unsync[count_data+1][1]
                    sync_z = data_unsync[count_data+1][2]
                    sync.append([sync_x, sync_y, sync_z])
                    break

                if front_diff < threshold  and back_diff > threshold:
                    print("checkpoint24", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    
                    sync_x = data_unsync[count_data][0]
                    sync_y = data_unsync[count_data][1]
                    sync_z = data_unsync[count_data][2]
                    sync.append([sync_x, sync_y, sync_z])
                    break

                if front_diff >= threshold  and back_diff >= threshold:
                    print("checkpoint25", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    false_counter += 1
                    if false_counter == false_max:
                        sys.exit('accelerometer error.')
                    else:
                        pos_j_front += 1
                        pos_j_back += 1
                        # Overwrite the oldest element
                        if pos_j_front > unsync_size - 1:
                            pos_j_front = 0
                        if pos_j_back > unsync_size - 1:
                            pos_j_back = 0
                        continue

            if unsync_buffer.data[pos_j_front] < std and unsync_buffer.data[pos_j_back] < std:
                print("checkpoint26", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                
                count_j += 1
                pos_j_front += 1
                pos_j_back += 1
                # Overwrite the oldest element
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                if count_j >= len(t_unsync):
                    if unsync_buffer.data[pos_j_front] == t_unsync[len(t_unsync) - 2] and unsync_buffer.data[pos_j_back] == t_unsync[len(t_unsync)-1] and std == std_buffer.data[pos_i]:
                        loop_counter += 1
                        if loop_counter > 1:
                            print(sync)
                            print(false_counter)
                            print(pos_j_front, pos_j_back, pos_i)
                            print(unsync_buffer.data, std_buffer.data)
                            sys.exit('accelerometer error.')
                        else:
                            continue
                else:
                    unsync_buffer.append(t_unsync[count_j])
                

        continue


    #for k in range(0, len(t_unsync)):
    while True:
        
        if pos_j_front == 0 and pos_j_back == 0:
            
            
    
            unsync_buffer.append(t_unsync[count_j])
            count_j += 1
            pos_j_back += 1
            if pos_j_back > unsync_size - 1:
                pos_j_back = 0 
            unsync_buffer.append(t_unsync[count_j])
            print("checkpoint", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
            break

        if unsync_buffer.data[pos_j_front] > std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] > std_buffer.data[pos_i]:
            print("checkpoint2", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
            count_j += 1
            pos_i += 1
            # Overwrite the oldest element
            if pos_i > std_size - 1:
                pos_i = 0
            unsync_buffer.append(t_unsync[count_j])
            break

        if unsync_buffer.data[pos_j_front] < std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] > std_buffer.data[pos_i]:
            
            # Parameters for interpolation
            front_diff = std_buffer.data[pos_i] - unsync_buffer.data[pos_j_front]
            back_diff = unsync_buffer.data[pos_j_back] - std_buffer.data[pos_i]
            diff = unsync_buffer.data[pos_j_back] -unsync_buffer.data[pos_j_front]
            front_scale = back_diff / diff
            back_scale = front_diff / diff

            if front_diff < threshold and back_diff < threshold:
                print("checkpoint3", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                sync_x = front_scale * data_unsync[count_data][0] + back_scale * data_unsync[count_data+1][0]
                sync_y = front_scale * data_unsync[count_data][1] + back_scale * data_unsync[count_data+1][1]
                sync_z = front_scale * data_unsync[count_data][2] + back_scale * data_unsync[count_data+1][2]
                sync.append([sync_x, sync_y, sync_z])
                count_data += 1
                count_j += 1
                pos_i += 1 
                pos_j_front += 1
                pos_j_back += 1
                # Overwrite the oldest element
                if pos_i > std_size - 1:
                    pos_i = 0
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                unsync_buffer.append(t_unsync[count_j])
                break

            if front_diff > threshold and back_diff < threshold:
                print("checkpoint4", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                sync_x = data_unsync[count_data+1][0]
                sync_y = data_unsync[count_data+1][1]
                sync_z = data_unsync[count_data+1][2]
                sync.append([sync_x, sync_y, sync_z])
                count_data += 1
                count_j += 1
                pos_i += 1 
                pos_j_front += 1
                pos_j_back += 1
                # Overwrite the oldest element
                if pos_i > std_size - 1:
                    pos_i = 0
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                unsync_buffer.append(t_unsync[count_j])
                break

            if front_diff < threshold and back_diff > threshold:
                print("checkpoint5", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                sync_x = data_unsync[count_data][0]
                sync_y = data_unsync[count_data][1]
                sync_z = data_unsync[count_data][2]
                sync.append([sync_x, sync_y, sync_z])
                count_data += 1
                count_j += 1
                pos_i += 1 
                pos_j_front += 1
                pos_j_back += 1
                # Overwrite the oldest element
                if pos_i > std_size - 1:
                    pos_i = 0
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                print(count_j)
                unsync_buffer.append(t_unsync[count_j])
                break

            if front_diff >= threshold and back_diff >= threshold:
                print("checkpoint6", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                false_counter += 1
                if false_counter == false_max:
                    sys.exit('accelerometer error.')
                else:
                    pos_i += 1
                    unsync_buffer.append(t_unsync[count_j])
                    break

        if unsync_buffer.data[pos_j_front] < std_buffer.data[pos_i] and unsync_buffer.data[pos_j_back] < std_buffer.data[pos_i]:
            count_j += 1

            if count_j > len(t_unsync):  # Freeze t_unsync position if reach the end
                
                front = unsync_buffer.data[pos_j_front]
                back = unsync_buffer.data[pos_j_back]       

                if front > std_buffer.data[pos_i] and back > std_buffer.data[pos_i]:
                    print("checkpoint7", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    pos_i += 1
                    # Overwrite the oldest element
                    if pos_i > std_buffer - 1:
                        pos_i = 0
                    break

                if front < std_buffer.data[pos_i] and back > std_buffer.data[pos_i]:
                    print("checkpoint8", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    # Parameters for interpolation
                    front_diff = std_buffer.data[pos_i] - front
                    back_diff = back - std_buffer.data[pos_i]
                    diff = back - front
                    front_scale = back_diff / diff
                    back_scale = front_diff / diff

                    if front_diff < threshold and back_diff < threshold:
                        print("checkpoint9", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        
                        sync_x = front_scale * data_unsync[count_data][0] + back_scale * data_unsync[count_data+1][0]
                        sync_y = front_scale * data_unsync[count_data][1] + back_scale * data_unsync[count_data+1][1]
                        sync_z = front_scale * data_unsync[count_data][2] + back_scale * data_unsync[count_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break

                    if front_diff > threshold and back_diff < threshold:
                        print("checkpoint10", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[count_data+1][0]
                        sync_y = data_unsync[count_data+1][1]
                        sync_z = data_unsync[count_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1 
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break

                    if front_diff < threshold and back_diff > threshold:
                        print("checkpoint11", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[count_data][0]
                        sync_y = data_unsync[count_data][1]
                        sync_z = data_unsync[count_data][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1 
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break


                    if front_diff >= threshold and back_diff >= threshold:
                        print("checkpoint12", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        false_counter += 1
                        if false_counter == false_max:
                            sys.exit('accelerometer error.')
                        else:
                            pos_i += 1
                            if pos_i > std_size - 1:
                                pos_i = 0
                            break
                    
                if front < std_buffer.data[pos_i] and back < std_buffer.data[pos_i]:
                    print("checkpoint13", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    pos_i += 1
                    # Overwrite the oldest element
                    if pos_i > std_size - 1:
                        pos_i = 0
                    break

            elif count_j == len(t_unsync):
                pos_j_front += 1
                pos_j_back += 1
                count_data += 1
                # Overwrite the oldest element
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                
                front = unsync_buffer.data[pos_j_front]
                back = unsync_buffer.data[pos_j_back]        

                if front > std_buffer.data[pos_i] and back > std_buffer.data[pos_i]:
                    print("checkpoint14", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    pos_i += 1
                    # Overwrite the oldest element
                    if pos_i > std_buffer - 1:
                        pos_i = 0
                    break

                if front < std_buffer.data[pos_i] and back > std_buffer.data[pos_i]:
                    print("checkpoint15", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                    # Parameters for interpolation
                    front_diff = std_buffer.data[pos_i] - front
                    back_diff = back - std_buffer.data[pos_i]
                    diff = back - front
                    front_scale = back_diff / diff
                    back_scale = front_diff / diff

                    if front_diff < threshold and back_diff < threshold:
                        print("checkpoint16", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = front_scale * data_unsync[count_data][0] + back_scale * data_unsync[count_data+1][0]
                        sync_y = front_scale * data_unsync[count_data][1] + back_scale * data_unsync[count_data+1][1]
                        sync_z = front_scale * data_unsync[count_data][2] + back_scale * data_unsync[count_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break

                    if front_diff > threshold and back_diff < threshold:
                        print("checkpoint17", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[count_data+1][0]
                        sync_y = data_unsync[count_data+1][1]
                        sync_z = data_unsync[count_data+1][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1 
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break

                    if front_diff < threshold and back_diff > threshold:
                        print("checkpoint18", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        sync_x = data_unsync[count_data][0]
                        sync_y = data_unsync[count_data][1]
                        sync_z = data_unsync[count_data][2]
                        sync.append([sync_x, sync_y, sync_z])
                        pos_i += 1 
                        # Overwrite the oldest element
                        if pos_i > std_size - 1:
                            pos_i = 0
                        break


                    if front_diff >= threshold and back_diff >= threshold:
                        print("checkpoint19", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                        
                        false_counter += 1
                        if false_counter == false_max:
                            sys.exit('accelerometer error.')
                        else:
                            pos_i += 1
                            if pos_i > std_size - 1:
                                pos_i = 0
                            break

            elif count_j < len(t_unsync):
                print("checkpoint20", unsync_buffer.data[pos_j_front], unsync_buffer.data[pos_j_back], std_buffer.data[pos_i])
                pos_j_front += 1
                pos_j_back += 1
                count_data += 1
                # Overwrite the oldest element
                if pos_j_front > unsync_size - 1:
                    pos_j_front = 0
                if pos_j_back > unsync_size - 1:
                    pos_j_back = 0
                unsync_buffer.append(t_unsync[count_j])


print(sync)
print(false_counter)
print(pos_j_front, pos_j_back, pos_i)
print(unsync_buffer.data, std_buffer.data)


        