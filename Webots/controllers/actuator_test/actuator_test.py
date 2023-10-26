from controller import Robot, Motor, PositionSensor
from math import pi, sin
import os
import numpy as np

# Get the current directory
current_directory = os.path.dirname(os.path.abspath(__file__))
# Specify the filename (change 'your_file.txt' to the actual filename)
filename = "q.txt"
# Create the full file path by joining the current directory and the filename
file_path = os.path.join(current_directory, filename)
# Initialize an empty list to store the rows
data = []
# Read the file and store its contents into the data list
try:
    with open(file_path, 'r') as file:
        for line in file:
            # Split each line into individual values using whitespace as the delimiter
            row_values = line.strip().split()
            
            # Convert the row values to float and append them to the data list
            data.append([float(value) for value in row_values])   
            
except FileNotFoundError:
    print(f"The file '{filename}' does not exist in the current directory.")
except IOError:
    print("An error occurred while trying to read the file.")
  
# Convert the data list into a numpy array
data_array = np.array(data)

# Open motorNames file and activate them for controller
filename = "motorNames.txt"

# Create the full file path by joining the current directory and the filename
file_path = os.path.join(current_directory, filename)

with open(file_path, 'r') as file:
    motorNum = len(file.readlines())
    
print("motorNum is: ", motorNum) 
robot = Robot()
motor = [0] * motorNum
#print(motorNum)
#i=0

with open(file_path, 'r') as file:
    for i in range(motorNum):
        motor[i] = robot.getDevice(file.readline().strip())


i = 0
discrete_time = 0
t = 0
#TIME_STEP = 50 # For Quasistatic
TIME_STEP = 30 # For ZMP



flag = 0

print("Here")    
while robot.step(TIME_STEP) != -1:

    if flag == 0:
        # Open motorNames file and activate them for controller
        filename = "q0.txt"
        
        # Create the full file path by joining the current directory and the filename
        file_path = os.path.join(current_directory, filename)
        
        with open(file_path, 'r') as file:
            fileLength = len(file.readlines())
            
        if fileLength != motorNum:
            print("Error - Incorrect length of initial pose file")
            
        with open(file_path, 'r') as file:
            for i in range(fileLength):
                motor[i].setPosition(float(file.readline().strip()))
                
        if t > 2:
            discrete_time = 0
            flag = 1

    
    else:
        #if discrete_time > 1/500:
            print(t)
            discrete_time = 0
            # Cycle throw row and then column
            if i < data_array.size/motorNum:
                for j in range(motorNum):
                    motor[j].setPosition(data_array[i, j])
                    #print(data_array[i, j])      
            i += 1
              
    
        
    #value = sensor.getValue()
    #print("Angle value is: ", angle)
    t += TIME_STEP / 1000.0
    discrete_time += TIME_STEP / 1000.0

    