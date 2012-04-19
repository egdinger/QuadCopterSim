from numpy import *

delta_t = 1 #in 100th's of a second

run_time = 5*100

time = 0
command = 0

#loads a command file, and converts the timestamps in seconds to
#timestamps in delta_t
def load_command_file(file, delta_conversion):
    command_data = loadtxt(file, delimiter=',')
    for i in range(len(command_data)):
        command_data[i][0] *= delta_conversion

    return command_data

run_data = load_command_file('./runs/r1.run',100)

while (time <= run_time):
    print (time)
    if run_data[command][0] <= time:
        print (run_data[command])
        if command < len(run_data-1):
            command += 1
    time += delta_t
