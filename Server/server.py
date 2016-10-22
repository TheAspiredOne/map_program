# Avery Tan 
# Alden Tan

from minheap import MinHeap
from graph import Graph
from math import sqrt
import sys
import argparse
# the following line actually imports serial (use this instead of import serial)
import textserial
from time import time

coordinates = dict()
road = dict()
list_of_waypoints = list()
timeout_flag = False
first_iteration = True

def read_city_graph(filename):
    '''Returns an instance of UndirectedGraph as defined in graph.py'''
    city_edmonton = Graph()
    with open(filename) as MAP:
        for line in MAP:
            line = line.strip()
            line = line.split(',')
            if line[0] == 'V':
                city_edmonton.add_vertex(int(line[1])) #use second element as arguement
                lat = int(float(line[2])*100000) #converting lat to 100000ths of degree
                lon = int(float(line[3])*100000) #converting lon to 100000ths of degree
                coordinates[int(line[1])] = (lat,lon) #coordinates as lat and lon as x and y coordinates

            if line[0] == 'E':
                city_edmonton.add_edge(int(line[1]),int(line[2])) # use second and third element as arguement
                road[int(line[1]),int(line[2])] = line[3] # store road names as a separate dictionary

    return city_edmonton #returns our directed graph 


def cost_distance(u, v):
    '''Computes and returns the straight-line distance between the two
    vertices u and v.
    Args:
    u, v: The ids for two vertices that are the start and
    end of a valid edge in the graph.
    Returns:
    numeric value: the distance between the two vertices.
    '''
    curr = coordinates[u] #starting coordinates
    dest = coordinates[v] #destination coordinates
    cost = sqrt((dest[0] - curr[0])**2 + (dest[1] - curr[1])**2) #Euclidean distance calculation
    
    return cost


def least_cost_path(graph, start, dest, cost):
    '''
    Dijkstra's algorithm
    Find and return the least cost path in graph from start
    vertex to dest vertex.
    Efficiency: If E is the number of edges, the run-time is
    O( E log(E) ).
    Args:
    graph (Graph): The digraph defining the edges between the
    vertices.
    start: The vertex where the path starts. It is assumed
    that start is a vertex of graph.
    dest: The vertex where the path ends. It is assumed
    that start is a vertex of graph.
    cost: A function, taking the two vertices of an edge as
    parameters and returning the cost of the edge. For its
    interface, see the definition of cost_distance.
    Returns:
    list: A potentially empty list (if no path can be found) of
    the vertices in the graph. If there was a path, the first
    vertex is always start, the last is always dest in the list.
    Any two consecutive vertices correspond to some
    edge in graph.
    '''    
    reached = dict() #dictionary containing vertices that are already reached

    route = MinHeap() 
    
    route.add((start, start), 0) #initial starting condition
    
    #dijkstras algorithm used to find the least cost path
    while route._array: 
        least_value = route.pop_min() 
        min_cost = least_value[1] 
        prev = least_value[0][0]
        curr = least_value[0][1]
        
        if curr not in reached:
            reached[curr] = prev
            
            for i in g.neighbours(curr):
                distance = cost_distance(curr, i)
                route.add((curr, i), min_cost + distance)
    
    s_dest = dest
    waypoints = list()
        
    if s_dest not in reached: #if no path, then return an empty list
        return []
        
    while s_dest != start: 
        waypoints.append(s_dest) #appending the previous vertices to the waypoints list
        s_dest = reached[s_dest] # mkaing previous vertices current vertices

    waypoints.append(start) 

    waypoints.reverse() # reverse the list containing our waypoints so that start is first item and dest is last item
    
    return waypoints


def parse_args():
    """
    Parses arguments for this program.
    
    Returns:
    An object with the following attributes:
     serialport (str): what is after -s or --serial on the command line
    """
    # try to automatically find the port
    port = textserial.get_port()
    if port==None:
        port = "0"

    parser = argparse.ArgumentParser(
          description='Serial port communication testing program.'
        , epilog = 'If the port is 0, stdin/stdout are used.\n'
        )
    parser.add_argument('-s', '--serial',
                        help='path to serial port '
                             '(default value: "%s")' % port,
                        dest='serialport',
                        default=port)

    return parser.parse_args()


def data_transfer(g,serial_in,serial_out):
    '''Code that does communication between Arduino and the server
    
    Args:
        g : our graph of edmonotn
        serial_in: stream that this function is reading from
        serial_out: stream that this function writes to
    '''
    first_iteration = None
    timeout_flag = None
    while(True):
        line = serial_in.readline() #read in the transmitted message
        line = line.rstrip('\r\n') # remove trailing newline
        
        user_input = line.split() # split them into an array
        

        s = dict() # dictionaries used for finding closest vertex
        d = dict()


        if user_input[0] == 'R' and (len(user_input) == 5): #processing data from the client
            xStart = (int(user_input[1]))
            yStart = (int(user_input[2]))
            destX = (int(user_input[3]))
            destY = (int(user_input[4]))


            #determining the closest vertex to the point chosen by the client
            
            for i in g.vertices():
                if (coordinates[i][0] > (xStart -6000)) and (coordinates[i][0]<(xStart + 6000)) and (coordinates[i][1] > (yStart -6000)) and (coordinates[i][1]<(yStart + 6000)) :

                    a = sqrt((coordinates[i][0] - xStart)**2 + (coordinates[i][1] - yStart)**2)
                    s[i] = a
                    start_vertex = min(s, key=s.get)

            for i in g.vertices():
                if (coordinates[i][0] > (destX -6000)) and (coordinates[i][0]<(destX + 6000)) and (coordinates[i][1] > (destY -6000)) and (coordinates[i][1]<(destY + 6000)) :
                    b = abs(coordinates[i][0] - destX) + abs(coordinates[i][1] - destY)
                    d[i] = b
                    dest_vertex = min(d, key=d.get)


            list_of_waypoints = least_cost_path(g,start_vertex,dest_vertex,cost = 0) #calcualte the shortest path
            number_of_waypoints = (len(list_of_waypoints)) #number of waypoints

            print('N',number_of_waypoints, file=serial_out)

            first_iteration = True
            timeout_flag = False

            counter = 0
            while counter < number_of_waypoints: #send our all the waypoints
                counter = counter + 1

                if timeout_flag == True:
                    break

                while True:

                    current_time = time() #recording the starting time of this while loop
                    if timeout_flag == True:
                        break

                    if first_iteration == True:
                        first_iteration = False #if its the first iteration, the next iteration will not be
                        if time() - current_time > 15:
                            timeout_flag = True

                    elif time() - current_time >1:
                        timeout_flag = True


                    line = serial_in.readline()
                    line = line.rstrip('\r\n') # remove trailing newline

                    if (line == "A"):
                        current_waypoint = list_of_waypoints.pop(0)
                        print("W",coordinates[current_waypoint][0],coordinates[current_waypoint][1],file=serial_out)
                        break
            while True:
                #if list is empty or there is no path to from start to destination
                if timeout_flag:
                    print("Timeout")
                    break
                
                current_time = time()
                
                line = serial_in.readline()
                line = line.rstrip('\r\n') # remove trailing newline
                
                
                if line[0] == "A":

                    print("E", file = serial_out)
                    break

                if time() - current_time > 1:
                    timeout_flag = True


if __name__=="__main__":

    g = read_city_graph('edmonton-roads-2.0.1.txt')

    args = parse_args()
            
    if args.serialport!="0":
        print("Opening serial port: %s" % args.serialport)
        baudrate = 9600 # [bit/seconds] 115200 also works
        with textserial.TextSerial(args.serialport,baudrate,newline=None) as ser:
            data_transfer(g,ser,ser)
            
    else:
        print("No serial port. Using stdin/stdout.")
        data_transfer(sys.stdin,sys.stdout)
