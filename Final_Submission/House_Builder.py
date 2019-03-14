#!/usr/bin/env python

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


def house_coordinates(x, y, z, width, height):
# x was set to 0.5 (dist of table from deniro) and z was 0.1 (height of table)
# coordinates are x, y, z and width, height

    t = 0.06  #thickness
    w = 0.09  #width
    h = 0.2  #height

    # bottom of table is (0,0,0)

    # one list of all positions in order
    list_of_positions = []

    # number of bricks in each layer
    layers = [5, 4, 4, 3, 3, 2, 2, 1]

    # height determines number of layers
    if (height*1000)%((h*1000)+(t*1000)) < 200:
        # this structure's height will be divisible by h+t + 1
        number_of_layers = int(height/(h+t))*2
    else:
        # this structure's height will be divisible by (h+t)
        number_of_layers = int(height/(h+t))*2 + 1
    # width determines which element in layers to start from
    if  int((width*10)/(h*10)) == 4:
        #start from 1st element
        layers = layers
    elif int((width*10)/(h*10)) == 3:
        #start from 3rd element
        del layers[0:2]
    elif int((width*10)/(h*10)) == 2:
        #start from 5th element
        del layers[0:4]
    elif int((width*10)/(h*10)) == 1:
        #start from 7th element
        del layers[0:6]

    print(number_of_layers)
    print(layers)
    # when width is smaller than height
    if len(layers) < number_of_layers:
        number_of_layers = len(layers)

    #coefficients for alternating brick picking from middle
    coefficient_odd = [0, 1.1, -1.1, 2.2, -2.2]
    coefficient_even = [-0.55, 0.55, -1.65, 1.65]

    #count layers
    count_layer = 1

    # height determines which element in layer we end on from base layer
    for i in range(number_of_layers):

        #list of coordinates for this specific layer
        layer_list = []

        # iterate through layers
        current_layer = layers[i]

        # count bricks
        count_brick = 1

        # iterate through bricks in layer
        for x in range(current_layer):


            #if layer is ODD NUMBER OF BRICKS
            if current_layer%2 == 1:

                y = coefficient_odd[count_brick-1]*h

                z= int(((count_layer)/2))*(h+t) + h*((count_layer)%2) - 0.03

                layer_list.append((0,y,z))

                count_brick += 1

            # if layer is EVEN NUMBER OF BRICKS
            elif current_layer%2 == 0:

                y= coefficient_even[count_brick-1]*h

                z= int(((count_layer)/2))*(h+t) + h*((count_layer)%2) - 0.03

                layer_list.append((0,y,z))

                count_brick = count_brick + 1


        # move on to next layer
        count_layer = count_layer + 1

        list_of_positions.append(layer_list)
    print(list_of_positions)
    return list_of_positions

def posify(Coordinates, v_orientation, h_orientation):

    #if block_poses[i][j] = int
    for i in range(len(Coordinates)):
        for j in range(len(Coordinates[i])):
            if i % 2:
                p =  Coordinates[i][j][0]
                u =  Coordinates[i][j][1]
                v =  Coordinates[i][j][2]
                Coordinates[i][j] =(Pose(
                        position=Point(x = p, y = u, z = v),
                        orientation=h_orientation))
            else:
                p =  Coordinates[i][j][0]
                u =  Coordinates[i][j][1]
                v =  Coordinates[i][j][2]
                Coordinates[i][j] =(Pose(
                        position=Point(x = p, y = u, z = v),
                        orientation=v_orientation))

    print (Coordinates)
    return Coordinates
