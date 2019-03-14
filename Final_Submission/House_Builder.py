#!/usr/bin/env python

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


def house_coordinates(x, y, z, width, height):

    '''
    This function takes in coordinates x, y, z to determine where
    the structure will be built from. It also takes in the width and
    height of the structure you want to build and outputs coordinates
    stored in a list for each layer within the greater list. The
    coordinates are output in the correct order, for each layer the
    first brick to be picked is the middle one for odd number of brick
    layers and the right hand side middle brick for even number of
    brick layers. The  right arm will always pick up the first brick
    of each layer.
    '''

    # maximum structure is 0.8m width and 1.04m height
    # see documentaion for baxter reachable workspace http://mfg.rethinkrobotics.com/wiki/Workspace_Guidelines#tab.3DBaxter

    if y > 0.8:
        print('You have input the maximum width!')

    if z > 1.04:
        print('You have input the maximum height!')


    t = 0.06  # thickness of brick
    w = 0.09  # width of brick
    h = 0.2  # height of brick

    # bottom of table is currently (0,0,0)

    # a list which will contain all positions in order, with lists in the list for each layer
    list_of_positions = []

    # a list for the number of bricks in each layer
    layers = [5, 4, 4, 3, 3, 2, 2, 1]

    # the structure will always begin with vertical bricks as the first layer to be built

    # height determines number of layers
    # if the height is divided by the height + thickness of the brick and the remainder< than 200 (brick height= 200mm), this means that there is not enough height to build an extra vertical brick layer so the height will be an even number of layers
    if (height*1000)%((h*1000)+(t*1000)) < 200:
        # this structure's height will be divisible by h+t
        # the number of layers is therefore equal to the integer value of (height/h+t)*2
        number_of_layers = int(height/(h+t))*2
    else:
        # this structure's height will be divisible by (h+t) + 1
        # if the remainder > 200, then the height is tall enough to fit another vertical brick layer and the number of layers is equal to the integer value of (height/h+t)*2 + 1
        # number of layers is equal to the integer value of (height/h+t)*2 + 1
        number_of_layers = int(height/(h+t))*2 + 1

    # width determines which element in layers to start from
    if  int((width*10)/(h*10)) == 4:
        #start from 1st element
        layers = layers
    elif int((width*10)/(h*10)) == 3:
        #start from 3rd element (delete first 2 elements)
        del layers[0:2]
    elif int((width*10)/(h*10)) == 2:
        #start from 5th element (delete first 4 elements)
        del layers[0:4]
    elif int((width*10)/(h*10)) == 1:
        #start from 7th element (delete first 6 elements)
        del layers[0:6]

    # number_of_layers is currently defined by the height of the structure, if the width of the structure is too small that it cannot build to that height this would be an invalid number of times to run the loop for
    # when the length of the layers list is smaller than the value of the number_of_layers (when width is smaller than height) then number_of_layers is equal to instead the length of layers
    if len(layers) < number_of_layers:
        print('The width of the structure you input is too small to build this high!')
        number_of_layers = len(layers)

    #coefficients for alternating brick picking from middle for odd and even brick layers
    #coefficients include the length of bricks and the 2cm buffer between bricks
    coefficient_odd = [0, 1.1, -1.1, 2.2, -2.2]
    coefficient_even = [-0.55, 0.55, -1.65, 1.65]

    #count layers
    count_layer = 1

    # iterate through each layer
    for i in range(number_of_layers):

        # list to store coordinates for this specific layer
        layer_list = []

        # value of current layer
        current_layer = layers[i]

        # count bricks
        count_brick = 1

        # iterate through bricks in this layer
        for j in range(current_layer):


            # if layer is ODD NUMBER OF BRICKS
            # layer will be odd if the current layer %2 = 1 and even if = 0
            if current_layer%2 == 1:

                # y is horizontal coordinate
                # indexing count_brick will assign the correct coefficient
                y = coefficient_odd[count_brick-1]*h

                # z is the vertical coordinate
                # calculate how many (h+t)s are in the structure and if an extra h needs to be added for the odd layers
                # -0.03 for the gripper position from top of brick
                z= int(((count_layer)/2))*(h+t) + h*((count_layer)%2) - 0.03

                #append coordinates into layer list
                layer_list.append((x,y,z))

                #move on to next brick
                count_brick += 1

            # if layer is EVEN NUMBER OF BRICKS
            elif current_layer%2 == 0:

                # y is horizontal coordinate
                # indexing count_brick will assign the correct coefficient
                y= coefficient_even[count_brick-1]*h

                # z is the vertical coordinate
                # calculate how many (h+t)s are in the structure and if an extra h needs to be added for the odd layers
                # -0.03 for the gripper position from top of brick
                z= int(((count_layer)/2))*(h+t) + h*((count_layer)%2) - 0.03

                #append coordinates into layer list
                layer_list.append((x,y,z))

                #move on to next brick
                count_brick = count_brick + 1


        # move on to next layer
        count_layer = count_layer + 1

        #append each layer list into greater list
        list_of_positions.append(layer_list)

    print(list_of_positions)
    return list_of_positions

def posify(Coordinates, v_orientation, h_orientation):
    '''
    This function onverts all the brick postitions to poses for baxter to read
    it takes in the two types of quaternions (vertical or horizontal) and sorts
    the poses out based on the type of later (horizontal or vertical)
    '''

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
