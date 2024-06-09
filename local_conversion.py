import numpy as np
import geodetic_functions as gf
import matplotlib.path as mpltPath


# NOTE: selected points must be vertices in a convex quadrilateral. The angle relative to the vertex chosen as origin
# must be between 0 and 90 degrees.


#######################################################################################################################
#   This function sorts the vertices in counter-clockwise order following the perimeter.
#   The first parameter "O" is the chosen origin.
#   Input: Operation area's vertices in geodetic coordinates with the first param the chosen origin,
#   Output: Sorted vertices NED coordinates.
#######################################################################################################################
def ccwsort(O, P2, P3, P4):
    P1ned = np.array([[0, 0]], dtype=float)  # Define point chosen as origin
    P2ned = np.array(gf.ll2ne(O, P2), dtype=float).reshape(1, 2)  #
    P3ned = np.array(gf.ll2ne(O, P3), dtype=float).reshape(1, 2)  # Remaining points in NED coordinates
    P4ned = np.array(gf.ll2ne(O, P4), dtype=float).reshape(1, 2)  #

    # Initialising vertex array
    V = [P2ned, P3ned, P4ned]

    # Looking for furthest vertex element from origin
    diagonal = V[0]
    left = V[0]
    right = V[0]
    down = np.array([0, 0, 1], dtype=float)

    for i in range(len(V)):
        # Computing perpendicular direction given the diagonal between origin and vertex i
        OVi = np.hstack((V[i], np.zeros((1, 1))))  # extended to 3d
        perpendicular = np.cross(OVi, down)[:, 0:2]

        count = 0
        for j in range(len(V)):
            proj = np.matmul(perpendicular, V[j].transpose())
            if proj > 0 and i != j:
                count += 1

        # counting the number of remaining vertex above the previously computed direction "OVi" (along "perpendicular")
        if count == 0:
            left = V[i]
        elif count == 1:
            diagonal = V[i]
        elif count == 2:
            right = V[i]

    # rearranging vertices in counter-clockwise order
    V = [P1ned, right, diagonal, left]

    return V


#######################################################################################################################
#   Given four points in lat/lon coordinates, the function returns the size of the smallest rectangular box containing
#   the quadrilateral. The first parameter "O" is the chosen origin. Height and width are expressed in meters.
#   Input: Operation area's vertices in geodetic coordinates with the first param the chosen origin,
#          boolean for the axis alignment
#   Output: Circumscribed rectangular map size in meters
#######################################################################################################################
def point2size(O, P2, P3, P4, x_aligned=True):
    V = ccwsort(O, P2, P3, P4)
    # defining local axes
    (x, y) = point2versor(O, P2, P3, P4, x_aligned)

    # computing height and width of the box
    height = 0
    width = 0

    for i in range(len(V)):
        w_temp = np.matmul(x, V[i].transpose())
        h_temp = np.matmul(y, V[i].transpose())

        if w_temp > width:
            width = w_temp[0, 0]
        if h_temp > height:
            height = h_temp[0, 0]

    return height, width


#######################################################################################################################
#   This function returns the NED coordinates values of the vertices of the box containing the working area.
#   Input: Operation area's vertices in geodetic coordinates with the first param the chosen origin,
#          boolean for the axis alignment
#   Output: Circumscribed rectangular map vertices in NED coordinates
#######################################################################################################################
def boundpoints(O, P2, P3, P4, x_aligned=True):
    (height, width) = point2size(O, P2, P3, P4, x_aligned)

    # defining local axes
    (x, y) = point2versor(O, P2, P3, P4, x_aligned)

    V1 = np.array([[0, 0]], dtype=float)
    V2 = width * x
    V3 = width * x + height * y[:, 0:2]
    V4 = height * y[:, 0:2]

    V = [V1, V2, V3, V4]

    return V


#######################################################################################################################
#   Check on the angle relative to the origin vertex
#   Inputs: Operation area's vertices in geodetic coordinates with the first param the origin to check
#   Output: Boolean value to determine if the vertex can be chosen as origin of the local frame
#######################################################################################################################
def isproperorigin(O, P2, P3, P4):
    V = ccwsort(O, P2, P3, P4)

    left = V[1]
    right = V[3]

    if np.matmul(left, right.transpose()) >= 0:
        return True
    else:
        return False


################################################################################################################
#   This function computes local axis versors in NED coordinates
#   Input: Operation area's vertices in geodetic coordinates with the first param the chosen origin,
#          boolean for the axis alignment
#   Output: Local axis versors in NED coordinates
################################################################################################################
def point2versor(O, P2, P3, P4, x_aligned=True):
    V = ccwsort(O, P2, P3, P4)
    down = np.array([0, 0, 1], dtype=float)

    if x_aligned:
        x = V[3] / np.linalg.norm(V[3])
        x_3d = np.hstack((x, np.zeros((1, 1))))  # extended to 3d
        y = np.cross(down, x_3d)[:, 0:2]

    else:
        y = V[1] / np.linalg.norm(V[1])
        y_3d = np.hstack((y, np.zeros((1, 1))))  # extended to 3d
        x = np.cross(y_3d, down)[:, 0:2]

    return x, y


################################################################################################################
# This function checks if the given point is inside the operation area
#   Input: Point in local coord. (point = np.array([[P0,P1]])
#          Array with sorted polygon vertices in local coord. (polygon = [[V00,V01],[V10,V11],[V20,V21],[V30,V31]])
#   Output: Boolean expression (True = point is inside polygon , False = point is outside polygon)
################################################################################################################
def ispointinside(point, polygon):
    path = mpltPath.Path(polygon)
    return path.contains_points(point)


################################################################################################################
# This function computes the operation area vertices in local coordinates
#   Input: Operation area's vertices in geodetic coordinates with the first param the chosen origin,
#          boolean for the axis alignment
#   Output: Array with sorted polygon vertices in local coord. (polygon = [[V00,V01],[V10,V11],[V20,V21],[V30,V31]])
################################################################################################################
def point2polygon(O, P1, P2, P3, x_aligned=True):
    V_ned = ccwsort(O, P1, P2, P3)
    x, y = point2versor(O, P1, P2, P3, x_aligned)
    R_ned2local = np.array([x[0], y[0]])
    O_local = np.matmul(R_ned2local, V_ned[0].transpose())
    P1_local = np.matmul(R_ned2local, V_ned[1].transpose())
    P2_local = np.matmul(R_ned2local, V_ned[2].transpose())
    P3_local = np.matmul(R_ned2local, V_ned[3].transpose())
    polygon = [O_local, P1_local, P2_local, P3_local]

    return polygon


################################################################################################################
# This function sorts given operation area waypoint in geodetic coord in order to obtain
# a rectangular map with minimum circumscribed area
#   Input:  Operation area's vertices in geodetic coordinates
#   Output: Sorted operation's area vertices in geodetic coordinates array with first elem the origin
#           Boolean for the axis alignment
################################################################################################################
def vertici_area_minima(WP1, WP2, WP3, WP4):
    V = [WP1, WP2, WP3, WP4]
    surface = np.Inf
    O = V[0]
    is_x_aligned = True
    O_index = 0

    for i in range(len(V)):
        O_i = V[i]
        V.pop(i)
        if isproperorigin(O_i, V[0], V[1], V[2]):
            (h_i_y_align, w_i_y_align) = point2size(O_i, V[0], V[1], V[2], x_aligned=False)  # y oriented
            (h_i_x_align, w_i_x_align) = point2size(O_i, V[0], V[1], V[2], x_aligned=True)  # x oriented
            surface_i_y_align = h_i_y_align * w_i_y_align
            surface_i_x_align = h_i_x_align * w_i_x_align
            if surface_i_y_align < surface:
                O = O_i
                O_index = i
                is_x_aligned = False
                surface = surface_i_y_align
            if surface_i_x_align < surface:
                O = O_i
                O_index = i
                is_x_aligned = True
                surface = surface_i_x_align
        V.insert(i, O_i)
    V.pop(O_index)
    V_ordinato = [O, V[0], V[1], V[2]]  # origin first element
    return V_ordinato, is_x_aligned