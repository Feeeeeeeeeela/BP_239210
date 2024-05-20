#Soubor: OPCUACobot.py
#Autor: Filip Plachý
#Datum 09.04.2024

from opcua import Client, ua
from pymycobot.mycobot import MyCobot
from math import pi, cos, sin, sqrt
from numpy import matrix, arctan2
from numpy.linalg import multi_dot
from time import sleep, time
from random import randint



# Inicializace kontroleru
# (port, baud - modulační rychlost)
mc = MyCobot("/dev/ttyAMA0", 1000000)



#Čtení proměnné ze OPCUA serveru (datablock OPCUA_DB z PLC)
#(node_id {str})
def read_value(node_id):
    node_id = 'ns=3;s="OPCUA_DB".' + node_id
    client_node = client.get_node(node_id)
    client_node_value = client_node.get_value()
    return client_node_value



#Zápis do proměnné OPCUA serveru (datablock OPCUA_DB z PLC)
#(node_id {str}, value {float,bool,int}, type {'float','bool','int'})
def write_value(node_id, value, type):
    node_id = 'ns=3;s="OPCUA_DB".' + node_id
    client_node = client.get_node(node_id)
    if type == "float":
        client_node_dv = ua.DataValue(ua.Variant(value, ua.VariantType.Float))
    elif type == "bool":
        client_node_dv = ua.DataValue(ua.Variant(value, ua.VariantType.Boolean))
    elif type == "int":
        client_node_dv = ua.DataValue(ua.Variant(value, ua.VariantType.Int16))
    client_node.set_value(client_node_dv)


#Provedení výpočtu transformace pro režim 1 - MANALL
#vrací seznam v kartézských souřadnicích
def transformation():
    theta = [read_value(f"MANALL_angle"+str(x)) for x in range(1, 7)]

    A6 = [0, 0, 0, 0, 0, 0]
    a = [0, -110.4, -96, 0, 0, 0]
    alpha = [0.5 * pi, 0, 0, 0.5 * pi, -0.5 * pi, 0]
    d = [131.22, 0, 0, 63.4, 75.05, 45.6]
    delta = [0, -0.5 * pi, 0, -0.5 * pi, 0.5 * pi, 0]
    for x in range(0, 6):
        A = matrix(
            [
                [
                    cos(theta[x] + delta[x]),
                    -sin(theta[x] + delta[x]) * cos(alpha[x]),
                    sin(theta[x] + delta[x]) * sin(alpha[x]),
                    a[x] * cos(theta[x] + delta[x]),
                ],
                [
                    sin(theta[x] + delta[x]),
                    cos(theta[x] + delta[x]) * cos(alpha[x]),
                    -cos(theta[x] + delta[x]) * sin(alpha[x]),
                    a[x] * sin(theta[x] + delta[x]),
                ],
                [0, sin(alpha[x]), cos(alpha[x]), d[x]],
                [0, 0, 0, 1],
            ]
        )
        A6[x] = A

    T = multi_dot(A6)
    R6 = T[0:3, 0:3]
    rx = (arctan2(R6[2, 1], R6[2, 2])) * 180 / pi
    ry = (arctan2(-R6[2, 0], sqrt(R6[2, 1] ** 2 + R6[2, 2] ** 2))) * 180 / pi
    rz = (arctan2(R6[1, 0], R6[0, 0])) * 180 / pi
    coords = [T[0, 3], T[1, 3], T[2, 3], rx, ry, rz]
    for coord in coords:
        round(coord, 2)
    return coords


#Funkce pro inicializaci automatického režimu - zjištění počáteční pozice kostky
def initial_pos():
    if read_value("AUTO_Kostka1") == 0:
        write_value("AUTO_Pozice", randint(1, 4), "int")
    else:
        write_value("AUTO_Pozice", read_value("AUTO_Kostka1"), "int")



    
#Automatický režim: stepper propojený se stepperem v PLC
#princip: PLC zašle příkaz, po vykonání robot přepne na další step
#(move (bool) - set pro vykonání pohybu,
#firstpick (bool) - robot zvedá nebo pokládá,
#position_number (int))
def auto_stepper(move, firstpick, position_number):
    #Statické pozice
    if position_number == 1:
        position_list=([-28 - 90, -12, -113, 44, 2, 0],[-28 - 90, -20, -120, 58, 2, 0])
        in_position_list=(
                        [-28 - 45, -22, -100, 90, 0, 0],
                        [-28 - 45, -22, -100, 90, 0, 0],
                        [-28, -22, -100, 90, 0, 0],
                        [-28 + 45, -22, -100, 90, 0, 0],
                    )
    if position_number == 2:
        position_list = ([-28, -12, -113, 44, 2, 0], [-28, -20, -120, 58, 2, 0])
        in_position_list = (
                        [-28 - 45, -22, -100, 90, 0, 0],
                        [-28 + 45, -22, -100, 90, 0, 0],
                        [-28 + 45, -22, -100, 90, 0, 0],
                        [-28 + 90, -22, -100, 90, 0, 0],
                    )   
    if position_number == 3:
        position_list = (
                        [-28 + 90, -12, -113, 44, 2, 0],
                        [-28 + 90, -20, -120, 58, 2, 0],
                    )
        in_position_list = (
                        [-28, -22, -100, 90, 0, 0],
                        [-28 + 45, -22, -100, 90, 0, 0],
                        [-28 + 45, -22, -100, 90, 0, 0],
                        [-28 + 135, -22, -100, 90, 0, 0],
                    )        
    if position_number == 4:
        position_list =  (
                        [-28 + 180, -12, -113, 44, 2, 0],
                        [-28 + 180, -20, -120, 58, 2, 0],
                    )
        in_position_list = (
                        [-28 + 45, -22, -100, 90, 0, 0],
                        [-28 + 90, -22, -100, 90, 0, 0],
                        [-28 + 135, -22, -100, 90, 0, 0],
                        [-28 + 135, -22, -100, 90, 0, 0],
                    )


    def step0():
        write_value("AUTO_Next0", False, "bool")
        if move:
            mc.send_angles(position_list[0], read_value("speed"))
            move = False
        if mc.is_in_position(position_list[0], 0) == 1:
            move = True
            write_value("AUTO_Next1", True, "bool")

    def step1():
        write_value("AUTO_Next1", False, "bool")
        if move:
            mc.send_angles(position_list[1], read_value("speed"))
            move = False
        if mc.is_in_position(position_list[1], 0) == 1:
            if read_value("AUTO_Kostka1") == 0:
                write_value("AUTO_Vakuum", True, "bool")
                for x in range(1, 5):
                    if position_number == x:
                        write_value("AUTO_Kostka1", x, "int")
                pickup = True
            else:
                pickup = False
                write_value("AUTO_Vakuum", False, "bool")
                write_value("AUTO_Kostka1", 0, "int")
            sleep(0.5)
            move = True
            write_value("AUTO_Next2", True, "bool")

    def step2():
        write_value("AUTO_Next2", False, "bool")
        if move:
            mc.send_angles(position_list[0], read_value("speed"))
            move = False
        if mc.is_in_position(position_list[0], 0) == 1:
            write_value("AUTO_Next3", True, "bool")
            move = True

    def step3():
        write_value("AUTO_Next3", False, "bool")
        if firstpick:
            if pickup:
                for x in range(1, 5):
                    if position_number == x:
                        next_position = x
                pickup = False
            else:
                next_position = randint(1, 4)
            firstpick = False

        for x in range(1, 5):
            if position_number == x:

                if position_number == x:
                    if move:
                        mc.send_angles(in_position_list[next_position-1], read_value("speed"))
                        move = False
                    if mc.is_in_position(in_position_list[next_position - 1], 0) == 1:
                        write_value("AUTO_Startstav", True, "bool")
                        write_value("AUTO_Pozice", next_position, "int")

    
    if read_value("AUTO_Step0") == True:
        step0()

    if read_value("AUTO_Step1") == True:
        step1()

    if read_value("AUTO_Step2") == True:
        step2()

    if read_value("AUTO_Step3") == True:
        step3()


#Real time manual režim
#ovládání pohybu jednotlivých kloubů -pomocí neustálého kontrolování úhlu
#(číslo kloubu)
def MANRT(joint_int):
    if read_value("MANRT_Change"):
        write_value("MANRT_Value", read_value("MANRT_angle" + str(joint_int)), "int")
        write_value("MANRT_Change", False, "bool")
    mc.send_angle(1, read_value("MANRT_Value"), read_value("speed"))
    write_value("MANRT_angle" + str(joint_int), read_value("MANRT_Value"), "int")


#JOG manual režim
#ovládání pohybu jednotlivých kloubů - pomocí jog funkce robota
#(číslo kloubu)
def jog_mode(joint_int):
    if read_value("JOG_plus"):
        mc.jog_angle(joint_int, 1, read_value("speed"))
        write_value("JOG_move0", True, "bool")
    elif read_value("JOG_minus"):
        mc.jog_angle(joint_int, 0, read_value("speed"))
        write_value("JOG_move0", True, "bool")

    else:
        z = joint_int - 1
        if read_value("JOG_move" + str(z)):
            mc.jog_stop()
            write_value("JOG_move" + str(z), False, "bool")


#Hlavní funkce programu
#while loop, který reaguje na proměnné ze serveru
#určuje aktuální režim pohybu robota (automatický, manuální na tlačítko, real time manuální pomocí stupňů nebo jogu, nastavení)
#provádění režimů - reagování na změnu uživatelem
#přerušením dochází k ukončení celého programu (zrušení spojení OPCUA - přerušením nebo uživatelem)
def main():
    while True:
        try:
            if not read_value("opcua_status"):
                break

            if read_value("mode") == 0:

                if read_value("AUTO_Start") == True:
                    initial_pos()
                    max_time = 10
                    write_value("AUTO_Running", True, "bool")
                    write_value("AUTO_Startstav", True, "bool")
                    write_value("AUTO_Start", False, "bool")

                if read_value("AUTO_Running"):
                    if read_value("AUTO_Startstav"):
                        write_value("AUTO_Next0", True, "bool")
                        start_time = time()
                        move = True
                        firstpick = True
                        write_value("AUTO_Startstav", False, "bool")

                    current_time = time()
                    time_diff = current_time - starttime
                    if time_diff >= max_time:
                        move = True
                        print("Restarting movement")
                        start_time = time()

                    if not (read_value("AUTO_Konec") or read_value("AUTO_Pause")):
                        auto_stepper(move,
                            firstpick,
                            read_value("AUTO_Pozice"))

                    if read_value("AUTO_Pause"):
                        print("Pause")
                        while mc.is_paused() != 1:
                            mc.pause()
                            break
                        while True:
                            angles = mc.get_angles()
                            if angles != []:
                                for x in range(0, 6):
                                    write_value("AUTO_angle" + str(x), angles[x], "int")
                            break
                        write_value("AUTO_Paused", True, "bool")
                        write_value("AUTO_Pause", False, "bool")

                    if read_value("AUTO_Resume"):
                        print("Resume")
                        mc.resume()
                        write_value("AUTO_Paused", False, "bool")
                        write_value("AUTO_Resume", False, "bool")

                    if read_value("AUTO_Konec"):
                        print("Konec")
                        mc.resume()
                        mc.send_angles([0, 0, 0, 0, 0, 0], read_value("speed"))
                        write_value("AUTO_Running", False, "bool")
                        write_value("AUTO_Paused", False, "bool")
                        write_value("AUTO_Konec", False, "bool")

            if read_value("mode") == 1:

                if read_value("MAN_Move") == 1:

                    coords = transformation()
                    for x in range(0, 6):
                        write_value("MAN_axis" + str(x), coords[x], "float")
                    write_value("MAN_Move", False, "bool")

                    mc.send_angles(coords, read_value("speed"))

            if read_value("mode") == 2:

                if read_value("MANRT_Angle") == 0:
                    MANRT(1)

                if read_value("MANRT_Angle") == 1:
                    MANRT(2)

                if read_value("MANRT_Angle") == 2:
                    MANRT(3)

                if read_value("MANRT_Angle") == 3:
                    MANRT(4)

                if read_value("MANRT_Angle") == 4:
                    MANRT(5)

                if read_value("MANRT_Angle") == 5:
                    MANRT(6)

            if read_value("mode") == 3:

                jogangles = mc.get_angles()
                if jogangles != []:
                    for x in range(0, 6):
                        y = x + 1
                        write_value("JOG_angle" + str(y), jogangles[x], "int")

                if read_value("JOG_Angle") == 0:
                    jog_mode(1)

                if read_value("JOG_Angle") == 1:
                    jog_mode(2)

                if read_value("JOG_Angle") == 2:
                    jog_mode(3)

                if read_value("JOG_Angle") == 3:
                    jog_mode(4)

                if read_value("JOG_Angle") == 4:
                    jog_mode(5)

                if read_value("JOG_Angle") == 5:
                    jog_mode(6)

            if read_value("mode") == 10:

                angles = mc.get_angles()
                if angles != []:
                    for x in range(1, 7):
                        y = x - 1
                        write_value("Set_angle" + str(x), angles[y], "int")

                if read_value("Set_ReleaseServos") == True:
                    mc.release_all_servos()
                    write_value("Set_CalibrationEnable", True, "bool")
                    write_value("Set_ReleaseServos", False, "bool")

                if read_value("Set_SetServos") == True:
                    mc.send_angles(angles, 50)
                    write_value("Set_CalibrationEnable", False, "bool")
                    write_value("Set_SetServos", False, "bool")

                if read_value("Set_Calibration") == True:
                    for x in range(1, 7):
                        mc.set_servo_calibration(x)
                    write_value("Set_Calibration", False, "bool")

            if read_value("disconnect") == True:
                write_value("opcua_status", False, "bool")
                print("Disconnect")
                client.disconnect()
                break

        except:
            write_value(
                "opcua_status",
                False,
                "bool",
            )
            break


"""
Inicializace OPCUA
připojení k OPCUA serveru - PLC
"""
try:
    client = Client("opc.tcp://192.168.0.10:4840")
    client.session_timeout = 30000
    client.connect()
    root = client.get_root_node()
    write_value("opcua_status", True, "bool")
    print("Objects node is: ", root)
    print("Children of root are: ", root.get_children())
    print("Running")
except:
    print("OPCUA inicialization failed")

"""
Volání hlavní funkce
"""
main()
