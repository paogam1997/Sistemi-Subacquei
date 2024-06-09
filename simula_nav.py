#!/usr/bin/env python
import rospy                                      # importo tutte le librerie richieste per utility e definizione dei messaggi
from marta_msgs.msg import NavStatus
import random

###################################################################################################################
#                                     CREAZIONE NODO                                                              #
###################################################################################################################

def talker():                                      # definisco la funzione principale da avviare nel main
    # creazione del nodo
    rospy.init_node('SIMULA_LAT_LONG',anonymous=True)
    # creazione del ruolo da publisher el NavStatus simulato di Zeno
    pub = rospy.Publisher('nav_status',NavStatus,queue_size=10)
    # definisco il rate a cui deve girare il while 
    rate = rospy.Rate(rospy.get_param('/simula_nav/rate_talker'))  # Zeno fornisce il suo stato circa ogni 10 Hz

    while not rospy.is_shutdown():  # cosa fa il publisher:
        # genera numeri random uniformi per latitudine, longitudine [deg] e profondita' [m]
        lat = random.uniform(rospy.get_param('/simula_nav/min_lat'), rospy.get_param('/simula_nav/max_lat'))     # lat sta circa in [ 43.7025, 43.7047] (WP frniteci)
        long = random.uniform(rospy.get_param('/simula_nav/min_long') ,rospy.get_param('/simula_nav/max_long'))  # long sta circa in [10.4627 ,10.56392 ] (WP forniteci)
        h = rospy.get_param('/simula_nav/h')   # lasciato sempre a 0, non si fa inabissare Zeno
        # genero in modo random anche la velocita' definita in terna NED [m/s], sempre minore di 1 m/s
        vx_ned = random.uniform(rospy.get_param('/simula_nav/vx_ned_min'),rospy.get_param('/simula_nav/vx_ned_max'))
        vy_ned = random.uniform(rospy.get_param('/simula_nav/vy_ned_min'),rospy.get_param('/simula_nav/vx_ned_max')) 
        # creo il messaggio di stato di navigazione (stesso che usa Zeno per comunicare)
        msg = NavStatus()                     # definizione del tipo di messaggio
        msg.position.latitude = lat           # nel campo latitude di position salvo la latitudine creata
        msg.position.longitude = long         # nel campo longitude di position salvo la longitudine creata
        msg.position.depth = h                # nel campo depht di position salvo la profondita' creata
        msg.ned_speed.x = vx_ned              # nel campo x di ned_speed salvo la velocita' lungo x creata
        msg.ned_speed.y = vy_ned              # nel campo y di ned_speed salvo la velocita' lungo y creata
        msg.header.stamp = rospy.Time.now()   # l'header del messaggio si riempie con il tempo a cui il messaggio viene trasmesso
        pub.publish(msg)                      # pubblico il messaggio
        #rospy.loginfo(msg)                   # mando a schermo il messaggio creato
       
        rate.sleep()      # comando permette di far eseguire il ciclo while non subito appena si completa una iterazione ma alla freq specificata nel rate

####################################################################################################
#                                                MAIN                                              #
####################################################################################################

if __name__ == '__main__':
    talker()           # richiamo la funzione talker definita sopra per essere eseguita
