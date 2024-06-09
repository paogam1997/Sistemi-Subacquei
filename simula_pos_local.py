#!/usr/bin/env python
import rospy                           # importo tutte le librerie richieste per utility e definizione dei messaggi
from marta_msgs.msg import NavStatus
import random

# ============== Questo era un nodo vecchio usato per controllare che il nodo di ============== #
#  ============= sensing_ambiente funzionasse a dovere nella generazione della T ============== #

def talker():  # Definisco il ruolo da talker
    # Inizializzo il nodo
    rospy.init_node('SENSORE_POSIZIONE',anonymous=True)  # SENSORE_POSIZIONE era il vecchio nome del nodo CONVERSIONE
    # Creazione del publisher
    # POSIZIONE e' stato sostituito con POS_VEL_LOCAL ed anche il type ora e' aggiornato a
    # LocalNavStatus, analogo a NavStatus ma coi nomi corretti ed una sezione nuova per le info sul frame local
    pub = rospy.Publisher('POSIZIONE', NavStatus,queue_size=10)
    # Imposto la frequenza di pubblicazione pari a quella di Zeno (circa)
    rate=rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():  # Cosa fa il publisher
        # Genera 2 numeri random per la posizione dentro una area di lavoro scelta
        x=random.uniform(0,123)
        y=random.uniform(0,756)
        # Genera 2 numeri random per la velocita' di Zeno
        vx_ned = random.uniform(0,1)
        vy_ned = random.uniform(0.1,0.5) 
        # Creo il messaggio (vecchio tipo)
        msg=NavStatus()
        msg.position.latitude=x  # in local
        msg.position.longitude=y # in local
        msg.position.depth=0
        msg.ned_speed.x=vx_ned
        msg.ned_speed.y=vy_ned
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo(msg)  # stampa a schermo

        rate.sleep()

        
if __name__=='__main__':
    talker()
