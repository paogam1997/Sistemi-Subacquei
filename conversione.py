#!/usr/bin/env python
import rospy                                     # importo tutte le librerie richieste per utility e definizione dei messaggi
from temperature_msgs.msg import LocalNavStatus
from marta_msgs.msg import NavStatus
import numpy as np
import math as m
import geodetic_functions as gf                 # file delle funzioni di conversione dato dai dottorandi
import local_conversion as lc                   # file di funzioni utili per definizione della zona di lavoro



def nav_callback(coo):  
    ##########################################################################################################################
    #                                                    ESTRAZIONE DATI                                                     #
    ##########################################################################################################################
    # estraggo i dati di navigazione in arrivo da Zeno (SIMULA_LAT_LONG nelle simulazioni)
    time_geod = coo.header.stamp      # salvo il tempo a cui e' arrivato il messaggio dal sistema di navigazione
    long = coo.position.longitude     # (lambda) longitudine attuale
    lat = coo.position.latitude       # (phi) latitudine attuale
    lat_long = np.array([lat, long])  # impilo questi ultimi due dati per creare un array
    h = coo.position.depth            # profondita' attuale
    vx_ned = coo.ned_speed.x          # velocita' lungo asse x della terna NED
    vy_ned = coo.ned_speed.y          # velocita' lungo asse y della terna NED


    ##########################################################################################################################
    #                                          EFFETTUO CONVERSIONE IN TERNA LOCAL                                           #
    ##########################################################################################################################
    # Usiamo la funzione di conversione da coordinate geodetiche a NED fornitaci dai dottorandi (salvata in sript gf)
    # primo argomento e' il centro scelto per terna NED (analogo del centro per local), segue vettore di latitudine e longitudine da convertire
    P_ned = gf.ll2ne(lat_long_0, lat_long)   # conversione della posizione letta nel frame NED
    P_local = np.dot(R_NED2local, P_ned)     # converto la posizione NED in assi local tramite apposita matrice di rotazione R_NED2local
    # converto in local anche la velocita' letta (da NED)
    vel_ned = np.array([vx_ned,vy_ned])      # impilo le velocita' lette per creare un array
    vel_local = np.dot(R_NED2local,vel_ned)  # ruoto in terna local la velocita' da NED come fatto in precedenza con la posizione
    # estraggo le componenti da trasmettere
    x = P_local[0]                           # prima componente del vettore posizione (P) in terna local(_local)
    y = P_local[1]                           # seconda componente del vettore posizione (P) in terna local(_local)
    vx = vel_local[0]                        # prima componente del vettore velocita' (vel) in terna local(_local)
    vy = vel_local[1]                        # seconda componente del vettore velocita' (vel) in terna local(_local)


    ##########################################################################################################################
    #                                             DEFINIZIONE RUOLO DA PUBLISHER                                             #
    ##########################################################################################################################
    # Definizione del publisher del topic POS_VEL_LOCAL, messaggio di tipo custom LocalNavStatus
    pub = rospy.Publisher('POS_VEL_LOCAL', LocalNavStatus, queue_size=10)
    local = LocalNavStatus()         # Definizione del tipo di messaggio
    local.header.stamp = time_geod   # header lo riempio con il tempo a cui la misura di Zeno e' stata trasmessa
    local.position.x = x             # posizione attuale lungo asse x del frame local
    local.position.y = y             # posizione attuale lungo asse y del frame local
    local.orientation.yaw = m.atan2(np.sin(coo.orientation.yaw - yaw), np.cos(coo.orientation.yaw - yaw))  # orientazione in local
    local.speed.x = vx               # velocita' attuale lungo asse x del frame local
    local.speed.y = vy               # velocita' attuale lungo asse x del frame local
    # convertito in local
    local.omega_body.x = coo.omega_body.x           # twist in body
    local.omega_body.y = coo.omega_body.y
    local.omega_body.z = coo.omega_body.z
    local.gps_status = coo.gps_status
    local.initialized = coo.initialized
    # nel campo frame trasmetto per ridondanza le info sul frame local creato e salvate anche nel parameter server
    local.frame.origin.latitude = lat_long_0[0]     # latitudine dell'origine del frame
    local.frame.origin.longitude = lat_long_0[1]    # longitudine dell'origine del frame
    local.frame.euler.yaw = yaw                     # angolo di rotazione tra frame NED e local
    # Definizione del messaggio da pubblicare
    pub.publish(local)
    # Mando a schermo cosa pubblico (per debug, poi verra' commentato per non intasare il terminale)
    rospy.loginfo(local)
    # rospy.loginfo("AUV sta in %s,%s al tempo %s e si sta muovendo con vel (%s,%s)", x,y,time_geod,vx,vy)


###############################################################################################################################
#                                           CREAZIONE NODO                                                                    #
###############################################################################################################################

def conversione(): 
    # Creo il nodo CONVERSIONE
    rospy.init_node("CONVERSIONE",anonymous=True)
    # Definisco il ruolo da subscriber al nav_status pubblicato da Zeno
    sub = rospy.Subscriber('nav_status',NavStatus,callback=nav_callback)  
    # Mando a schermo un avviso per assicurarmi che il nodo sia partito correttamente 
    rospy.loginfo("Nodo Conversione Avviato")  

    rospy.spin()  # analogo al rospy.sleep per il while, tiene attiva la definizione del nodo e le operazioni fatte nella callback


###############################################################################################################################
#                                                MAIN                                                                         #
###############################################################################################################################
# Notare che tutte le funzioni definite sopra sono richiamate a fine del main, vengono quindi eseguite solo per ultime
if __name__=='__main__':
    # Importo i way-point definiti nel parameter server, sezione /conversione, l'ordine in cui sono definiti e' casuale
    WP1 = rospy.get_param('/conversione/WP1')
    WP2 = rospy.get_param('/conversione/WP2')
    WP3 = rospy.get_param('/conversione/WP3')
    WP4 = rospy.get_param('/conversione/WP4')
    # Definisco le variabili come globali dovendo essere usate anche fuori dal main
    global V, lat_long_0,yaw,lato_x,lato_y


    ##########################################################################################################################
    #                                 CALCOLI PRELIMINARI PER DEFINIRE LA ZONA DI ESPLORAZIONE                               #
    ##########################################################################################################################
    # Stampa una riga di separazione con 10 - a lato ed al centro la parte tra " "
    print("-"*10 +  "CALCOLO DELL'AREA DI LAVORO" + "-"*10)

    # Riordino vertici in geodetiche per ottenere il vertice origine per la terna NED e local 
    V, is_x_aligned = lc.vertici_area_minima(WP1, WP2, WP3, WP4)
    # V = vettore dei vertici ordinati in senso antiorario a partire da origine
    # is_x_allined dice se allinea il primo asse a x o meno
    lat_long_0 = V[0]                                     # salvo la origine in coordinate geodetiche in una varaibile apposita
    rospy.set_param('/conversione/origine', lat_long_0)   # salvo origine anche nel parameter server
    # Mando a schermo le coo dell'origine per debug pre-missione
    print("L'origine della terna locale ha coordinate:\n\tO = (" + str(lat_long_0[0]) + " deg, " + str(lat_long_0[1]) + " deg) ")

    # Calcolo i versori della terna locale in coordinate NED
    # ingressi: vertici in geodetiche (ordine random tranne il primo che deve essere l'origine), se allineare primo asse a x o no
    x_ver, y_ver = lc.point2versor(V[0], V[1], V[2], V[3], is_x_aligned)  
    # Calcolo le dimensioni della mappa in terna locale
    lato_y, lato_x = lc.point2size(V[0], V[1], V[2], V[3], is_x_aligned)  
    rospy.set_param('/conversione/lato_y', float(lato_y))               # Salvo nel parameter server i lato_x e lato_y
    rospy.set_param('/conversione/lato_x', float(lato_x))
    # Mando a schermo le dimensioni della mappa per debug pre-missione
    print("La mappa ha dimensioni:\n\t(lato_x, lato_y) = (" + str(lato_x) + " m, " + str(lato_y) + " m) ")

    # Calcolo l'orientazione della terna locale rispetto terna NED (con stessa origine)
    yaw = m.atan2(x_ver[0, 1], x_ver[0, 0])  
    rospy.set_param('/conversione/yaw', float(yaw))                     # Salvo l'angolo di rotazione anche nel parameter server
    # Mando a schermo la info per il debug pre-missione
    print("La mappa e' ruotata rispetto a terna NED di:\n\tyaw = " + str(yaw) + " rad")

    # Calcolo la matrice di cambio di coordinate da NED a local
    xver = x_ver.tolist()                            # Trasformo i versori da array a lista per essere impilati in una matrice
    yver = y_ver.tolist()
    R_NED2local = np.array([xver[0],yver[0]])        # Creazione della matrice di rotazione

    # Calcolo dei vertici dell'area di lavoro in terna locale
    polygon = lc.point2polygon(V[0], V[1], V[2], V[3], is_x_aligned)
    # Estraggo i vertici calcolati per trasformarli meglio tra poco da array a lista
    P0 = polygon[0]
    P1 = polygon[1]
    P2 = polygon[2]
    P3 = polygon[3]
    # Ora li re-impilo in una vera e propria matrice
    polygon = [P0[:,0].tolist(), P1[:,0].tolist(), P2[:,0].tolist(), P3[:,0].tolist()]
    # Salvo i vertici nel parameter server, sezione /conversione
    rospy.set_param('/conversione/poligono', polygon)
    # Mando a schermo i limiti dell'area di lavoro sia in coo. geodetiche che local per debug pre-missione
    print("I vertici dell' area di lavoro in terna locale sono:\n" +
          "\tP0 = (" + str(P0[0, 0]) + " m, " + str(P0[1, 0]) + " m)\n"
          "\tP1 = (" + str(P1[0, 0]) + " m, " + str(P1[1, 0]) + " m)\n"
          "\tP2 = (" + str(P2[0, 0]) + " m, " + str(P2[1, 0]) + " m)\n"
          "\tP3 = (" + str(P3[0, 0]) + " m, " + str(P3[1, 0]) + " m) ")

    print("I vertici dell' area di lavoro in geodetice sono:\n" +
          "\tP0 = (" + str(V[0][0]) + " deg, " + str(V[0][1]) + " deg)\n"
          "\tP1 = (" + str(V[1][0]) + " deg, " + str(V[1][1]) + " deg)\n"
          "\tP2 = (" + str(V[2][0]) + " deg, " + str(V[2][1]) + " deg)\n"
          "\tP3 = (" + str(V[2][0]) + " deg, " + str(V[3][1]) + " deg) ")

    # Calcolo i vertici del rettangolo minimo che circoscrive la zona di lavoro
    MapBoundsNed = lc.boundpoints(V[0], V[1], V[2], V[3], is_x_aligned)
    # Gli trasformo da array a lista e li converto in coo geodetiche (la funzione li restituirebbe in NED)
    B0 = gf.ne2ll(lat_long_0, MapBoundsNed[0].tolist()[0])
    B1 = gf.ne2ll(lat_long_0, MapBoundsNed[1].tolist()[0])
    B2 = gf.ne2ll(lat_long_0, MapBoundsNed[2].tolist()[0])
    B3 = gf.ne2ll(lat_long_0, MapBoundsNed[3].tolist()[0])
    # Mando a schermo i vertici del rettangolo minimo per debug pre-missione 
    print("I vertici del rettangolo circoscritto in geodetice sono:\n" +
          "\tB0 = (" + str(B0[0]) + " deg, " + str(B0[1]) + " deg)\n"
          "\tB1 = (" + str(B1[0]) + " deg, " + str(B1[1]) + " deg)\n"
          "\tB2 = (" + str(B2[0]) + " deg, " + str(B2[1]) + " deg)\n"
          "\tB3 = (" + str(B3[0]) + " deg, " + str(B3[1]) + " deg)")
    

    # Avvio la funzione principale per eseguire il nodo e tutte le sue funzionalita'
    conversione()
