#!/usr/bin/env python
import rospy                           # importo tutte le librerie richieste per utility e definizione dei messaggi
from temperature_msgs.msg import LocalNavStatus
from sensor_msgs.msg import Temperature
import numpy as np


###################################################################################################################
#                                    CALLBACK  DA NODO CONVERSIONE                                                #
###################################################################################################################

def conversione_callback(pos): 
    # estraggo posizione, velocita' e tempo letti dal nodo CONVERSIONE
    # le devo definire come variabili globali dovendole usare fuori da questa funzione che svolge solo il ruolo di richiamo
    global x_ref,y_ref,flag_first_msg,vx,vy, time_geod   
    # posizione nel sistema di riferimento locale solidale alla zona di lavoro [m]
    x_ref = pos.position.x
    y_ref = pos.position.y
    # velocita' di Zeno in terna locale [m/s]
    vx=pos.speed.x
    vy=pos.speed.y 
    # tempo al quale Zeno ha comunicato la sua posizione, salvato nell' header del messaggio [s]
    time_geod=pos.header.stamp 
    # definisco una variabile Boleana di flag per discrimiare l'arrivo del primo messaggio
    if not flag_first_msg:
        flag_first_msg = True


###################################################################################################################
#                                     CREAZIONE NODO                                                              #
###################################################################################################################

def sensing_ambiente():  # funzione principale da eseguire nel main, contiene la definizione del nodo ed i suoi ruoli
    # definizione del ruolo da publisher di T letta 
    pub = rospy.Publisher('TEMPERATURA_LETTA',Temperature,queue_size=10)
    # creazione del nodo
    rospy.init_node("SENSING_AMBIENTE",anonymous=True)
    rate = rospy.Rate(rospy.get_param('/sensing/rate'))   # il rate di pubblicazione del sensore CTD vale 20 Hz
    # specifico il ruolo da subscriber al topic del nodo di conversione
    sub = rospy.Subscriber('POS_VEL_LOCAL',LocalNavStatus,callback=conversione_callback)
    
    # mando a schermo un avviso per assicurarmi che il nodo sia partito correttamente
    rospy.loginfo("Nodo Avviato")  # verra' commentato per evitare di intasare il terminale

    # richiamo delle variabili globali definite nella callback
    global x_ref, y_ref, time_geod, vx, vy,flag_first_msg  
    # inizializzo la variabile flag al valore false
    flag_first_msg = False 

    ################################################################################################################
    #                               DEFINIZIONE PARAMETRI MAPPA                                                    #
    ################################################################################################################
    # Parametri generali:
    # Grandezza mappa [m]
    lato_x = rospy.get_param('/conversione/lato_x')  
    lato_y = rospy.get_param('/conversione/lato_y')  
    # Limiti di Temperatura [C] 
    Tmin = rospy.get_param('/sensing/Tmin')
    Tmax = rospy.get_param('/sensing/Tmax')
    Delta_T = Tmax - Tmin 

    # Parametri sorgente sferica 1 # (alto a destra)
    # Posizione centro [m]
    xs1 = lato_x * rospy.get_param('/sensing/c_xs1')
    ys1 = lato_y * rospy.get_param('/sensing/c_ys1')  
    # [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
    sigma_sorg1 = lato_x * rospy.get_param('/sensing/c_sigma_sorg1')  
    
    # Parametri sorgente sferica  inversa 1.1 #
    # Posizione centro [m]
    ys1_inv = lato_y * rospy.get_param('/sensing/c_ys1_inv')
    xs1_inv = lato_x * rospy.get_param('/sensing/c_xs1_inv')  
    # [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
    sigma_sorg1_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg1_inv')  
    
    # Parametri sorgente sferica 2 # (basso a sinistra)
    # Posizione centro [m]
    xs2 = lato_x * rospy.get_param('/sensing/c_xs2')
    ys2 = lato_y * rospy.get_param('/sensing/c_ys2')  
    # [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
    sigma_sorg2 = lato_x * rospy.get_param('/sensing/c_sigma_sorg2')  

    # Parametri sorgente sferica inversa 2.1 #
    # Posizione centro [m]
    ys2_inv = lato_y * rospy.get_param('/sensing/c_ys2_inv')
    xs2_inv = lato_x * rospy.get_param('/sensing/c_xs2_inv')  
    # [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
    sigma_sorg2_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg2_inv')  

    # Parametri effetto riva 3 # (angolo basso a sinistra)
    # "larghezza" della gaussiana lungo cui si sviluppa la retta [m]
    dev_std_retta = rospy.get_param('/sensing/c_dev_std_retta') * lato_x
    # inclinazione della retta [adim]
    m = np.tan(np.pi / rospy.get_param('/sensing/c_m'))  

    # Parametri effetto riva inversa 3.1 #
    # "larghezza" della gaussiana lungo cui si sviluppa la retta [m]
    dev_std_retta_inv = rospy.get_param('/sensing/c_dev_std_retta_inversa') * lato_y
    # inclinazione della retta [adim]
    m_inv = np.tan(np.pi / rospy.get_param('/sensing/c_m_inv'))  
    
    # Parametri sorgente ellittica 4 # (alto-destra)
    # Posizione centro [m]
    xse1 = rospy.get_param('/sensing/c_xse1') * lato_x
    yse1 = rospy.get_param('/sensing/c_yse1') * lato_y  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_x1 = lato_x * rospy.get_param('/sensing/c_sigma_sorg_x1')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_y1 = lato_x * rospy.get_param('/sensing/c_sigma_sorg_y1')  

    # Parametri sorgente ellittica inversa 4.1 #
    # Posizione centro [m]
    yse1_inv = rospy.get_param('/sensing/c_yse1_inv') * lato_y
    xse1_inv = rospy.get_param('/sensing/c_xse1_inv') * lato_x  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_x1_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_x1_inv')
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_y1_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_y1_inv')  

    # Parametri sorgente ellittica 5 # (basso-sx)
    # Posizione centro [m]
    xse2 = rospy.get_param('/sensing/c_xse2') * lato_x
    yse2 = rospy.get_param('/sensing/c_yse2') * lato_y  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_x2 = lato_x * rospy.get_param('/sensing/c_sigma_sorg_x2') 
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_y2 = lato_x * rospy.get_param('/sensing/c_sigma_sorg_y2')  

    # Parametri sorgente ellittica inversa 5.1 #
    # Posizione centro [m]
    yse2_inv = rospy.get_param('/sensing/c_yse2_inv') * lato_y
    xse2_inv = rospy.get_param('/sensing/c_xse2_inv') * lato_x  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_y2_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_y2_inv')
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_x2_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_x2_inv')  

    # Parametri sorgente ellittica ruotata 6 # (alto-sx)
    # Posizione centro sorgente in assi fissi [m]
    x_c = lato_x * rospy.get_param('/sensing/c_x_c')
    y_c = lato_y * rospy.get_param('/sensing/c_y_c')  
    # [rad] Inclinazione assi locali rispetto ai fissi
    theta = np.pi / rospy.get_param('/sensing/c_theta') 
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_r_x = lato_x * rospy.get_param('/sensing/c_sigma_sorg_r_x')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_r_y = lato_x * rospy.get_param('/sensing/c_sigma_sorg_r_y')  

    # Parametri sorgente ellittica ruotata inversa 6.1 #
    # Posizione centro sorgente in assi fissi [m]
    y_c_inv = lato_y * rospy.get_param('/sensing/c_y_c_inv')
    x_c_inv = lato_x * rospy.get_param('/sensing/c_x_c_inv')  
    # [rad] Inclinazione assi locali rispetto ai fissi
    theta_inv = np.pi / rospy.get_param('/sensing/c_theta_inv')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_r_y_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_r_y_inv')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_r_x_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_r_x_inv')  

    # Parametri sorgente ellittica ruotata aggiuntiva 7 # 
    # Posizione centro sorgente in assi fissi [m]
    x_ca = lato_x * rospy.get_param('/sensing/c_x_ca')
    y_ca = lato_y * rospy.get_param('/sensing/c_y_ca')  
    # [rad] Inclinazione assi locali rispetto ai fissi
    theta_a = np.pi / rospy.get_param('/sensing/c_theta_a')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_ra_x = lato_x * rospy.get_param('/sensing/c_sigma_sorg_ra_x')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_ra_y = lato_x * rospy.get_param('/sensing/c_sigma_sorg_ra_y')  

    # Parametri sorgente ellittica ruotata aggiuntiva inversa 7.1 #
    # Posizione centro sorgente in assi fissi [m]
    y_ca_inv = lato_y * rospy.get_param('/sensing/c_y_ca_inv')
    x_ca_inv = lato_x * rospy.get_param('/sensing/c_x_ca_inv')  
    # [rad] Inclinazione assi locali rispetto ai fissi
    theta_a_inv = np.pi / rospy.get_param('/sensing/c_theta_a_inv')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
    sigma_sorg_ra_y_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_ra_y_inv')  
    # [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
    sigma_sorg_ra_x_inv = lato_y * rospy.get_param('/sensing/c_sigma_sorg_ra_x_inv')  

    ################################################################################################################
    #                                     DEFINIZIONE PESI                                                         #
    ################################################################################################################
    # prendiamo prima il caso di una zona di lavoro molto bassa e larga, (caso C_inv)
    # tutti i pesi con _inv saranno diversi da zero, gli altri invece sono nulli (le _inv sono le primitive specifiche per
    # il caso "basso e largo")  
    if lato_y / lato_x < 1 / 5:
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_Cinv')
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_Cinv')                             # Disattivata
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_Cinv')
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_Cinv')                             # Disattivata
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_Cinv')
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_Cinv')                         # Disattivata
            p_retta_inv = rospy.get_param('/sensing/retta_inv_Cinv')
            p_retta = rospy.get_param('/sensing/retta_Cinv')                              # Disattivata
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_Cinv')
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_Cinv')                       # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_Cinv')                             # Disattivata
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_Cinv')
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_Cinv')                   # Disattivata
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_Cinv')
        # prendiamo ora il caso di una zona di lavoro mediamente bassa e larga (caso B_inv)
    if 1 / 5 <= lato_y / lato_x < 1 / 2.4:
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_Binv')
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_Binv')                             # Disattivata
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_Binv')
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_Binv')                             # Disattivata
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_Binv')
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_Binv')                         # Disattivata
            p_retta_inv = rospy.get_param('/sensing/retta_inv_Binv')
            p_retta = rospy.get_param('/sensing/retta_Binv')                              # Disattivata
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_Binv')               # Disattivata
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_Binv')                       # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_Binv')                             # Disattivata
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_Binv')
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_Binv')                   # Disattivata
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_Binv')
        # prendiamo invece il caso di una zona di lavoro  bassa e larga anche se abbastanza squadrata (caso A_inv)
    if 1 / 2.4 <= lato_y / lato_x < 1:
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_Ainv')                     # Disattivata
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_Ainv')                             # Disattivata
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_Ainv')                     # Disattivata
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_Ainv')                             # Disattivata
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_Ainv')                 # Disattivata
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_Ainv')                         # Disattivata
            p_retta_inv = rospy.get_param('/sensing/retta_inv_Ainv')
            p_retta = rospy.get_param('/sensing/retta_Ainv')                              # Disattivata
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_Ainv')               # Disattivata
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_Ainv')                       # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_Ainv')                             # Disattivata
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_Ainv')
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_Ainv')                   # Disattivata
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_Ainv')

        # ora andiamo a considerare il caso in cui la zona di lavoro sia stretta e lunga (adesso tutti i _inv sono nulli e
        # solo i pesi originari vengono attivati):
        # iniziamo con il caso di zona di lavoro abbastanza squadrata (caso A)
    if 1 <= lato_y / lato_x < 2.4:
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_A')                                # Disattivata
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_A')                        # Disattivata
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_A')                                # Disattivata
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_A')                        # Disattivata
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_A')                            # Disattivata
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_A')                    # Disattivata
            p_retta = rospy.get_param('/sensing/retta_A')
            p_retta_inv = rospy.get_param('/sensing/retta_inv_A')                         # Disattivata
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_A')                          # Disattivata
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_A')                  # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_A')
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_A')                        # Disattivata
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_A')
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_A')              # Disattivata
        # passiamo al caso di zona di lavoro mediamente lunga e stretta (caso B)
    if 2.4 <= lato_y / lato_x < 5:
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_B')
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_B')                        # Disattivata
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_B')
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_B')                        # Disattivata
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_B')
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_B')                    # Disattivata
            p_retta = rospy.get_param('/sensing/retta_B')
            p_retta_inv = rospy.get_param('/sensing/retta_inv_B')                         # Disattivata
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_B')                          # Disattivata
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_B')                  # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_B')
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_B')                        # Disattivata
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_B')
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_B')              # Disattivata
        # concludiamo con il caso di zona di lavoro molto lunga e stretta (caso C)
    if lato_y / lato_x >= 5:
            p_sorg_sf1 = rospy.get_param('/sensing/sf1_C')
            p_sorg_sf1_inv = rospy.get_param('/sensing/sf1_inv_C')                        # Disattivata
            p_sorg_sf2 = rospy.get_param('/sensing/sf2_C')
            p_sorg_sf2_inv = rospy.get_param('/sensing/sf2_inv_C')                        # Disattivata
            p_sorg_ell_r = rospy.get_param('/sensing/ell_r_C')
            p_sorg_ell_r_inv = rospy.get_param('/sensing/ell_r_inv_C')                    # Disattivata
            p_retta = rospy.get_param('/sensing/retta_C')
            p_retta_inv = rospy.get_param('/sensing/retta_inv_C')                         # Disattivata
            p_sorg_ell_ra = rospy.get_param('/sensing/ell_ra_C')
            p_sorg_ell_ra_inv = rospy.get_param('/sensing/ell_ra_inv_C')                  # Disattivata
            p_sorg_ell = rospy.get_param('/sensing/ell_C')
            p_sorg_ell_inv = rospy.get_param('/sensing/ell_inv_C')                        # Disattivata
            p_sorg_ell_sost = rospy.get_param('/sensing/ell_sost_C')
            p_sorg_ell_sost_inv = rospy.get_param('/sensing/ell_sost_inv_C')              # Disattivata
        
    ##################################################################################################
    #                           DEFINIZIONE LETTURA SENSORE                                         #
    ##################################################################################################

    while not rospy.is_shutdown():  # manipolo gli ingressi per avere la T_meas da pubblicare, ciclo si attiva al rate specificato prima
        if not flag_first_msg:      # se non arriva una prima misura aspetta
            continue
            # altrimenti prosegui ...

        # devo propagare in avanti la misura di posizione, Zeno si sposta anche se il sistema di navigazione ancora non lo ha detto
        now = rospy.Time.now()                           #  salvo il tempo attuale
        delta_t = now.to_sec() -time_geod.to_sec()       # differenza tra istante attuale e tempo di ultima  misurazine
        x = x_ref+vx*delta_t                             # propgo la posizione
        y = y_ref+vy*delta_t
        # ora e' questa misura che metto nella mappa per trovare la T in (x,y) attuali      

        ##################################################################################################
        #                        CREAZIONE DELLE PRIMITIVE DELLE FUNZIONI DA USARE                       #
        ################################################################################################## 
        # Tutte le sorgenti danno un valore normalizzato [adim], sono riscalate in [C] quando si sommano in T
        # 1.Sorgente SFERICA PICCOLA 1 # (alto a destra)
        # (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
        T_sorg_sf1 = np.exp(-((x - xs1) / sigma_sorg1) ** 2 - ((y - ys1) / sigma_sorg1) ** 2)
         
        # 1.1 Sorgente SFERICA PICCOLA 1 inversa #
        # (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
        T_sorg_sf1_inv = np.exp(-((x - xs1_inv) / sigma_sorg1_inv) ** 2 - ((y - ys1_inv) / sigma_sorg1_inv) ** 2)

        # 2.Sorgente SFERICA PICCOLA 2 # (basso a sinistra)
        # (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
        T_sorg_sf2 = np.exp(-((x - xs2) / sigma_sorg2) ** 2 - ((y - ys2) / sigma_sorg2) ** 2)

        # 2.1 Sorgente SFERICA PICCOLA 2 inversa #
        # (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
        T_sorg_sf2_inv = np.exp(-((x - xs2_inv) / sigma_sorg2_inv) ** 2 - ((y - ys2_inv) / sigma_sorg2_inv) ** 2)

        # 3. RETTA INCLINATA # (angolo basso a sinistra)
        # (sono Gaussiane il cui massimo si sviluppa lungo una retta,
        # max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'esponenziale come una retta)       
        T_retta = np.exp(-((y - m * x - (0 - m * lato_x)) / dev_std_retta) ** 2)  # riva

        # 3.1 RETTA INCLINATA  inversa # (angolo basso a sinistra)
        # (sono Gaussiane il cui massimo si sviluppa lungo una retta,
        # max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'esponenziale come una retta)       
        T_retta_inv = np.exp(-((x - lato_x - 0.5*lato_y - m_inv * y - (0 - m_inv * lato_y)) / dev_std_retta_inv) ** 2)  # riva

        # 4. SORGENTE ELLITTICA # (alto-destra)
        #  e' una sferica con dev.stnd. diverse per x e y        
        T_sorg_ell = np.exp(-((x - xse1) / sigma_sorg_x1) ** 2 - ((y - yse1) / sigma_sorg_y1) ** 2)

        # 4.1 SORGENTE ELLITTICA inversa  # (alto-destra)
        #  e' una sferica con dev.stnd. diverse per x e y

        T_sorg_ell_inv = np.exp(-((x - xse1_inv) / sigma_sorg_x1_inv) ** 2 - ((y - yse1_inv) / sigma_sorg_y1_inv) ** 2)


        # 5. SORGENTE ELLITTICA SOSTITUTIVA DI REYLEIGH. # (basso-sx)
        # e' una sferica con dev.stnd. diverse per x e y       
        T_sorg_ell_sost = np.exp(-((x - xse2) / sigma_sorg_x2) ** 2 - ((y - yse2) / sigma_sorg_y2) ** 2)

        # 5.1 SORGENTE ELLITTICA SOSTITUTIVA DI REYLEIGH. inversa # (basso-sx)
        # e' una sferica con dev.stnd. diverse per x e y        
        T_sorg_ell_sost_inv = np.exp(-((x - xse2_inv) / sigma_sorg_x2_inv) ** 2 - ((y - yse2_inv) / sigma_sorg_y2_inv) ** 2)

        # 6.SORGENTE ELLITTICA RUOTATA #  (alto-sx)
        #  si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
        #  x,y --> sara' x_local, y_local che va definita la sorgente ellittica
        # Definisco le variabili locali in funzioni delle fisse
        x_local = x * np.cos(theta) + y * np.sin(theta)
        y_local = -x * np.sin(theta) + y * np.cos(theta)
        # Riporto il centro in coordinate locali
        x_c_local = x_c * np.cos(theta) + y_c * np.sin(theta)
        y_c_local = -x_c * np.sin(theta) + y_c * np.cos(theta)
        # Definisco l'ellisse nelle coordinate locali       
        T_sorg_ell_r = np.exp(-((x_local - x_c_local) / sigma_sorg_r_x) ** 2 - ((y_local - y_c_local) / sigma_sorg_r_y) ** 2)

        # 6.1 SORGENTE ELLITTICA RUOTATA inversa #  (alto-sx)
        #  si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
        #  x,y --> sara' x_local, y_local che va definita la sorgente ellittica       
        # Definisco le variabili locali in funzioni delle fisse
        x_local_inv = x * np.cos(theta_inv) + y * np.sin(theta_inv)
        y_local_inv = -x * np.sin(theta_inv) + y * np.cos(theta_inv)
        # Riporto il centro in coordinate locali
        x_c_local_inv = x_c_inv * np.cos(theta_inv) + y_c_inv * np.sin(theta_inv)
        y_c_local_inv = -x_c_inv * np.sin(theta_inv) + y_c_inv * np.cos(theta_inv)
        # Definisco l'ellisse nelle coordinate locali        
        T_sorg_ell_r_inv = np.exp(
            -((x_local_inv - x_c_local_inv) / sigma_sorg_r_x_inv) ** 2 - ((y_local_inv - y_c_local_inv) / sigma_sorg_r_y_inv) ** 2)

        # 7.SORGENTE ELLITTICA RUOTATA AGGIUNTIVA #
        # si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
        # x,y --> sara' x_local, y_local che va definita la sorgente ellittica       
        # Definisco le variabili locali in funzioni delle fisse
        x_local_a = x * np.cos(theta_a) + y * np.sin(theta_a)
        y_local_a = -x * np.sin(theta_a) + y * np.cos(theta_a)
        # Riporto il centro in coordinate locali
        x_c_local_a = x_ca * np.cos(theta_a) + y_ca * np.sin(theta_a)
        y_c_local_a = -x_ca * np.sin(theta_a) + y_ca * np.cos(theta_a)
        # Definisco l'ellisse nelle coordinate locali        
        T_sorg_ell_ra = np.exp(
            -((x_local_a - x_c_local_a) / sigma_sorg_ra_x) ** 2 - ((y_local_a - y_c_local_a) / sigma_sorg_ra_y) ** 2)

        # 7.1 SORGENTE ELLITTICA RUOTATA AGGIUNTIVA inversa #
        # si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
        # x,y --> sara' x_local, y_local che va definita la sorgente ellittica       
        # Definisco le variabili locali in funzioni delle fisse
        x_local_a_inv = x * np.cos(theta_a_inv) + y * np.sin(theta_a_inv)
        y_local_a_inv = -x * np.sin(theta_a_inv) + y * np.cos(theta_a_inv)
        # Riporto il centro in coordinate locali
        x_c_local_a_inv = x_ca_inv * np.cos(theta_a_inv) + y_ca_inv * np.sin(theta_a_inv)
        y_c_local_a_inv = -x_ca_inv * np.sin(theta_a_inv) + y_ca_inv * np.cos(theta_a_inv)
        # Definisco l'ellisse nelle coordinate locali       
        T_sorg_ell_ra_inv = np.exp(
            -((x_local_a_inv - x_c_local_a_inv) / sigma_sorg_ra_x_inv) ** 2 - ((y_local_a_inv - y_c_local_a_inv) / sigma_sorg_ra_y_inv) ** 2)

        ####################################################################################################
        #                           SOMMO PESANDOLI I CONTRIBUTI CHE MI SERVONO                            #
        ####################################################################################################

        # sommo tutti i contributi assieme, tanto sono scremati e pesati coi rispettivi pesi, [T]=C
        T = (Delta_T) * (p_sorg_sf1 * T_sorg_sf1 + p_sorg_sf2 * T_sorg_sf2 + p_retta * T_retta + p_sorg_ell * T_sorg_ell +
                        p_sorg_ell_r * T_sorg_ell_r + p_sorg_ell_sost * T_sorg_ell_sost + p_sorg_ell_ra * T_sorg_ell_ra +  # da ora sommo i termini per espanzione su x
                        p_sorg_sf1_inv * T_sorg_sf1_inv + p_sorg_sf2_inv * T_sorg_sf2_inv + p_retta_inv * T_retta_inv +
                        p_sorg_ell_inv * T_sorg_ell_inv + p_sorg_ell_r_inv * T_sorg_ell_r_inv +
                        p_sorg_ell_sost_inv * T_sorg_ell_sost_inv + p_sorg_ell_ra_inv * T_sorg_ell_ra_inv) + Tmin


        # definisco il rumore di misura
        dev_std_Tnoise = rospy.get_param('/sensing/valore_dev_std_Tnoise') # deviazione standard del rumore [C]
        noise = np.random.normal(loc=0,scale=dev_std_Tnoise)     # rumore bianco gaussiano a media nulla e dev.std. specificata
        # creo la misura del CTD da mandare
        T_meas_ = T + noise                      # si somma T vera e rumore
        T_meas = round(T_meas_ , 3)              # arrotondo a 3 cifre dopo la virgola per tener conto della risoluzione del CTD
        # la mando a schermo
        rospy.loginfo("La T letta vale %s",T_meas_)  # doppione del loginfo sottostante (anche questo va commentato per non intasare il terminale)
        #rospy.logwarn("delta_t vale %s",delta_t)    # mandate a schermo per debug 
        #rospy.logerr("now vale %s e geod %s", now.to_sec(),time_geod)
        # creo il messaggio di Temperatura da mandare
        T_msg = Temperature()        # definizione del type del messaggio
        T_msg.header.stamp = now     # riempio l'header con il tempo attuale a cui la misura di T viene calcolata
        T_msg.temperature = T_meas   # riempio il campo id temperature con la misura del CTD
        rospy.loginfo(T_msg)         #  mando a schermo cio' che pubblico
        pub.publish(T_msg)           # pubblico il messaggio

        rate.sleep()                 # continua a far eseguire il while al rate impostato




 ####################################################################################################
 #                                                MAIN                                              #
 ####################################################################################################

if __name__ == '__main__':
    sensing_ambiente()       # avvia la funzione sensing_ambiente sopra definita
   
