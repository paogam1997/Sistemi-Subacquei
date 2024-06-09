import numpy as np
import matplotlib.pyplot as plt
import seaborn
##################################################################################################
#                          CREAZIONE RETTANGOLO DI LAVORO                                        #
##################################################################################################
# Definizione lati del rettangolo di lavoro (sand-box)
# valori usati per l'esperimento al lago
lato_x = 27.86294930441822  # lunghezza lato lungo asse x [m] 
lato_y = 83.96066726146015  # lunghezza lato lungo asse y [m]  

# Definizione dei vertici della zona di lavoro [(m,m)]
v1 = np.array([0, 0])
v2 = np.array([lato_x, 0])
v3 = np.array([lato_x, lato_y])
v4 = np.array([0, lato_y])
# Creazione della sand-box  [m]
x = np.linspace(0, lato_x)
y = np.linspace(0, lato_y)[:, np.newaxis]  # opzione per definire l'asse perpendicolare ad x
# Definisco i limiti di Temperatura dell'ambiente da rispettare [Celsius]
Tmin = 12
Tmax = 14
Delta_T = Tmax - Tmin

##################################################################################################
#                        CREAZIONE DELLE PRIMITIVE DELLE FUNZIONI DA USARE                       #
##################################################################################################
# 1.Sorgente SFERICA PICCOLA 1 # (alto a destra)
# (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
# Coordinate [m] del centro della sorgente
xs1 = lato_x * 0.85
ys1 = lato_y * 0.35  
# [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
sigma_sorg1 = lato_x * 0.2  
# funzione T(x,y) [adim]
T_sorg_sf1 = np.exp(-((x - xs1) / sigma_sorg1) ** 2 - ((y - ys1) / sigma_sorg1) ** 2)

# 1.1 Sorgente SFERICA PICCOLA 1 inversa #
# (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
# Coordinate [m] del centro della sorgente
ys1 = lato_y * (1 - 0.85)
xs1 = lato_x * (1 - 0.35) 
# [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
sigma_sorg1 = lato_y * 0.2  
# funzione T(x,y) [adim]
T_sorg_sf1_inv = np.exp(-((x - xs1) / sigma_sorg1) ** 2 - ((y - ys1) / sigma_sorg1) ** 2)

# 2.Sorgente SFERICA PICCOLA 2 # (basso a sinistra)
# (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
# Coordinate [m] del centro della sorgente
xs2 = lato_x * 0.25
ys2 = lato_y * 0.9 
# [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
sigma_sorg2 = lato_x * 0.25  
# funzione T(x,y) [adim]
T_sorg_sf2 = np.exp(-((x - xs2) / sigma_sorg2) ** 2 - ((y - ys2) / sigma_sorg2) ** 2)

# 2.1 Sorgente SFERICA PICCOLA 2 inversa #
# (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
# Coordinate [m] del centro della sorgente
ys2 = lato_y * (1 - 0.25)
xs2 = lato_x * (1 - 0.9)  
# [m] ~ dev. stnd., rapprensenta quanto "e' larga" la circonferenza risultante
sigma_sorg2 = lato_y * 0.25  
# funzione T(x,y) [adim]
T_sorg_sf2_inv = np.exp(-((x - xs2) / sigma_sorg2) ** 2 - ((y - ys2) / sigma_sorg2) ** 2)

# 3. RETTA INCLINATA # (angolo basso a sinistra)
# (sono Gaussiane il cui massimo si sviluppa lungo una retta,
# max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'esponenziale come una retta)
# "larghezza" della sorgente [m]
dev_std_retta = 0.5 * lato_x
# coeff. angolare della retta [adim]
m = np.tan(np.pi / 6)  
# funzione T(x,y) [adim]
T_retta = np.exp(-((y - m * x - (v2[1] - m * v2[0])) / dev_std_retta) ** 2)  # riva

# 3.1 RETTA INCLINATA  inversa # (angolo basso a sinistra)
# (sono Gaussiane il cui massimo si sviluppa lungo una retta,
# max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'esponenziale come una retta)
# "larghezza" della sorgente [m]
dev_std_retta_inv = 0.5 * lato_y
# coeff. angolare della retta [adim]
m = np.tan(np.pi / 6)  
# funzione T(x,y) [adim]
T_retta_inv = np.exp(-((x - lato_x - 0.5*lato_y - m * y - (0 - m * lato_y)) / dev_std_retta_inv) ** 2)  # riva

# 4. SORGENTE ELLITTICA # (alto-destra)
#  e' una sferica con dev.stnd. diverse per x e y
# Coordinate [m] del centro della sorgente
xse1 = 0.8 * lato_x
yse1 = 0.7 * lato_y 
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_x1 = lato_x * 0.2  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_y1 = lato_x * 0.4  
# funzione T(x,y) [adim]
T_sorg_ell = np.exp(-((x - xse1) / sigma_sorg_x1) ** 2 - ((y - yse1) / sigma_sorg_y1) ** 2)

# 4.1 SORGENTE ELLITTICA inversa  # (alto-destra)
#  e' una sferica con dev.stnd. diverse per x e y
# Coordinate [m] del centro della sorgente
yse1 = (1 - 0.8) * lato_y
xse1 = (1 - 0.7) * lato_x  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_x1 = lato_y * 0.2  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_y1 = lato_y * 0.4  
# funzione T(x,y) [adim]
T_sorg_ell_inv = np.exp(-((x - xse1) / sigma_sorg_x1) ** 2 - ((y - yse1) / sigma_sorg_y1) ** 2)

# 5. SORGENTE ELLITTICA SOSTITUTIVA DI REYLEIGH. # (basso-sx)
# e' una sferica con dev.stnd. diverse per x e y
# Coordinate [m] del centro della sorgente
xse2 = 0.25 * lato_x
yse2 = 0.35 * lato_y  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_x2 = lato_x * 0.2  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_y2 = lato_x * 0.65  
# funzione T(x,y) [adim]
T_sorg_ell_sost = np.exp(-((x - xse2) / sigma_sorg_x2) ** 2 - ((y - yse2) / sigma_sorg_y2) ** 2)

# 5.1 SORGENTE ELLITTICA SOSTITUTIVA DI REYLEIGH. inversa # (basso-sx)
# e' una sferica con dev.stnd. diverse per x e y
# Coordinate [m] del centro della sorgente
yse2 = (1 - 0.25) * lato_y
xse2 = (1 - 0.35) * lato_x 
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_y2 = lato_y * 0.2  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_x2 = lato_y * 0.65  
# funzione T(x,y) [adim]
T_sorg_ell_sost_inv = np.exp(-((x - xse2) / sigma_sorg_x2) ** 2 - ((y - yse2) / sigma_sorg_y2) ** 2)

# 6.SORGENTE ELLITTICA RUOTATA #  (alto-sx)
#  si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
#  x,y --> sara' x_local, y_local che va definita la sorgente ellittica
# Coordinate [m] del centro della sorgente in assi fissi
x_c = lato_x * 0.2
y_c = lato_y * 0.7  
# Inclinazione assi locali rispetto ai fissi [rad]
theta = np.pi / 4  
# Definisco le variabili locali in funzioni delle fisse [m]
x_local = x * np.cos(theta) + y * np.sin(theta)
y_local = -x * np.sin(theta) + y * np.cos(theta)
# Riporto il centro in coordinate locali [m]
x_c_local = x_c * np.cos(theta) + y_c * np.sin(theta)
y_c_local = -x_c * np.sin(theta) + y_c * np.cos(theta)
# Definisco l'ellisse nelle coordinate locali
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_r_x = lato_x * 0.15  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_r_y = lato_x * 0.3  
# funzione T(x,y) [adim]
T_sorg_ell_r = np.exp(-((x_local - x_c_local) / sigma_sorg_r_x) ** 2 - ((y_local - y_c_local) / sigma_sorg_r_y) ** 2)

# 6.1 SORGENTE ELLITTICA RUOTATA inversa #  (alto-sx)
#  si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
#  x,y --> sara' x_local, y_local che va definita la sorgente ellittica
# Coordinate [m] del centro della sorgente in assi fissi
y_c = lato_y * (1 - 0.2)
x_c = lato_x * (1 - 0.7) 
# Inclinazione assi locali rispetto ai fissi [rad]
theta = np.pi / 4  
# Definisco le variabili locali in funzioni delle fisse [m]
x_local = x * np.cos(theta) + y * np.sin(theta)
y_local = -x * np.sin(theta) + y * np.cos(theta)
# Riporto il centro in coordinate locali [m]
x_c_local = x_c * np.cos(theta) + y_c * np.sin(theta)
y_c_local = -x_c * np.sin(theta) + y_c * np.cos(theta)
# Definisco l'ellisse nelle coordinate locali
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_r_y = lato_y * 0.15  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_r_x = lato_y * 0.3  
# funzione T(x,y) [adim]
T_sorg_ell_r_inv = np.exp(
    -((x_local - x_c_local) / sigma_sorg_r_x) ** 2 - ((y_local - y_c_local) / sigma_sorg_r_y) ** 2)

# 7.SORGENTE ELLITTICA RUOTATA AGGIUNTIVA #
# si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
# x,y --> sara' x_local, y_local che va definita la sorgente ellittica
# Coordinate [m] del centro della sorgente in assi fissi
x_ca = lato_x * 0.85
y_ca = lato_y * 0.9  
# Inclinazione assi locali rispetto ai fissi [rad]
theta_a = - np.pi / 6  
# Definisco le variabili locali in funzioni delle fisse [m]
x_local_a = x * np.cos(theta_a) + y * np.sin(theta_a)
y_local_a = -x * np.sin(theta_a) + y * np.cos(theta_a)
# Riporto il centro in coordinate locali [m]
x_c_local_a = x_ca * np.cos(theta_a) + y_ca * np.sin(theta_a)
y_c_local_a = -x_ca * np.sin(theta_a) + y_ca * np.cos(theta_a)
# Definisco l'ellisse nelle coordinate locali
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_ra_x = lato_x * 0.15  
# [m] ~ dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_ra_y = lato_x * 0.5  
# funzione T(x,y) [adim]
T_sorg_ell_ra = np.exp(
    -((x_local_a - x_c_local_a) / sigma_sorg_ra_x) ** 2 - ((y_local_a - y_c_local_a) / sigma_sorg_ra_y) ** 2)

# 7.1 SORGENTE ELLITTICA RUOTATA AGGIUNTIVA inversa #
# si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
# x,y --> sara' x_local, y_local che va definita la sorgente ellittica
# Coordinate [m] del centro della sorgente in assi fissi
y_ca = lato_y * (1 - 0.85)
x_ca = lato_x * (1 - 0.9)  
# Inclinazione assi locali rispetto ai fissi [rad]
theta_a = - np.pi / 6  
# Definisco le variabili locali in funzioni delle fisse [m]
x_local_a = x * np.cos(theta_a) + y * np.sin(theta_a)
y_local_a = -x * np.sin(theta_a) + y * np.cos(theta_a)
# Riporto il centro in coordinate locali [m]
x_c_local_a = x_ca * np.cos(theta_a) + y_ca * np.sin(theta_a)
y_c_local_a = -x_ca * np.sin(theta_a) + y_ca * np.cos(theta_a)
# Definisco l'ellisse nelle coordinate locali
# [m] 2*dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo x
sigma_sorg_ra_y = lato_y * 0.15  
# [m] 2*dev. stnd., rappresenta quanto "e' larga" l'ellisse' risultante lungo y
sigma_sorg_ra_x = lato_y * 0.5  
# funzione T(x,y) [adim]
T_sorg_ell_ra_inv = np.exp(
    -((x_local_a - x_c_local_a) / sigma_sorg_ra_x) ** 2 - ((y_local_a - y_c_local_a) / sigma_sorg_ra_y) ** 2)

####################################################################################################
#                           SOMMO PESANDOLI I CONTRIBUTI CHE MI SERVONO                            #
####################################################################################################
# definisco i pesi [adim] in funzione di quanto la mappa e' grossa
# inizializzo i pesi
p_sorg_sf1_inv = 0
p_sorg_sf1 = 0
p_sorg_sf2_inv = 0
p_sorg_sf2 = 0
p_sorg_ell_r_inv = 0
p_sorg_ell_r = 0
p_retta_inv = 0
p_retta = 0
p_sorg_ell_ra_inv = 0
p_sorg_ell_ra = 0
p_sorg_ell = 0
p_sorg_ell_inv = 0
p_sorg_ell_sost = 0
p_sorg_ell_sost_inv = 0
# ora sono i pesi per il caso largo e stretto (caso C_inv)
if lato_y / lato_x < 1 / 5:
    p_sorg_sf1_inv = 0.85
    p_sorg_sf1 = 0
    p_sorg_sf2_inv = 0.85
    p_sorg_sf2 = 0
    p_sorg_ell_r_inv = 0.75
    p_sorg_ell_r = 0
    p_retta_inv = 0.65
    p_retta = 0
    p_sorg_ell_ra_inv = 0.85
    p_sorg_ell_ra = 0
    p_sorg_ell = 0
    p_sorg_ell_inv = 0.85
    p_sorg_ell_sost = 0
    p_sorg_ell_sost_inv = 1
# caso B_inv
if 1 / 5 <= lato_y / lato_x < 1 / 2.4:
    p_sorg_sf1_inv = 0.85
    p_sorg_sf1 = 0
    p_sorg_sf2_inv = 0.85
    p_sorg_sf2 = 0
    p_sorg_ell_r_inv = 0.67
    p_sorg_ell_r = 0
    p_retta_inv = 0.55
    p_retta = 0
    p_sorg_ell_ra_inv = 0
    p_sorg_ell = 0
    p_sorg_ell_inv = 0.85
    p_sorg_ell_sost = 0
    p_sorg_ell_sost_inv = 1
# caso A_inv
if 1 / 2.4 <= lato_y / lato_x < 1:
    p_sorg_sf1_inv = 0
    p_sorg_sf1 = 0
    p_sorg_sf2_inv = 0
    p_sorg_sf2 = 0
    p_sorg_ell_r_inv = 0
    p_sorg_ell_r = 0
    p_retta_inv = 0.5
    p_retta = 0
    p_sorg_ell_ra_inv = 0
    p_sorg_ell_ra = 0
    p_sorg_ell = 0
    p_sorg_ell_inv = 0.85
    p_sorg_ell_sost = 0
    p_sorg_ell_sost_inv = 1

# ora sono pesi per stretto e lungo (caso A)
if 1 <= lato_y / lato_x < 2.4:
    p_sorg_sf1 = 0
    p_sorg_sf1_inv = 0
    p_sorg_sf2 = 0
    p_sorg_sf2_inv = 0
    p_sorg_ell_r = 0
    p_sorg_ell_r_inv = 0
    p_retta = 0.6
    p_retta_inv = 0
    p_sorg_ell_ra = 0
    p_sorg_ell_ra_inv = 0
    p_sorg_ell = 0.85
    p_sorg_ell_inv = 0
    p_sorg_ell_sost = 1
    p_sorg_ell_sost_inv = 0
# caso B
if 2.4 <= lato_y / lato_x < 5:
    p_sorg_sf1 = 0.85
    p_sorg_sf1_inv = 0
    p_sorg_sf2 = 0.85
    p_sorg_sf2_inv = 0
    p_sorg_ell_r = 0.75
    p_sorg_ell_r_inv = 0
    p_retta = 0.7
    p_retta_inv = 0
    p_sorg_ell_ra = 0
    p_sorg_ell_ra_inv = 0
    p_sorg_ell = 0.85
    p_sorg_ell_inv = 0
    p_sorg_ell_sost = 1
    p_sorg_ell_sost_inv = 0
# caso C
if lato_y / lato_x >= 5:
    p_sorg_sf1 = 0.85
    p_sorg_sf1_inv = 0
    p_sorg_sf2 = 0.85
    p_sorg_sf2_inv = 0
    p_sorg_ell_r = 0.75
    p_sorg_ell_r_inv = 0
    p_retta = 0.7
    p_retta_inv = 0
    p_sorg_ell_ra = 0.85
    p_sorg_ell_ra_inv = 0
    p_sorg_ell = 0.85
    p_sorg_ell_inv = 0
    p_sorg_ell_sost = 1
    p_sorg_ell_sost_inv = 0


# sommo tutti i contributi per avere la T complessiva nella mappa [Celsius]
T = (Delta_T) * (p_sorg_sf1 * T_sorg_sf1 + p_sorg_sf2 * T_sorg_sf2 + p_retta * T_retta + p_sorg_ell * T_sorg_ell +
                 p_sorg_ell_r * T_sorg_ell_r + p_sorg_ell_sost * T_sorg_ell_sost + p_sorg_ell_ra * T_sorg_ell_ra +  # da ora sommo i termini per espanzione su x
                 p_sorg_sf1_inv * T_sorg_sf1_inv + p_sorg_sf2_inv * T_sorg_sf2_inv + p_retta_inv * T_retta_inv +
                 p_sorg_ell_inv * T_sorg_ell_inv + p_sorg_ell_r_inv * T_sorg_ell_r_inv +
                 p_sorg_ell_sost_inv * T_sorg_ell_sost_inv + p_sorg_ell_ra_inv * T_sorg_ell_ra_inv) + Tmin

################################################################################################################################
#                                                    CREAZIONE DEI GRAFICI                                                     #
################################################################################################################################
# Plotto le sorgenti usate
if lato_y > lato_x:       # casi A,B,C
    plt.figure()          # creo una nuova figura

    plt.subplot(2, 3, 1)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_sf1 + T_sorg_sf2, cmap='rocket')     # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Sferiche')                      # imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 2)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_retta, cmap='rocket')                     # plotto la sorgente
    plt.title('Distr. T [adim] Riva')                                # imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 3)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell, cmap='rocket')                  # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica')                     # imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 4)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_sost, cmap='rocket')             # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica sostitutiva')         # imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 5)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_r, cmap='rocket')                # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica ruotata')             # imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 6)                                             # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_ra, cmap='rocket')               # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica ruotata aggiuntiva')  #  imposto il titolo del subplot
    plt.xlabel('x [m]')                                              # definisco l'asse x
    plt.ylabel('y [m]')                                              # definisco l'asse y
    plt.axis('equal')                                                # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                   # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                       # inverto asse y per coerenza con grafico real-time

    plt.show()
else:           # mappa larga e stretta relativa ai casi A_inv,B_inv,C_inv
    plt.figure()                                                          # creo una nuova figura
    
    plt.subplot(2, 3, 1)                                                  # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_sf1_inv + T_sorg_sf2_inv, cmap='rocket')  # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Sferiche inverse')                   # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                   # definisco l'asse x
    plt.ylabel('y [m]')                                                   # definisco l'asse y
    plt.axis('equal')                                                     # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                        # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                            # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 2)                                                  # creo il primo subplot
    plt.pcolormesh(x, y, T_retta_inv, cmap='rocket')                      # plotto la sorgente
    plt.title('Distr. T [adim] Riva inversa')                             # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                   # definisco l'asse x
    plt.ylabel('y [m]')                                                   # definisco l'asse y
    plt.axis('equal')                                                     # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                        # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                            # inverto asse y per coerenza con grafico real-time
      
    plt.subplot(2, 3, 3)                                                  # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_inv, cmap='rocket')                   # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica inversa')                  # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                   # definisco l'asse x
    plt.ylabel('y [m]')                                                   # definisco l'asse y
    plt.axis('equal')                                                     # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                        # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                            # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 4)                                                  # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_sost_inv, cmap='rocket')              # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica sostitutiva inversa')      # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                   # definisco l'asse x
    plt.ylabel('y [m]')                                                   # definisco l'asse y
    plt.axis('equal')                                                     # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                        # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                            # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 5)                                                  # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_r_inv, cmap='rocket')                 # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica ruotata inversa')          # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                   # definisco l'asse x
    plt.ylabel('y [m]')                                                   # definisco l'asse y
    plt.axis('equal')                                                     # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                        # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                            # inverto asse y per coerenza con grafico real-time

    plt.subplot(2, 3, 6)                                                    # creo il primo subplot
    plt.pcolormesh(x, y, T_sorg_ell_ra_inv, cmap='rocket')                  # plotto la sorgente
    plt.title('Distr. T [adim] Sorg. Ellittica ruotata aggiuntiva inversa') # imposto il titolo del subplot
    plt.xlabel('x [m]')                                                     # definisco l'asse x
    plt.ylabel('y [m]')                                                     # definisco l'asse y
    plt.axis('equal')                                                       # impongo che i due assi siano scalati ugualmente
    plt.colorbar()                                                          # aggiungo la scala di colori di riferimento
    plt.ylim(max(plt.ylim()), min(plt.ylim()))                              # inverto asse y per coerenza con grafico real-time

    plt.show()

# Plotto la mappa complessiva
plt.figure()                                                                # creo una nuova figura
plt.pcolormesh(x, y, T, cmap='rocket',label=u'Temperatura [Celsius]')       # plotto la mappa
plt.title('Mappa di T [Celsius] complessiva')                               # imposto il titolo della figura
plt.xlabel('x [m]')                                                         # definisco l'asse x
plt.ylabel('y [m]')                                                         # definisco l'asse y
plt.axis('equal')                                                           # impongo che i due assi siano scalati ugualmente
plt.colorbar()                                                              # aggiungo la scala di colori di riferimento
plt.ylim(max(plt.ylim()), min(plt.ylim()))                                  # inverto asse y per coerenza con grafico real-time
#  imposta i limiti dell'asse y in base ai limiti correnti,
#  ma scambia i valori massimi e minimi per invertire la direzione.
plt.show()                                                                  # mostro il plot creato