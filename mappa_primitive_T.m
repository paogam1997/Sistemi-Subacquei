% ========================= PARAMETRI INIZIALI ========================== %
% Definizione lati del rettangolo di lavoro (sand-box)
x = linspace(0,10);   % [m]
y = linspace(0,10)';  % [m]
% Limiti di T da voler rispettare
Tmin = 10; Tmax = 22; % [°C]
Delta_T = Tmax-Tmin;  % [°C]

% =============== CREAZIONE DELLE PRIMITIVE DA IMPORTARE ================ %
% (tutti i valori di posizioni e dev.std. sono solo esemplificativi)

%%% 1. Riva %%%
% "linea orizzontale che decade come Gaussiana"
% Per averla serve mettere i massimi della gaussiana tutti su una retta
sigma_riva = 0.5;
% [m] ~ deviazione standard, rappresenta per quanto si estende l'effetto della riva
T_riva = exp(-(y.^2)/sigma_riva^2)+0*x;
% 0*x serve solo per renderla una matrice sennò Matlab si arrabbia


%%% 2. Sorgente sferica %%%
% "composizione di due Gaussiane sui due assi principali" per avere decrementi simmetrici ovunque
% serve pensare che tenendo x cost vedi una gaussiana T(y) sul piano T-y ed
% analogamente per y cost vedi su T-x un'altra gaussiana T(x), queste due
% devono valere in contemporanea quindi si moltiplicano --> si sommano gli
% esponenti delle gaussiane
xs = 5; ys = 5; % coordinate del centro della sorgente
sigma_sorg = 1; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
T_sorg_sf = exp(-( (x-xs)/sigma_sorg ).^2-( (y-ys)/sigma_sorg ).^2); 


%%% 3. Rette inclinate %%%
% (sono Gaussiane il cui massimo si sviluppa lungo una retta,
% max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'exp come una retta) 
% se ne creano un ventaglio per avere un effetto più ampio
m1 = 1; m2 = 2; m3 = 0.5; m4 = 0.75; m5 = 1.5; % coeff. angolari delle rette
T_retta_1 = exp(-(y-m1*x-(ys-m1*xs)).^2); %corrente
T_retta_2 = exp(-(y-m2*x-(ys-m2*xs)).^2);
T_retta_3 = exp(-(y-m3*x-(ys-m3*xs)).^2);
T_retta_4 = exp(-(y-m4*x-(ys-m4*xs)).^2);
T_retta_5 = exp(-(y-m5*x-(ys-m5*xs)).^2);
% pesi delle rette dati in modo simmetrico dalla retta centrale (1)
p1=0.1655;
p2=0.155;
p3=0.155;
p4=0.16355;
p5=0.16355;
%somma pesata delle rette (impacchetto in una unica sorgente)
T_retta = p1*T_retta_1+p2*T_retta_2+p3*T_retta_3+p4*T_retta_4+p5*T_retta_5; 
%! PROBLEMA: si applica bene se le facciamo intersecare su un bordo della
%mappa, altrimenti si vedono anche le "code" delle rette (i prolungamenti
%nella direzione non desiderata) --> sorgente scartata a favore di un
%ellisse ruotato

%%% 4.Sorgente ellittica %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
xse = 5; yse = 5; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_x = 2; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
sigma_sorg_y = 1; 
T_sorg_ell = exp(-( (x-xse)/sigma_sorg_x ).^2 -( (y-yse)/sigma_sorg_y ).^2); %sorgente ellittica


%%% 5.Sorgente ellittica ruotata %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà in x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
x_c = 5;
y_c = 5; 
%inclinazione relativa assi local rispetto ai fissi (può essere la direzione della corrente scelta)
theta = pi/8;
% Definisco le variabili local in funzione delle global
x_local =  x .* cos(theta) + y .* sin(theta);
y_local = -x .* sin(theta) + y .* cos(theta);
% riporto anche il centro in coo. local
x_c_local =  x_c .* cos(theta) + y_c .* sin(theta);
y_c_local = -x_c .* sin(theta) + y_c .* cos(theta);
% Calcolo dell'ellisse in coo local 
sigma_sorg_r_x = 2; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_y = 1; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
T_sorg_ell_r = exp(-( (x_local-x_c_local)/sigma_sorg_r_x ).^2 -( (y_local-y_c_local)/sigma_sorg_r_y ).^2);


%%% 6. Sorgenti con corrente inclinata e uniforme (sc) %%%
% sono delle funzioni che si sviluppano come Gaussiane se y cost. (dir.
% perp. a corrente) e funzioni particolari se x cost (dir. della corrente) 
% T è creata come ( comport. a y cost )*( comport. a x cost )

    % 6.1 Logonormale lungo corrente 
    % centro Gaussiana
    x_c_sc_log = 5; 
    sigma_sc_log_x = 2; % [m] ~ dev.stnd. della Gaussiana
    sigma_sc_log_y = 1; mu=2; % dev.std. e media della distribuzione logonormale
    T_sc_logonormale = ( exp(-( (x-x_c_sc_log)/sigma_sc_log_x ).^2) ) .* ( 1./(y*sqrt(2*pi*sigma_sc_log_y^2)) .* exp(-0.5*( (log(y)-mu)/sigma_sc_log_y ).^2)  ); 
    %! PROBLEMA: non può essere ruotata a causa del log, servirebbero
    %troppi if per controllare le zone a y_local < 0 --> scartata

    % 6.2 Plank lungo corrente 
    sigma_sc_Plank_x=2; % [m] ~ dev.stnd. della Gaussiana
    x_c_sc_Plank=5;  % centro Gaussiana
    b2=25;
    c2=0.01; % coeff per Plank
    T_sc_Plank = ( exp(-( ((x-x_c_sc_Plank)/sigma_sc_Plank_x).^2 )) ) .* ( b2./(y-y_c).^5 .* 1./(exp(c2./(y-y_c))-1) ); %Plank
    %! PROBLEMA: decade troppo in fretta --> scartata

    %6.3 Rayleigh lungo corrente
    x_c_sc_Ray = 5; % centro Gaussiana
    sigma_sc_Ray_x = 2; % [m] ~ dev.stnd. della Gaussiana
    moda_sc_Ray_y = 5; % massimo della distribuzione di Rayleigh
    T_sc_Ray = ( exp(-( (x-x_c_sc_Ray)/sigma_sc_Ray_x ).^2) ) .* ( y/moda_sc_Ray_y^2 .*exp(- y.^2/(2*moda_sc_Ray_y^2) ) ); %Reyleigh
       
     %! PROBLEMA: Nella versione robusta non è facile tunare il suo peso
     %in modo che sia sempre parimenti afficace per ogni scelta del
     %rettangolo di lavoro --> scartata (peccato aveva bella forma)

% ============= SOMMO PESANDOLI I CONTRIBUTI CHE MI SERVONO ============= %

% definisco i pesi (sono troppo dipendenti dalla forma del rettangolo
% impostata)
p_riva = 0.2;
p_sorg_sf = 0;
% T_retta già pesata al suo interno
p_sorg_ell = 1;
p_sorg_ell_r = 0;
p_sc_logonormale = 0;
p_sc_Plank = 0;
p_sc_Ray = 0;
% sommo i contributi
T=(Delta_T)*( p_riva*T_riva + p_sorg_sf*T_sorg_sf + T_retta + p_sorg_ell*T_sorg_ell + ...
              p_sorg_ell_r*T_sorg_ell_r + p_sc_logonormale*T_sc_logonormale + ...
              p_sc_Plank*T_sc_Plank + p_sc_Ray*T_sc_Ray ) + Tmin;



% =============================== GRAFICI =============================== %
figure(); 
    pcolor(x,y,T_riva); 
        title('Distr. T[adim] Sorg. riva'); axis equal;xlabel('x [m]'); ylabel('y [m]');
figure();
    pcolor(x,y,T_retta) ; 
        title('Distr. T[adim] ventaglio rette'); axis equal;xlabel('x [m]'); ylabel('y [m]');
figure();
    subplot(3,2,1); pcolor(x,y,T_sorg_sf); 
        title('Distr. T[adim] Sorg.Sferica'); axis equal;xlabel('x [m]'); ylabel('y [m]');
    subplot(3,2,2); pcolor(x,y,T_sorg_ell); 
        title('Distr. T[adim] Sorg.Ellittica'); axis equal;xlabel('x [m]'); ylabel('y [m]');
    subplot(3,2,3); pcolor(x,y,T_sorg_ell_r); 
        title('Distr. T[adim] Sorg.Ellittica ruotata'); axis equal;xlabel('x [m]'); ylabel('y [m]');
    subplot(3,2,4); pcolor(x,y,T_sc_logonormale); 
        title('Distr. T[adim] Sorg.Gauss+Logonorm'); axis equal;xlabel('x [m]'); ylabel('y [m]');
    subplot(3,2,5); pcolor(x,y,T_sc_Ray); 
        title('Distr. T[adim] Sorg.Gauss+Ray'); axis equal;xlabel('x [m]'); ylabel('y [m]');
    subplot(3,2,6); pcolor(x,y,T_sc_Plank); 
        title('Distr. T[adim] Sorg.Gauss+Planck'); axis equal;xlabel('x [m]'); ylabel('y [m]');
figure();
    pcolor(x,y,T); 
        title('Distr. T[°C] complessiva'); axis equal;xlabel('x [m]'); ylabel('y [m]');
 
