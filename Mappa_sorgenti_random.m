% ========================= PARAMETRI INIZIALI ========================== %
% Definizione lati del rettangolo di lavoro (sand-box)
tic
% lato_x = 60;  
% lato_y = 10;
% Valori usati al lago
lato_x = 27.86294930441822;  % lunghezza lato lungo asse x [m]
lato_y = 83.96066726146015;  % lunghezza lato lungo asse y [m]
% Definizione dei limiti di Temperatura da rispettare
Tmin = 12; Tmax = 14; 
Delta_T = Tmax-Tmin;
% Definisco le variabili simboliche per calcolare analiticamente le T(x,y)
syms x y;


% =================== DEFINIZIONE DEI PRAMETRI RANDOMICI ================ %
% Questa sezione serve sennò nel caso di simbolico e poi numerico i valori
% delle posizioni delle sorgenti messe random assumono due valori diversi,
% ciò non consentirebbe di controllare bene il valore di picco di T
% (Serve per far funzionae il metodo poco elegante)
% Per tutti i parametri delle sorgenti si segue la notazione "c_<variabile a cui va assegnato il coeff.>"  (c_ sta per coeff.)
% coeff. caratteristici della sorgente 1 (sferica) 
 c_xs1 = rand(1,1);                    % [adim] scalatura per posizionare componente x del centro della sorgente
 c_ys1 = rand(1,1);                    % [adim] scalatura per posizionare componente y del centro della sorgente
 c_sigma_sorg1 = 0.2;             % [adim] scalatura per ottenere la "larghezza" della sorgente

% coeff. caratteristici della sorgente 1.1 (sferica inversa)
 c_ys1_inv = rand(1,1);   % [adim] scalatura per posizionare componente y del centro della sorgente  
 c_xs1_inv = rand(1,1);   % [adim] scalatura per posizionare componente x del centro della sorgente
 c_sigma_sorg1_inv = 0.2;         % [adim] scalatura per ottenere la "larghezza" della sorgente

% coeff. caratteristici della sorgente 2 (sferica)
 c_xs2 = rand(1,1);                    % [adim] scalatura per posizionare componente x del centro della sorgente
 c_ys2 = rand(1,1);                    % [adim] scalatura per posizionare componente y del centro della sorgente
 c_sigma_sorg2 = 0.25;            % [adim] scalatura per ottenere la "larghezza" della sorgente

% coeff. caratteristici della sorgente 2.1 (sferica inversa)
 c_ys2_inv = rand(1,1);    % [adim] scalatura per posizionare componente y del centro della sorgente 
 c_xs2_inv = rand(1,1);    % [adim] scalatura per posizionare componente x del centro della sorgente
 c_sigma_sorg2_inv = 0.25;        % [adim] scalatura per ottenere la "larghezza" della sorgente

% coeff. caratteristici della sorgente 3 (retta)
 c_dev_std_retta = 0.5;           % [adim] scalatura per ottenere la "larghezza" della sorgente
 c_m = pi*1/6;                 % [adim] scalatura legata a pendenza della retta

% coeff. caratteristici della sorgente 3.1 (retta inversa)
 c_dev_std_retta_inversa = 0.5;   % [adim] scalatura per ottenere la "larghezza" della sorgente
 c_m_inv = pi*1/6;             % [adim] scalatura legata a pendenza della retta

% coeff. caratteristici della sorgente 4 (ellittica)
 c_xse1 = rand(1,1);                    % [adim] scalatura per posizionare componente x del centro della sorgente
 c_yse1 = rand(1,1);                   % [adim] scalatura per posizionare componente y del centro della sorgente
 c_sigma_sorg_x1 = 0.2;           % [adim] scalatura per ottenere la "larghezza" lungo x della sorgente
 c_sigma_sorg_y1 = 0.4;           % [adim] scalatura per ottenere la "larghezza" lungo y della sorgente

% coeff. caratteristici della sorgente 4.1 (ellittica inversa)
 c_yse1_inv = rand(1,1);    % [adim] scalatura per posizionare componente y del centro della sorgente
 c_xse1_inv = rand(1,1);    % [adim] scalatura per posizionare componente x del centro della sorgente
 c_sigma_sorg_x1_inv = 0.2;       % [adim] scalatura per ottenere la "larghezza" lungo x della sorgente
 c_sigma_sorg_y1_inv = 0.4;       % [adim] scalatura per ottenere la "larghezza" lungo y della sorgente

% coeff. caratteristici della sorgente 5 (ellittica)
 c_xse2 = rand(1,1);                   % [adim] scalatura per posizionare componente x del centro della sorgente
 c_yse2 = rand(1,1);                   % [adim] scalatura per posizionare componente y del centro della sorgente
 c_sigma_sorg_x2 = 0.2;           % [adim] scalatura per ottenere la "larghezza" lungo x della sorgente
 c_sigma_sorg_y2 = 0.65;          % [adim] scalatura per ottenere la "larghezza" lungo y della sorgente

% coeff. caratteristici della sorgente 5.1 (ellittica inversa)
 c_yse2_inv = rand(1,1);  % [adim] scalatura per posizionare componente y del centro della sorgente
 c_xse2_inv = rand(1,1);  % [adim] scalatura per posizionare componente x del centro della sorgente
 c_sigma_sorg_y2_inv = 0.2;       % [adim] scalatura per ottenere la "larghezza" lungo y della sorgente
 c_sigma_sorg_x2_inv = 0.65;      % [adim] scalatura per ottenere la "larghezza" lungo x della sorgente

% coeff. caratteristici della sorgente 6 (ellittica ruotata)
 c_x_c = rand(1,1);                     % [adim] scalatura per posizionare componente x del centro della sorgente 
 c_y_c = rand(1,1);                     % [adim] scalatura per posizionare componente y del centro della sorgente
 c_theta = rand(1,1);           % [adim] denominatore dell'angolo di cui l'ellisse e' ruotato rispetto agli assi x-y locali
 c_sigma_sorg_r_x = 0.15;         % [adim] scalatura per ottenere la "larghezza" lungo asse x' locale ruotato della sorgente
 c_sigma_sorg_r_y = 0.3;          % [adim] scalatura per ottenere la "larghezza" lungo asse y' locale ruotato della sorgente

% coeff. caratteristici della sorgente 6.1 (ellittica ruotata inversa)
 c_x_c_inv = rand(1,1);     % [adim] scalatura per posizionare componente x del centro della sorgente
 c_y_c_inv = rand(1,1);     % [adim] scalatura per posizionare componente y del centro della sorgente
 c_theta_inv = rand(1,1);       % [adim] denominatore dell'angolo di cui l'ellisse e' ruotato rispetto agli assi x-y locali
 c_sigma_sorg_r_x_inv = 0.3;      % [adim] scalatura per ottenere la "larghezza" lungo asse x' locale ruotato della sorgente
 c_sigma_sorg_r_y_inv = 0.15;     % [adim] scalatura per ottenere la "larghezza" lungo asse y' locale ruotato della sorgente

% coeff. caratteristici della sorgente 7 (ellittica ruotata)
 c_x_ca = rand(1,1);                   % [adim] scalatura per posizionare componente x del centro della sorgente
 c_y_ca = rand(1,1);                    % [adim] scalatura per posizionare componente y del centro della sorgente
 c_theta_a = -pi*rand(1,1);      % [adim] denominatore dell'angolo di cui l'ellisse e' ruotato rispetto agli assi x-y locali
 c_sigma_sorg_ra_x = 0.15;        % [adim] scalatura per ottenere la "larghezza" lungo asse x' locale ruotato della sorgente
 c_sigma_sorg_ra_y = 0.5;         % [adim] scalatura per ottenere la "larghezza" lungo asse y' locale ruotato della sorgente

% coeff. caratteristici della sorgente 7.1 (ellittica ruotata inversa)
 c_y_ca_inv = rand(1,1);  % [adim] scalatura per posizionare componente y del centro della sorgente
 c_x_ca_inv = rand(1,1);   % [adim] scalatura per posizionare componente x del centro della sorgente
 c_theta_a_inv = -pi*rand(1,1);    % [adim] denominatore dell'angolo di cui l'ellisse e' ruotato rispetto agli assi x-y locali
 c_sigma_sorg_ra_y_inv = 0.15;    % [adim] scalatura per ottenere la "larghezza" lungo asse y' locale ruotato della sorgente
 c_sigma_sorg_ra_x_inv = 0.5;     % [adim] scalatura per ottenere la "larghezza" lungo asse x' locale ruotato della sorgente


% =================== IMPORTO LE PRIMITIVE NECESSARIE =================== %
% Ora imposto tutte le posizioni delle sorgenti a numeri random 
%%% 1. Sorgente sferica piccola 1 %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
xs1 = lato_x*c_xs1; ys1 = lato_y*c_ys1; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg1 = lato_x*c_sigma_sorg1;
% funzione T(x,y)
T_sorg_sf1 = exp(-( (x-xs1)/sigma_sorg1 ).^2-( (y-ys1)/sigma_sorg1 ).^2); 


%%% 1.1 Sorgente sferica piccola 1 inversa %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
ys1_inv = lato_y*c_ys1_inv; xs1_inv = lato_x*c_xs1_inv; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg1_inv = lato_y*c_sigma_sorg1_inv; 
% funzione T(x,y)
T_sorg_sf1_inv = exp(-( (x-xs1_inv)/sigma_sorg1_inv ).^2-( (y-ys1_inv)/sigma_sorg1_inv ).^2); 


%%% 2. Sorgente sferica piccola 2 %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
xs2 = lato_x*c_xs2; ys2 = lato_y*c_ys2; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg2 = lato_x*c_sigma_sorg2; 
% funzione T(x,y)
T_sorg_sf2 = exp(-( (x-xs2)/sigma_sorg2 ).^2-( (y-ys2)/sigma_sorg2 ).^2); 

%%% 2.1 Sorgente sferica piccola 2 inversa %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
ys2_inv = lato_y*c_ys2_inv; xs2_inv = lato_x*c_xs2_inv; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg2_inv = lato_y*c_sigma_sorg2_inv;
% funzione T(x,y)
T_sorg_sf2_inv = exp(-( (x-xs2_inv)/sigma_sorg2_inv ).^2-( (y-ys2_inv)/sigma_sorg2_inv ).^2); 


%%% 3. Retta inclinata %%%
% (sono Gaussiane il cui massimo si sviluppa lungo una retta,
% max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'exp come una retta)
% "larghezza" della sorgente
dev_std_retta = c_dev_std_retta*lato_x;  
m = tan(c_m); % coeff. angolare della retta
% funzione T(x,y)
T_retta = exp(-((y-m*x-(0-m*lato_x))/dev_std_retta).^2);


%%% 3.1 Retta inclinata inv %%%
% (sono Gaussiane il cui massimo si sviluppa lungo una retta,
% max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'exp come una retta)
% "larghezza" della sorgente
dev_std_retta_inv = c_dev_std_retta_inversa*lato_y; 
m_inv = tan(c_m_inv); % coeff. angolari delle rette
% funzione T(x,y)
T_retta_inv = exp(-((x-lato_x-0.5*lato_y-m_inv*y-(0-m_inv*lato_y))/dev_std_retta_inv).^2); %corrente


%%% 4.Sorgente ellittica %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
xse = c_xse1*lato_x; yse = c_yse1*lato_y; 
sigma_sorg_x = lato_x*c_sigma_sorg_x1; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y = lato_x*c_sigma_sorg_y1; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell = exp(-( (x-xse)/sigma_sorg_x ).^2 -( (y-yse)/sigma_sorg_y ).^2);


%%% 4.1 Sorgente ellittica inversa %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
yse_inv = c_yse1_inv*lato_y; xse_inv = c_xse1_inv*lato_x; 
sigma_sorg_x_inv = lato_y*c_sigma_sorg_x1_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y_inv = lato_y*c_sigma_sorg_y1_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_inv = exp(-( (x-xse_inv)/sigma_sorg_x_inv ).^2 -( (y-yse_inv)/sigma_sorg_y_inv ).^2); 


%%% 5.Sorgente ellittica sostutiva di Rey. %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
xse1 = c_xse2*lato_x; yse1 = c_yse2*lato_y; 
sigma_sorg_x1 = lato_x*c_sigma_sorg_x2;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y1 = lato_x*c_sigma_sorg_y2; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_sost = exp(-( (x-xse1)/sigma_sorg_x1 ).^2 -( (y-yse1)/sigma_sorg_y1 ).^2); 


%%% 5.1 Sorgente ellittica sostutiva di Rey. inversa %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
yse1_inv = c_yse2_inv*lato_y; xse1_inv = c_xse2_inv *lato_x; 
sigma_sorg_y1_inv = lato_y*c_sigma_sorg_y2_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_x1_inv = lato_y*c_sigma_sorg_x2_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_sost_inv = exp(-( (x-xse1_inv)/sigma_sorg_x1_inv ).^2 -( (y-yse1_inv)/sigma_sorg_y1_inv ).^2); 


%%% 6.Sorgente ellittica ruotata %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
% coordinate del centro sorgente nel frame fisso
x_c = lato_x*c_x_c;
y_c = lato_y*c_y_c; 
% inclinazione relativa assi local rispetto ai fissi 
theta = c_theta;
% Definisco le variabili local in funzione delle global
x_local =  x .* cos(theta) + y .* sin(theta);
y_local = -x .* sin(theta) + y .* cos(theta);
% riporto anche il centro in coo. local
x_c_local =  x_c .* cos(theta) + y_c .* sin(theta);
y_c_local = -x_c .* sin(theta) + y_c .* cos(theta);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_x = lato_x*c_sigma_sorg_r_x; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_y = lato_x*c_sigma_sorg_r_y;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_r = exp(-( (x_local-x_c_local)/sigma_sorg_r_x ).^2 -( (y_local-y_c_local)/sigma_sorg_r_y ).^2);


%%% 6.1 Sorgente ellittica ruotata inversa %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
y_c_inv = lato_y*c_y_c_inv;
x_c_inv = lato_x*c_x_c_inv; 
%inclinazione relativa assi local rispetto ai fissi 
theta_inv = c_theta_inv;
% Definisco le variabili local in funzione delle global
x_local_inv =  x .* cos(theta_inv) + y .* sin(theta_inv);
y_local_inv = -x .* sin(theta_inv) + y .* cos(theta_inv);
% riporto anche il centro in coo. local
x_c_local_inv =  x_c_inv .* cos(theta_inv) + y_c_inv .* sin(theta_inv);
y_c_local_inv = -x_c_inv .* sin(theta_inv) + y_c_inv .* cos(theta_inv);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_y_inv = lato_y*c_sigma_sorg_r_y_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_x_inv = lato_y*c_sigma_sorg_r_x_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_r_inv = exp(-( (x_local_inv-x_c_local_inv)/sigma_sorg_r_x_inv ).^2 -( (y_local_inv-y_c_local_inv)/sigma_sorg_r_y_inv ).^2);


%%% 7.Sorgente ellittica ruotata agiuntiva %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
x_ca = lato_x*c_x_ca;
y_ca = lato_y*c_y_ca; 
%inclinazione relativa assi local rispetto ai fissi 
theta1 = c_theta_a ;
% Definisco le variabili local in funzione delle global
x_local1 =  x .* cos(theta1) + y .* sin(theta1);
y_local1 = -x .* sin(theta1) + y .* cos(theta1);
% riporto anche il centro in coo. local
x_c_local1 =  x_ca .* cos(theta1) + y_ca .* sin(theta1);
y_c_local1 = -x_ca .* sin(theta1) + y_ca .* cos(theta1);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_x1 = lato_x*c_sigma_sorg_ra_x; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_y1 = lato_x*c_sigma_sorg_ra_y;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_ra = exp(-( (x_local1-x_c_local1)/sigma_sorg_r_x1 ).^2 -( (y_local1-y_c_local1)/sigma_sorg_r_y1 ).^2);


%%% 7.1 Sorgente ellittica ruotata agiuntiva inversa %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
y_ca_inv = lato_y*c_y_ca_inv;
x_ca_inv = lato_x*c_x_ca_inv; 
%inclinazione relativa assi local rispetto ai fissi 
theta_inv = c_theta_a_inv;
% Definisco le variabili local in funzione delle global
x_local =  x .* cos(theta_inv) + y .* sin(theta_inv);
y_local = -x .* sin(theta_inv) + y .* cos(theta_inv);
% riporto anche il centro in coo. local
x_c_local =  x_ca_inv .* cos(theta_inv) + y_ca_inv .* sin(theta_inv);
y_c_local = -x_ca_inv .* sin(theta_inv) + y_ca_inv .* cos(theta_inv);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_y = lato_y*c_sigma_sorg_ra_y_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_x = lato_y*c_sigma_sorg_ra_x_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_ra_inv = exp(-( (x_local-x_c_local)/sigma_sorg_r_x ).^2 -( (y_local-y_c_local)/sigma_sorg_r_y ).^2);


% ============== SELEZIONO SOLO I CONTRIBUTI CHE MI SERVONO ============= %
if  lato_y/lato_x < 1/5
    p_sorg_sf1_inv = 1;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 1;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 1;    p_sorg_ell_r = 0;
    p_retta_inv = 1;         p_retta = 0;
    p_sorg_ell_ra_inv =1;   p_sorg_ell_ra = 0;
end
% caso B_inv
if lato_y/lato_x >= 1/5 && lato_y/lato_x < 1/2.4
    p_sorg_sf1_inv = 1;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 1;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 1;    p_sorg_ell_r = 0;
    p_retta_inv = 1;         p_retta = 0;
    p_sorg_ell_ra_inv = 0;   p_sorg_ell_ra =0;
end
% caso A_inv
if lato_y/lato_x >= 1/2.4 && lato_y/lato_x < 1
    p_sorg_sf1_inv = 0;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 0;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 0;    p_sorg_ell_r = 0;
    p_retta_inv = 1;       p_retta = 0;
    p_sorg_ell_ra_inv = 0;   p_sorg_ell_ra = 0;
end
%ora sono pesi per stretto e lungo (caso A)
if lato_y/lato_x >= 1 && lato_y/lato_x < 2.4
    p_sorg_sf1 = 0;          p_sorg_sf1_inv = 0;
    p_sorg_sf2 = 0;          p_sorg_sf2_inv = 0;
    p_sorg_ell_r = 0;        p_sorg_ell_r_inv = 0;
    p_retta = 1;           p_retta_inv = 0;
    p_sorg_ell_ra = 0;       p_sorg_ell_ra_inv = 0;
end
% caso B
if lato_y/lato_x >= 2.4 && lato_y/lato_x < 5
    p_sorg_sf1=1;          p_sorg_sf1_inv=0;
    p_sorg_sf2=1;          p_sorg_sf2_inv=0;
    p_sorg_ell_r = 1;       p_sorg_ell_r_inv=0;
    p_retta=1;              p_retta_inv=0;
    p_sorg_ell_ra=0;          p_sorg_ell_ra_inv=0;
end 
% caso C
if lato_y/lato_x >= 5
    p_sorg_sf1=1;          p_sorg_sf1_inv=0;
    p_sorg_sf2=1;          p_sorg_sf2_inv=0;
    p_sorg_ell_r =1;       p_sorg_ell_r_inv=0;
    p_retta=1;              p_retta_inv=0;
    p_sorg_ell_ra=1;       p_sorg_ell_ra_inv=0;
end
%condizioni più generali
if lato_y/lato_x >= 1
    p_sorg_ell = 1;        p_sorg_ell_inv = 0;
    p_sorg_ell_sost = 1;      p_sorg_ell_sost_inv = 0;
else
    p_sorg_ell = 0;           p_sorg_ell_inv=1;
    p_sorg_ell_sost = 0;      p_sorg_ell_sost_inv = 1;  
end
% sommo i contributi
T = (Delta_T)*( p_sorg_sf1*T_sorg_sf1+ p_sorg_sf2*T_sorg_sf2+ ...
    p_retta*T_retta + p_sorg_ell*T_sorg_ell + ...
    p_sorg_ell_r*T_sorg_ell_r +...
    p_sorg_ell_sost*T_sorg_ell_sost+ ...
    p_sorg_ell_ra*T_sorg_ell_ra+ ... % da ora sommo i termini per espanzione su x
    p_sorg_sf1_inv*T_sorg_sf1_inv+ p_sorg_sf2_inv*T_sorg_sf2_inv+ ...
    p_retta_inv*T_retta_inv + p_sorg_ell_inv*T_sorg_ell_inv + ...
    p_sorg_ell_r_inv*T_sorg_ell_r_inv + ...
    p_sorg_ell_sost_inv*T_sorg_ell_sost_inv+ ...
    p_sorg_ell_ra_inv*T_sorg_ell_ra_inv) + Tmin;


% ==================== CERCO IL MASSIMO DI TEMPERATURA ================== %
% Definisco l'intervallo di interesse dello spazio di lavoro
x_min = 0;
x_max = lato_x;
y_min = 0;
y_max = lato_y;
% Calcolo il massimo assoluto di T(x, y) nell'intervallo specificato
max_value = eval(max(max(abs(subs(T, {x, y}, {linspace(x_min, x_max, 100), linspace(y_min, y_max, 100)})))))
% alla peggio max_value = Delta_T*(# sorgenti sovrapposte)+Tmin (è quando si sovrappongono
% nel centro) dato che tutte ora hanno massimo valore a 1
% !NON FUNZIONA BENISSIMO!

% ========================= ADATTAMENTO DEI PESI ======================== %
% Vado a ridefinire i pesi delle sorgenti sulla base di quelli che avevamo
% tunati nel caso di sorgenti fisse moltiplicandoli per Tmax/max_value
if  lato_y/lato_x < 1/5
    p_sorg_sf1_inv = 0.85*Tmax/max_value;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 0.85*Tmax/max_value;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 0.75*Tmax/max_value;    p_sorg_ell_r = 0;
    p_retta_inv = 0.65*Tmax/max_value;         p_retta = 0;
    p_sorg_ell_ra_inv = 0.85*Tmax/max_value;   p_sorg_ell_ra = 0;
end
% caso B_inv
if lato_y/lato_x >= 1/5 && lato_y/lato_x < 1/2.4
    p_sorg_sf1_inv = 0.85*Tmax/max_value;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 0.85*Tmax/max_value;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 0.67*Tmax/max_value;    p_sorg_ell_r = 0;
    p_retta_inv = 0.55*Tmax/max_value;         p_retta = 0;
    p_sorg_ell_ra_inv = 0;
end
% caso A_inv
if lato_y/lato_x >= 1/2.4 && lato_y/lato_x < 1
    p_sorg_sf1_inv = 0;      p_sorg_sf1 = 0;
    p_sorg_sf2_inv = 0;      p_sorg_sf2 = 0;
    p_sorg_ell_r_inv = 0;    p_sorg_ell_r = 0;
    p_retta_inv = 0.5*Tmax/max_value;       p_retta = 0;
    p_sorg_ell_ra_inv = 0;   p_sorg_ell_ra = 0;
end
%ora sono pesi per stretto e lungo (caso A)
if lato_y/lato_x >= 1 && lato_y/lato_x < 2.4
    p_sorg_sf1 = 0;          p_sorg_sf1_inv = 0;
    p_sorg_sf2 = 0;          p_sorg_sf2_inv = 0;
    p_sorg_ell_r = 0;        p_sorg_ell_r_inv = 0;
    p_retta = 0.6*Tmax/max_value;           p_retta_inv = 0;
    p_sorg_ell_ra = 0;       p_sorg_ell_ra_inv = 0;
end
% caso B
if lato_y/lato_x >= 2.4 && lato_y/lato_x < 5
    p_sorg_sf1=0.85*Tmax/max_value;          p_sorg_sf1_inv=0;
    p_sorg_sf2=0.85*Tmax/max_value;          p_sorg_sf2_inv=0;
    p_sorg_ell_r =0.75*Tmax/max_value;       p_sorg_ell_r_inv=0;
    p_retta=0.7*Tmax/max_value;              p_retta_inv=0;
    p_sorg_ell_ra=0;          p_sorg_ell_ra_inv=0;
end 
% caso C
if lato_y/lato_x >= 5
    p_sorg_sf1=0.85*Tmax/max_value;          p_sorg_sf1_inv=0;
    p_sorg_sf2=0.85*Tmax/max_value;          p_sorg_sf2_inv=0;
    p_sorg_ell_r =0.75*Tmax/max_value;       p_sorg_ell_r_inv=0;
    p_retta=0.7*Tmax/max_value;              p_retta_inv=0;
    p_sorg_ell_ra=0.85*Tmax/max_value;       p_sorg_ell_ra_inv=0;
end
%condizioni più generali
if lato_y/lato_x >= 1
    p_sorg_ell = 0.85*Tmax/max_value;        p_sorg_ell_inv = 0;
    p_sorg_ell_sost = 1*Tmax/max_value;      p_sorg_ell_sost_inv = 0;
else
    p_sorg_ell = 0;           p_sorg_ell_inv = 0.85*Tmax/max_value;
    p_sorg_ell_sost = 0;      p_sorg_ell_sost_inv = 1*Tmax/max_value;  
end


% ============ POCO ELEGANTE MA RAPIDO, RIDEFINISCO SORGENTI ============ %
% Definisco la sand-box di lavoro
x = linspace(0,lato_x );
y = linspace(0, lato_y)';
% Ridefinisco le sorgenti per non averle più in simbolico
%%% 1. Sorgente sferica piccola 1 %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
xs1 = lato_x*c_xs1; ys1 = lato_y*c_ys1; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg1 = lato_x*c_sigma_sorg1;
% funzione T(x,y)
T_sorg_sf1_ = exp(-( (x-xs1)/sigma_sorg1 ).^2-( (y-ys1)/sigma_sorg1 ).^2); 


%%% 1.1 Sorgente sferica piccola 1 inversa %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
ys1_inv = lato_y*c_ys1_inv; xs1_inv = lato_x*c_xs1_inv; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg1_inv = lato_y*c_sigma_sorg1_inv; 
% funzione T(x,y)
T_sorg_sf1_inv_ = exp(-( (x-xs1_inv)/sigma_sorg1_inv ).^2-( (y-ys1_inv)/sigma_sorg1_inv ).^2); 


%%% 2. Sorgente sferica piccola 2 %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
xs2 = lato_x*c_xs2; ys2 = lato_y*c_ys2; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg2 = lato_x*c_sigma_sorg2; 
% funzione T(x,y)
T_sorg_sf2_ = exp(-( (x-xs2)/sigma_sorg2 ).^2-( (y-ys2)/sigma_sorg2 ).^2); 

%%% 2.1 Sorgente sferica piccola 2 inversa %%%
% (composizione di due Gaussiane sui due assi principali per avere decrementi simmetrici ovunque)
% coordinate del centro della sorgente
ys2_inv = lato_y*c_ys2_inv; xs2_inv = lato_x*c_xs2_inv; 
% [m] ~ dev. stnd., rapprensenta quanto " è larga " la circonferenza risultante
sigma_sorg2_inv = lato_y*c_sigma_sorg2_inv;
% funzione T(x,y)
T_sorg_sf2_inv_ = exp(-( (x-xs2_inv)/sigma_sorg2_inv ).^2-( (y-ys2_inv)/sigma_sorg2_inv ).^2); 


%%% 3. Retta inclinata %%%
% (sono Gaussiane il cui massimo si sviluppa lungo una retta,
% max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'exp come una retta)
% "larghezza" della sorgente
dev_std_retta = c_dev_std_retta*lato_x;  
m = tan(c_m); % coeff. angolare della retta
% funzione T(x,y)
T_retta_ = exp(-((y-m*x-(0-m*lato_x))/dev_std_retta).^2);


%%% 3.1 Retta inclinata inv %%%
% (sono Gaussiane il cui massimo si sviluppa lungo una retta,
% max(Gauss.)--> arg(exp)=0 --> definisco l'argomento del quadrato dell'exp come una retta)
% "larghezza" della sorgente
dev_std_retta_inv = c_dev_std_retta_inversa*lato_y; 
m_inv = tan(c_m_inv); % coeff. angolari delle rette
% funzione T(x,y)
T_retta_inv_ = exp(-((x-lato_x-0.5*lato_y-m_inv*y-(0-m_inv*lato_y))/dev_std_retta_inv).^2); %corrente


%%% 4.Sorgente ellittica %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
xse = c_xse1*lato_x; yse = c_yse1*lato_y; 
sigma_sorg_x = lato_x*c_sigma_sorg_x1; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y = lato_x*c_sigma_sorg_y1; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_ = exp(-( (x-xse)/sigma_sorg_x ).^2 -( (y-yse)/sigma_sorg_y ).^2);


%%% 4.1 Sorgente ellittica inversa %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
yse_inv = c_yse1_inv*lato_y; xse_inv = c_xse1_inv*lato_x; 
sigma_sorg_x_inv = lato_y*c_sigma_sorg_x1_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y_inv = lato_y*c_sigma_sorg_y1_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_inv_ = exp(-( (x-xse_inv)/sigma_sorg_x_inv ).^2 -( (y-yse_inv)/sigma_sorg_y_inv ).^2); 


%%% 5.Sorgente ellittica sostutiva di Rey. %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
xse1 = c_xse2*lato_x; yse1 = c_yse2*lato_y; 
sigma_sorg_x1 = lato_x*c_sigma_sorg_x2;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_y1 = lato_x*c_sigma_sorg_y2; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_sost_ = exp(-( (x-xse1)/sigma_sorg_x1 ).^2 -( (y-yse1)/sigma_sorg_y1 ).^2); 


%%% 5.1 Sorgente ellittica sostutiva di Rey. inversa %%%
% è una sferica con dev.stnd. diverse per x e y
% coordinate del centro sorgente
yse1_inv = c_yse2_inv*lato_y; xse1_inv = c_xse2_inv *lato_x; 
sigma_sorg_y1_inv = lato_y*c_sigma_sorg_y2_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_x1_inv = lato_y*c_sigma_sorg_x2_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_sost_inv_ = exp(-( (x-xse1_inv)/sigma_sorg_x1_inv ).^2 -( (y-yse1_inv)/sigma_sorg_y1_inv ).^2); 


%%% 6.Sorgente ellittica ruotata %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
% coordinate del centro sorgente nel frame fisso
x_c = lato_x*c_x_c;
y_c = lato_y*c_y_c; 
% inclinazione relativa assi local rispetto ai fissi 
theta = c_theta;
% Definisco le variabili local in funzione delle global
x_local =  x .* cos(theta) + y .* sin(theta);
y_local = -x .* sin(theta) + y .* cos(theta);
% riporto anche il centro in coo. local
x_c_local =  x_c .* cos(theta) + y_c .* sin(theta);
y_c_local = -x_c .* sin(theta) + y_c .* cos(theta);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_x = lato_x*c_sigma_sorg_r_x; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_y = lato_x*c_sigma_sorg_r_y;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_r_ = exp(-( (x_local-x_c_local)/sigma_sorg_r_x ).^2 -( (y_local-y_c_local)/sigma_sorg_r_y ).^2);


%%% 6.1 Sorgente ellittica ruotata inversa %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
y_c_inv = lato_y*c_y_c_inv;
x_c_inv = lato_x*c_x_c_inv; 
%inclinazione relativa assi local rispetto ai fissi 
theta_inv = c_theta_inv;
% Definisco le variabili local in funzione delle global
x_local_inv =  x .* cos(theta_inv) + y .* sin(theta_inv);
y_local_inv = -x .* sin(theta_inv) + y .* cos(theta_inv);
% riporto anche il centro in coo. local
x_c_local_inv =  x_c_inv .* cos(theta_inv) + y_c_inv .* sin(theta_inv);
y_c_local_inv = -x_c_inv .* sin(theta_inv) + y_c_inv .* cos(theta_inv);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_y_inv = lato_y*c_sigma_sorg_r_y_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_x_inv = lato_y*c_sigma_sorg_r_x_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_r_inv_ = exp(-( (x_local_inv-x_c_local_inv)/sigma_sorg_r_x_inv ).^2 -( (y_local_inv-y_c_local_inv)/sigma_sorg_r_y_inv ).^2);


%%% 7.Sorgente ellittica ruotata agiuntiva %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
x_ca = lato_x*c_x_ca;
y_ca = lato_y*c_y_ca; 
%inclinazione relativa assi local rispetto ai fissi 
theta1 = c_theta_a ;
% Definisco le variabili local in funzione delle global
x_local1 =  x .* cos(theta1) + y .* sin(theta1);
y_local1 = -x .* sin(theta1) + y .* cos(theta1);
% riporto anche il centro in coo. local
x_c_local1 =  x_ca .* cos(theta1) + y_ca .* sin(theta1);
y_c_local1 = -x_ca .* sin(theta1) + y_ca .* cos(theta1);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_x1 = lato_x*c_sigma_sorg_ra_x; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_y1 = lato_x*c_sigma_sorg_ra_y;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_ra_ = exp(-( (x_local1-x_c_local1)/sigma_sorg_r_x1 ).^2 -( (y_local1-y_c_local1)/sigma_sorg_r_y1 ).^2);


%%% 7.1 Sorgente ellittica ruotata agiuntiva inversa %%%
% si vanno a definire degli assi "local" ruotati di theta rispetto ai fissi
% x,y --> sarà x_local,y_local che va definita la sorgente ellittica
%coordinate del centro sorgente nel frame fisso
y_ca_inv = lato_y*c_y_ca_inv;
x_ca_inv = lato_x*c_x_ca_inv; 
%inclinazione relativa assi local rispetto ai fissi 
theta_inv = c_theta_a_inv;
% Definisco le variabili local in funzione delle global
x_local =  x .* cos(theta_inv) + y .* sin(theta_inv);
y_local = -x .* sin(theta_inv) + y .* cos(theta_inv);
% riporto anche il centro in coo. local
x_c_local =  x_ca_inv .* cos(theta_inv) + y_ca_inv .* sin(theta_inv);
y_c_local = -x_ca_inv .* sin(theta_inv) + y_ca_inv .* cos(theta_inv);
% Calcolo dell'ellisse in coo local
sigma_sorg_r_y = lato_y*c_sigma_sorg_ra_y_inv; % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo x l'ellisse risultante
sigma_sorg_r_x = lato_y*c_sigma_sorg_ra_x_inv;  % [m] ~ dev. stnd., rapprensenta quanto " è larga " lungo y l'ellisse risultante
% funzione T(x,y)
T_sorg_ell_ra_inv_ = exp(-( (x_local-x_c_local)/sigma_sorg_r_x ).^2 -( (y_local-y_c_local)/sigma_sorg_r_y ).^2);

% Ridefinisco la T complessiva come media pesata
T_ = (Delta_T)*( p_sorg_sf1*T_sorg_sf1_+ p_sorg_sf2*T_sorg_sf2_+ ...
    p_retta*T_retta_ + p_sorg_ell*T_sorg_ell_ + ...
    p_sorg_ell_r*T_sorg_ell_r_ +...
    p_sorg_ell_sost*T_sorg_ell_sost_+ ...
    p_sorg_ell_ra*T_sorg_ell_ra_+ ... % da ora sommo i termini per espanzione su x
    p_sorg_sf1_inv*T_sorg_sf1_inv_+ p_sorg_sf2_inv*T_sorg_sf2_inv_+ ...
    p_retta_inv*T_retta_inv_ + p_sorg_ell_inv*T_sorg_ell_inv_ + ...
    p_sorg_ell_r_inv*T_sorg_ell_r_inv_ +...
    p_sorg_ell_sost_inv*T_sorg_ell_sost_inv_+ ...
    p_sorg_ell_ra_inv*T_sorg_ell_ra_inv_) + Tmin;

% ========================== ELEGANTE MA LENTO ========================== %
% % Inizializzo le variabili delle T sorgenti
% T_=[];T_sorg_sf1_=[];T_sorg_sf2_=[];T_sorg_ell_=[];T_sorg_ell_r_=[];T_retta_=[];T_sorg_ell_sost_=[];
% T_sorg_ell_ra_=[];
% T_sorg_sf1_inv_=[];T_sorg_sf2_inv_=[];T_sorg_ell_inv_=[];T_sorg_ell_r_inv_=[];T_retta_inv_=[];T_sorg_ell_sost_inv_=[];
% T_sorg_ell_ra_inv_=[];
% % Definisco la grandezza della sand-box
% x_ = linspace(0,lato_x );
% y_ = linspace(0, lato_y)';
% toc
% tic
% % Per ogni punto (x,y) vado a valutare le T lo scalvo nella coordinata
% % (i,j) corrispondente alla posizione occupata da x e y nei vettori x_,y_
% for i=1:length(x_)
%     for j=1:length(y_)
%         T_(i,j)=subs(T,[x,y],[x_(i),y_(i)]);
%         T_sorg_sf1_(i,j)=subs(T_sorg_sf1,[x,y],[x_(i),y_(i)]);
%         T_sorg_sf2_(i,j)=subs(T_sorg_sf2,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_(i,j)=subs(T_sorg_ell,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_r_(i,j)=subs(T_sorg_ell_r,[x,y],[x_(i),y_(i)]);
%         T_retta_(i,j)=subs(T_retta,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_sost_(i,j)=subs(T_sorg_ell_sost,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_ra_(i,j)=subs(T_sorg_ell_ra,[x,y],[x_(i),y_(i)]);
%         T_sorg_sf1_inv_(i,j)=subs(T_sorg_sf1_inv,[x,y],[x_(i),y_(i)]);
%         T_sorg_sf2_inv_(i,j)=subs(T_sorg_sf2_inv,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_inv_(i,j)=subs(T_sorg_ell_inv,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_r_inv_(i,j)=subs(T_sorg_ell_r_inv,[x,y],[x_(i),y_(i)]);
%         T_retta_inv_(i,j)=subs(T_retta_inv,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_sost_inv_(i,j)=subs(T_sorg_ell_sost_inv,[x,y],[x_(i),y_(i)]);
%         T_sorg_ell_ra_inv_(i,j)=subs(T_sorg_ell_ra_inv,[x,y],[x_(i),y_(i)]);
% 
%     end
% end
% % Problemi: 1. Impiega troppo in questo calcolo
%            %2. Sbaglia nel riordinare le T dele sorgenti e crea tutte delle rette
%             % (pure la retta che non è randomica è ricostruita male)
toc


figure();  % plotto tutte le sorgenti 
if lato_x < lato_y
    subtitle("Sorgenti usate nei casi A,B,C")

    subplot(2,3,1); pcolor(x,y,T_sorg_sf1_+T_sorg_sf2_);colorbar()
        title('Distr. T[adim] Sorg. Sferiche'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,2); pcolor(x,y,T_sorg_ell_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,3); pcolor(x,y,T_sorg_ell_r_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica ruotata'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,4); pcolor(x,y,T_retta_); colorbar()
        title('Distr. T[adim] Riva'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,5); pcolor(x,y,T_sorg_ell_sost_); colorbar()
        title('Distr. T[adim] Sorg. Sost. Ray.'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,6); pcolor(x,y,T_sorg_ell_ra_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica ruotata aggiuntiva'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')

else % lato_x>lato_y ---> siamo nel caso _inv
% Plotto tutte le sorgenti inverse
    subtitle("Sorgenti usate nei casi A_inv, B_inv, C_inv")

    subplot(2,3,1); pcolor(x,y,T_sorg_sf1_inv_+T_sorg_sf2_inv_); colorbar()
        title('Distr. T[adim] Sorg. Sferiche'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,2); pcolor(x,y,T_sorg_ell_inv_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,3); pcolor(x,y,T_sorg_ell_r_inv_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica ruotata'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,4); pcolor(x,y,T_retta_inv_); colorbar()
        title('Distr. T[adim] Riva'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,5); pcolor(x,y,T_sorg_ell_sost_inv_); colorbar()
        title('Distr. T[adim] Sorg. Sost. Ray.'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
    subplot(2,3,6); pcolor(x,y,T_sorg_ell_ra_inv_); colorbar()
        title('Distr. T[adim] Sorg. Ellittica ruotata aggiuntiva'); xlabel('x [m]'); ylabel('y [m]'); axis equal;set(gca, 'YDir','reverse')
end

figure();  % Plotto la mappa complessiva
    pcolor(x,y,T_);
    title('Mappa di T [°C] complessiva'); axis equal; xlabel('x [m]'); ylabel('y [m]');
    set(gca, 'YDir','reverse'); colorbar()   