PROGETTO DI SISTEMI SUBACQUEI- README

PRESENTAZIONE FILE PRESENTI NELLA CARTELLA:

1. Mappa_T_riferimento.py
	File python usato per creare la mappa di temperatura di riferimento, lo script è completamente eseguibile impostati i parametri di Tmin, Tmax (limiti di 	temperatura dell'ambiente scelti) e lato_x, lato_y (grandezza della mappa, nella implementazione sincrona con i nodi ROS questi valori vanno copiati ed 		incollati dai parametri corrispondenti che il nodo del file conversione.py manda a schermo come debug iniziale.
	Le sorgenti sono tutte definite in funzione di lato_x, lato_y, per spostarle o ingrnadirle regolare solo il coeff. numerico che moltiplica tali parametri 	( posizioni e defiazioni standard sono una frazione delle lunghezze dei lati corrispondenti).

2. cartella Complessivo:
La cartella contiene il pacchetto sviluppato dal gruppo Sensing (noi) mappa_temperatura, dal gruppo Mapping temperature_mapping e dal gruppo Planning my_package2.

	2.1. laghetto_2023-06-01-17-50-58.bag
		File bag salvata durante l'esperimento al lago

	2.2 package mappa_temperatura:
		Il scr del package da noi sviluppato contiene i seguenti file:

		2.2.1 conversione.py
			File python usato per definire il nodo CONVERSIONE che legge il nav_status di Zeno e pubblica un messaggio di tipo custom LocalNavStatus con 				le posizioni e  velocità in terna locale nonchè le info utili per la definizione della terna local.

		2.2.2 geodetic-functions.py
			File python che contiene tutte le funzioni usate nel file conversione.py per eseguire le trasformazioni da coordinate geodetiche a NED. Il 				file contiene anche altre funzioni sempre legate alla conversione geodetiche-NED e viceversa per il caso 2D e 3D.

		2.2.3 local_conversion.py 
			File python di funzioni usate nel nodo conversione.py per creare l'area di lavoro ottimale (la più piccola che circoscrive il quadrilatero di 				lavoro), calcolare i versori della terna local, calcolare i parametri lato_x e lato_y e molte altre funzioni implementate nei file del gruppo 				Mapping come utility per la definizione dell'area di lavoro anche in coordinate NED e local (posizioni dei way-point in tali due terne e 				riordinazione in senso antiorario dei WP seguendo il perimetro dell'area di lavoro). Altre funzioni presenti controllano se un dato punto è 				interno alla zona di lavoro, se l'origine scelta è opportuna (se è al vertice di un angolo acuto con lati congiungenti i WP adiacenti).


		2.2.4 sensing_ambiente.py
			File python definente il nodo SENSING_AMBIENTE che legge da CONVERSIONE e pubblica la TEMPERATURA_LETTA come messaggio standard Temperature.
			Importa tutti i parametri delle sorgenti dal parameter server, calcola le T(x,y) delle singole sorgenti come fatto nel file 				Mappa_T_riferimento.py, le pesa con appositi pesi selezionati in automatico sulla base della grandezza della mappa (in base a lato_y/lato_x) e 				le somma tutte assieme. La T così ottenuta è sporcata con una realizzazione del rumore bianco, troncata alla terza cifra (per tenere conto 				della risoluzione effettiva del sensore) e mandata dentro al campo Temperature del messaggio inviato.

		2.2.5 simula_nav.py
			File python per creare un nodo SIMULA_LAT_LONG che vada a sostituire Zeno, pubblica latitudine, longitudine, vx e vy (in NED) in maniera 				randomica all'interno di range specificati. Il nodo serve per testare la funzionalità dei nodi di CONVERSIONE, SENSING_AMBIENTE ed i nodi del 				gruppo mapping (map_graph_node e mapping_node).

		2.2.6 simula_pos_local.py
			Analogo al file sopra citato ma tutte le info sono in terna local (serve per debug nel caso in cui conversione non fosse pronto).

		Nello stesso package, in cartella config è presente il file dei parametri 'parametri_sensing.yaml' contenente la definizione di tutti i parametri 			usati negli script conversione.py, sensing_ambiente.py e simula_nav.py. 

		Nella cartella launch ci sono invece 2 file di lancio utili per la simulazione dei nodi sopra citati: inizializzazione.launch carica i parametri dal 			file yaml presentato ed avvia il nodo CONVERSIONE, sensing.launch invece avvia i nodi SIMULA_LAT_LONG e SENSING_AMBIENTE


	2.3 package temperature_mapping:
	Package sviluppato dal gruppo Mapping, sono compresi vari file tra cui map_graph_node.py che crea la mappa real-time durante l'esplorazione e mapping_node.py 		che implementa il nodo di interpolazione e creazione della mappa, questo è il subscriber del topic TEMPERATURA_LETTA.
	Per una spiegazione più dettagliata si rimanda al readme del gruppo Mapping.

	2.4 package my_package2:
	Package sviluppato dal gruppo Planning, contiene i file di lancio usati durante l'esplorazione ed i file python per creare il loro nodo.
	Per una spiegazione più dettagliata si rimanda al readme del gruppo Planning.
	(Questi ultimi due package spiegati sono stati inseriti solo per dare la possibilità di riemulare i risultati dell'esperimento al lago e la validazione della 		mappa, come farlo è spiegato nella sezione successiva).

	2.5 marta_msgs
	Package necessario per utilizzare il messaggio NavStatus usato da Zeno

	2.6 temperature_msgs
	Package contenete la definizione del messaggio custom LocalNavStatus

	2.7 zeno_mission_manager
	Package necessario per comunicare con la macchina a stati di Zeno


3. Relazione 


####################################################################################################################################################################

Questa sezione del Read-me è stata fatta con lo scopo di poter riottenere i risultati discussi nella presentazione del progetto, richiede di importare il package costituito dalla cartella 'Complessivo'ed effettuare il catkin_make:

1- Validazione mappa riferimento con campionamento randomico della posizione:
	Per ottenere tale mappa seguire i seguenti passaggi:
		- Aprire il terminale

		- Eseguire: $ roslaunch my_package2 inizializzazione.launch           # per caricare tutti i parametri nel server (ridurre a 1 manualmente la lunghezza della traiettoria percorsa dal file di parametri del mapping (ultimo parametro) ) 

NOTA: A schermo compariranno varie info sul frame local creato, una delle prime cose mandate a schermo è una riga per la definizione di lato_x e lato_y, questi due valori vanno copiati ed incollati negli appositi campi presenti nel file Mappa_T_riferimento.py

		- Aprire ed eseguire file: Mappa_T_riferimento.py                     # per aprire la mappa di riferimento (chiudere la prima finestra con rappresentazioni delle sorgenti)
		- Su nuovo terminale: $ rosrun mappa_temperatura simula_nav.py        # avvia il simulatore di posizioni (lat,long)
		- Su nuovo terminale: $ roslaunch my_package2 temperature.launch      # avvia tutti i nodi del macrogruppo (compresi quelli per riempire la mappa)
		- Su nuovo terminale: $ rosrun temperature_mapping map_graph_node.py  # apre la mappa real-time



2- Riottenere i risultati dell'esperimento al lago:
		- Aprire il terminale

		- Eseguire: $ roslaunch my_package2 inizializzazione.launch     # per caricare tutti i parametri nel server (risettare i parametri del mapping al valore lasciato commentato )

NOTA: A schermo compariranno varie info sul frame local creato, una delle prime cose mandate a schermo è una riga per la definizione di lato_x e lato_y, questi due valori vanno copiati ed incollati negli appositi campi presenti nel file Mappa_T_riferimento.py

		- Aprire ed eseguire file: Mappa_T_bella.py       # per aprire la mappa di riferimento (chiudere la prima finestra con rappresentazioni delle sorgenti)
		- Su nuovo terminale: $ rosrun temperature_mapping map_graph_node.py  # apre la mappa real-time
		- Su nuovo terminale: $ rosbag play <directory della bag laghetto_2023-06-01-17-50-58.bag>  # riesegue la bag salvata al lago
		(nei primi 5 minuti non si muove per problemi di settaggio dello stato di Zeno, però si vedono i way-point generarsi)
