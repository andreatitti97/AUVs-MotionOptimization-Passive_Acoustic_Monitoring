# ros_simulation_ws
Workspace for PHD project simulation in ROS

##TODO

- MIGLIORA ANCORA IL CODICE + DEBUG PER VEDERE SE TUTTO È CORRETTO + eventuale tuning per 4 auv
- INTRODUCI PIU DELAY NELL MEASURAMENT UPDATE DEL EKF (OGNI 10 SEC)
- FAI TRAIETTORIA TARGET CURVILINEA
- TESTA ORIZZONTE MOBILE O ORIZZONTE FISSO 
- INTRODUCI L'AGGIORNAMENTO KALMAN CON VECCHIE MISURE + FAI PIU TRACKER
	- SOLO SE SEMPLICE SENNO
- IMPLEMENTA LA STIMA DI CASSINO
- SENZA PROBLEMI DI COMUNICAZIONI 
- AGGIORNA EKF CON LA STIMA DI CASSINO CON MISURE ANCHE VECCHIE (ciò va di pari passo con 
    	il tentare di aggiornare EKF con misure passate)
	- Secondo me è duale la cosa, come aggiorno regressore cassino aggiorno H su ekf
	- Quello che va fatto è avere 4 stime locali che si correggono con misure esterne
	- potresti campionare localmente AD UNA FREQUENZA PIU ALTA ma mandare le misure degli altri ogni 10 secondi (quindi ogni 10 secondi ho una stima). In ogni caso la stima generale la si aggiorna ogni 10 secondi, 
	ASSUNZIONE MASSIMA: No perdita pacchetti in comunicazione, solo delay, ma le misure arrivano tutte
	- Quindi ogni TOT le stime locali ricevono le altre misure e vengono corrette. Dopo che tutte e 4 sono corrette si passano le stime (mi aspetto che siano simili - da verificare, se non fosse fusione covarianza e stato) all OTTIMIZZATORE che genera il path. 

## Run the project
'''
roslaunch ipp_pkg ipp_simulation.launch
'''
