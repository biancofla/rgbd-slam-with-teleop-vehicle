# RGB-D SLAM con veicolo teleoperato

Un problema di grande importanza nell'ambito della robotica mobile è quello di creare dei veicoli in grado di sapersi localizzare nello spazio circostante - sia esso conosciuto o meno - e di saper eventualmente creare una mappa di questo ambiente.

## Inizializzazione

L'esecuzione del codice necessita della presenza di un robot mobile, agente da server, e di un computer remoto, agente da client.

### Lato client - Computer remoto

**Pre-requisiti software**:
- Ubuntu 18.04
- ROS Melodic Morenia
- moduli Python contenuti nel file `requirements.txt`

Occorre clonare il codice sorgente all'interno dello spazio di lavoro di ROS. 

`git clone https://github.com/biancofla/rgbd-slam-with-teleop-vehicle.git $HOME/<nome_spazio_di_lavoro>/src`

Una volta clonato il codice, occorre posizionarsi nella directory `$HOME/<nome_spazio_di_lavoro>` attraverso il comando

`cd $HOME/<nome_spazio_di_lavoro>`

e lanciare il comando

`catkin_make`

al fine di permettere la compilazione del package.

### Lato server - Robot mobile

Si assume che l'utente sia in possesso di un robot mobile [simil-MARRtino](https://www.marrtino.org/).

**Pre-requisiti software**:
- Ubuntu 20.04
- ROS Noetic Ninjemy
- RTAB-Map
- libfreenect
- Arduino IDE
- moduli Python contenuti nel file `requirements.txt`

Occorre clonare il package e configurarlo similmente a quanto fatto sul computer remoto.

## Esecuzione

### Configurazione ROS Master

Occorre configurare - lato client e lato server - delle variabili d'ambiente utili a definire su quale delle due parti verrà eseguito il nodo master di ROS. Bisogna quindi aggiungere al file `.bashrc` o `.zshrc` le linee di codice

```bash
export ROS_MASTER_URI=<uri-master>
export ROS_HOSTNAME=<ip-dispositivo-corrente>
export ROS_IP=<ip-dispositivo-corrente>
```

### Guida teleoperata

[![](https://img.youtube.com/vi/TUHRMXqMq3Y/0.jpg)](https://youtu.be/TUHRMXqMq3Y)

#### Lato server - Robot mobile

Fare l'upload dello sketch Arduino `arduino_cmd.ino` attraverso l'apposito ambiente di sviluppo e lanciare il comando

`rosrun  rgbd-slam-with-teleop-vehicle teleop_listener.py`

#### Lato client - Computer remoto

Lanciare il comando

`rosrun  rgbd-slam-with-teleop-vehicle teleop_talker.py`

### Guida teleoperata con acquisizione delle immagini

L'operazione di acquisizione e visualizzazione delle immagini attraverso il sensore di visione di Kinect può essere effettuata in due modalità differenti
- utilizzando solo il robot mobile
- sfruttando la logica client-server

[![](https://img.youtube.com/vi/f8B4taeDggw/0.jpg)](https://youtu.be/f8B4taeDggw)
[![](https://img.youtube.com/vi/viF9TI1LlGg/0.jpg)](https://youtu.be/viF9TI1LlGg)

#### Lato server - Robot mobile

Fare l'upload dello sketch Arduino `arduino_cmd.ino` attraverso l'apposito ambiente di sviluppo e lanciare il comando

`rosrun rgbd-slam-with-teleop-vehicle teleop_listener.py`

##### Acquisizione e visualizzazione utilizzando il solo robot mobile

Lanciare il comando

`freenect-glview`

##### Acquisizione mediante logica client-server

Lanciare i comandi

``` bash
roslaunch freenect_launch freenect.launch depth_registration:=true
rosrun rgbd-slam-with-teleop-vehicle kinect_listener.py
````

#### Lato client - Computer remoto

Lanciare il comando

`rosrun  rgbd-slam-with-teleop-vehicle teleop_talker.py`

##### Acquisizione mediante logica client-server

Lanciare il comando

`rosrun  rgbd-slam-with-teleop-vehicle kinect_visualizer.py`

### Guida teleoperata con acquisizione delle immagini e RGB-D SLAM

[![](https://img.youtube.com/vi/qfKFda2wjDI/0.jpg)](https://youtu.be/qfKFda2wjDI)

#### Lato server - Robot mobile

Fare l'upload dello sketch Arduino `arduino_cmd.ino` attraverso l'apposito ambiente di sviluppo, lanciare i comandi

``` bash
rosrun rgbd-slam-with-teleop-vehicle teleop_listener.py
rtabmap
```

Aperto RTAB-Map, occorre configurarlo attraverso i seguenti passaggi:
- selezionare la voce *Detection ⇒ Select Source ⇒ RGB-D Camera ⇒ Kinect ⇒ Freenect*
- selezionare la voce *File ⇒ New Database*
- selezionare *Detection ⇒ Start*

#### Lato client - Computer remoto

Lanciare il comando

`rosrun  rgbd-slam-with-teleop-vehicle teleop_talker.py`
