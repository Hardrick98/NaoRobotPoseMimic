# NaoRobotPoseMimic

Per eseguire il codice assicurarsi di avere una webcam funzionante. 

## 1. Attivare La Simulazione su Gazebo

```
roslaunch nao_moveit_config demo_gazebo.launch
```

## 2. Attivare la stima della Posa 3D con Webcam 

```
roslaunch pose_estimation pose.launch
```
Il sistema di Posa3D riprende e adatta lo splendido lavoro fatto da https://github.com/facebookresearch/VideoPose3D

## 3. Attivare Computazione Traiettoria (In questo momento solo del braccio destro)

```
roslaunch nao_trajectory trajectory.launch
```

## PROSSIMI STEP

### Ricalibrare Sistema di Riferimento
### Risolvere Problema Controllo
### Eventualmente Aggiungere Orientamento Mano
### Eventualmente aggiungere posizione del braccio
