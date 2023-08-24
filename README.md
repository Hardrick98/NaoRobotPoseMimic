# NaoRobotPoseMimic

Devo ancora aggiornare la cartella del Robot. Devo anche caricare il file Istruzioni qui.

## Link per VideoPose3D

https://github.com/facebookresearch/VideoPose3D

## 1. Attivare La Simulazione su Gazebo

```
roslaunch nao_moveit_config demo_gazebo.launch
```

## 2. Attivare la stima della Posa 3D con Webcam 

```
roslaunch pose_estimation pose.launch
```

## 3. Attivare Computazione Traiettoria 

```
roslaunch nao_trajectory trajectory.launch
```

## PROSSIMI STEP

### Ricalibrare Sistema di Riferimento
### Risolvere Problema Controllo
### Eventualmente Aggiungere Orientamento Mano
