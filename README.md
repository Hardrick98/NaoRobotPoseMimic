# NaoRobotPoseMimic

Devo ancora aggiornare la cartella del Robot. Devo anche caricare il file Istruzioni qui.

## Link per VideoPose3D

https://github.com/facebookresearch/VideoPose3D

## Per Attivare La Simulazione
```
roslaunch nao_moveit_config demo_gazebo.launch
```
## Per comandare braccio destro e sinistro tramite video 
ATTENZIONE! Deve essere presente il file joint.npy nella cartella VideoPose3D

```
roslaunch nao_trajectory trajectory.launch
```

## PROSSIMI STEP

### Fare il tutto in RealTime con WebCam
### Risolvere Problema Controllo
### Eventualmente Aggiungere Orientamento Mano
