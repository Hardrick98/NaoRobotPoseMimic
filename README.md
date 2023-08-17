# NaoRobotPoseMimic

Devo ancora aggiornare la cartella del Robot.

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

### Step 3
Creare un nodo trajectory per ogni move_group e dargli in input i valori corrispondenti del nodo precedente

### Step 4
Sincronizzare il tutto. Non penso convenga fare Pose Estimation da WebCAM, potremmo comunque trovare delle applicazioni utili (Tipo registrare le pose)
Da valutare anche fattore di equilibrio. 
