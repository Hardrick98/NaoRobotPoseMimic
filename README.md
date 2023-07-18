# NaoRobotPoseMimic

## To activate Simulation
```
roslaunch nao_moveit_config demo_gazebo.launch
```
## To send Trajectory
```
roslaunch nao_trajectory trajectory.launch
```
Cambiare i valori all'interno del file command_trajectory.py

## PROSSIMI STEP

Al momento possiamo calcolare gli angoli tra alcuni link tramite la pose estimation di un immagine 2D, che ovviamente non è esaustiva. Servirà una pose Estimation 3D per calcolare tutti gli angoli.

### Step 1
Pose 3D

### Step 2
Creare nodo che fa la Pose 3D e pubblica in output i valori degli angoli dei vari joints

### Step 3
Creare un nodo trajectory per ogni move_group e dargli in input i valori corrispondenti del nodo precedente

### Step 4
Sincronizzare il tutto. Non penso convenga fare Pose Estimation da WebCAM, potremmo comunque trovare delle applicazioni utili (Tipo registrare le pose)
Da valutare anche fattore di equilibrio. 
