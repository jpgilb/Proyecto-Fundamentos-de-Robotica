# Proyecto de Fundamentos de Robótica 2024-2
En este proyecto se modifica el diseño del robot Comau e.DO para agregarle un grado de libertad extra en forma de una articulación prismática. Es decir, este robot consiste en un manipulador de 7 GDL. Este nuevo robot manipulador tiene el propósito de asistir con la respiración de pacientes en una ambulancia.
Se definió la cinemática directa del modelo con la convención de Denavit-Hartenberg. Asimismo, se calcularon la cinemática inversa, el control cinemático y el control dinámico mediante funciones de ROS. Estos controladores fueron aplicados al visualizador RViz para observar su comportamiento y también al simulador Gazebo para analizar teniendo en cuenta los efectos dinámicos.

## Esquema del manipulador
```
      Base
       │
       │
       ├──> joint_1 (Rotación)
       │
       ├──> joint_2 (Rotación)
       │
       ├──> joint_3 (Rotación)
       │
       ├──> prismatic_joint (Prismática)
       │
       ├──> joint_4 (Rotación)
       │
       ├──> joint_5 (Rotación)
       │
       └──> joint_6 (Rotación)
             └── Gripper (Efector final)

```
## Ejecución
### Terminal 1 (para lanzar el entorno)
#### RViz
`roslaunch robot_description display.launch`
#### RViz (con sliders)
`roslaunch robot_description display_sliders.launch`
#### Gazebo (control de trayectoria)
`roslaunch robot_description robot_gazebo_control.launch`
#### Gazebo (control de esfuerzos)
`roslaunch robot_description robot_gazebo_control_effort.launch`

### Terminal 2 (para ejecutar script)
`rosrun proyecto <ejecutable>`

## Dependencias necesarias
- NumPy
- MatPlotLib
- RBDL (no se cargan los archivos del paquete a este repositorio. Es necesario que cada usuario clone el paquete en su propio workspace por su cuenta. Podría ocasionar errores relaciones a RBDL).

## Listado de archivos
```
proyecto/
├── src/
│   ├── __pycache__/
│   ├── control_pdg
│   ├── markers.py
│   ├── pfruncs.py
│   ├── test_diffkine
│   ├── test_diffkine_pose
│   ├── test_fdkine
│   ├── test_gazebo
│   ├── test_ikine
│   ├── validacion
│   ├── CMakeLists.txt
│   └── package.xml
robot_description/
└── launch/
    ├── display.launch
    ├── display_sliders.launch
    ├── gazebo.launch
    ├── robot_gazebo.launch
    ├── robot_gazebo_control.launch
    └── robot_gazebo_control_effort.launch
```
