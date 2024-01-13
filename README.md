# Robot de carreras F1

Este projecto de ROS simula (en un entorno Stage) un robot sigue lineas que simula mediante una máquina de estados los comportamientos de los coches de la Formula 1.

## Mundos

Para ejecutar la simulación, se han creado tres circuitos. Los circuitos contienen un recorrido negro y banderas de colores, además de una línea de meta (señales horizontal y vertical) y un muro de parada en boxes.

## Compilación

Si no tienes un workspace creado, puedes hacerlo de la siguiente manera:
```console
mkdir your_ws
cd your_ws
mkdir src
cd src
```

Una vez creado el directorio, clona el repositorio en la carpeta _src_ y compilalo:
```console
cd ~/your_ws/src
git clone https://github.com/elestofao/final_project.git
cd ..
catkin_make
```

## Permisos de ejecución
Una vez hayas compilado, hay que darle permisos de ejecución a los archivos _.py_. Para esto, ve al directorio donde se encuentren los archivos y da permisos a todos los archivos:
```console
cd ~/your_ws/src/final_project/src
chmod +x *
```

## Launch
Para lanzar la simulación en Stage:
```console
cd ~/your_ws
roslaunch final_project racing.launch
```

