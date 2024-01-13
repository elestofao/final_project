#Definicion box
define box model
(
  size [0.05 0.6 0.4]
  gui_nose 0
  color "blue"
)

define finishWall model
(
  size [0.05 0.1 0.5]
  gui_nose 0
  color "gray"
)

#Muros para el circuito
define wall1 model
(
  size [8 4 0.6]
  gui_nose 0
  color "gray"
)

define wall2 model
(
  size [2.5 4.5 0.6]
  gui_nose 0
  color "gray"
)

#Configuración de las banderas
define rflag model
(
  size [0.3 0.001 0.2]
  gui_nose 0
  color "red"
)

define gflag model
(
  size [0.3 0.001 0.2]
  gui_nose 0
  color "green"
)

define yflag model
(
  size [0.3 0.001 0.2]
  gui_nose 0
  color "yellow"
)

#Rango del sensor
define topurg ranger
(
	sensor( 			
    range [ 0.0  30.0 ]
    fov 240 #360 para que el sensor sea 360 grados
    samples 958
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define mycamera camera
(
	range [ 0.2 8.0 ]
	resolution [ 640 480 ]
	fov [ 60 49.5 ]
	pantilt [ 0 0 ]
	alwayson 1
)

define erratic position
(
  size [0.3 0.3 0.2]
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  topurg(pose [ 0.050 0.000 0 0.000 ])
  mycamera(pose [ 0 0 0 90.0 ])
  localization "odom"
)
