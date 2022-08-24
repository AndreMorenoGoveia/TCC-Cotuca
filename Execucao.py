#Biblioteca importada
import cv2
from gpiozero import AngularServo
import math
import numpy as np
from time import sleep
import pyrealsense2 as rs

 
#Arquivo gerado para detecção de objetos
path = 'cascadeApple.xml'


#Configurações de câmera
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)


#Carrega o arquivo gerado
cascade = cv2.CascadeClassifier(path)


# Define a câmera infravermelho
class DepthCamera:
    def __init__(self):

        # Configura profundidade e color streams
        self.pipeline = rs.pipeline()
        config = rs.config()


        # Pega a linha do seu dispositivo para definir uma resolução suportada
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


        # Inicia a transmissão
        self.pipeline.start(config)        

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()


# Inicia Camera infravermelho  
dc = DepthCamera()


# Função de configuração de câmera infravermelho
def GetDepth(x,y,w,Coords):


	# Mede a distância até o ponto
    z = depth_frame[y, x]
    Coords.append([x, y, z, w])



# Função que pega as informações da maçã detectada
def getObjects():

	#Configurando a luminosidade
	BrilhoCam = cv2.getTrackbarPos("Luminosidade", "Resultado") #Definir 
	cap.set(10, BrilhoCam)


	#Pega a imagem da câmera e configura
	succes, img =  cap.read()
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


	#Detecta a maçã com o arquivo gerado
	scaleVal = 1 + (cv2.getTrackbarPos("Escala", "Resultado") / 1000) #Definir
	neig = cv2.getTrackbarPos("Neig", "Resultado") # Definir
	objects = cascade.detectMultiScale(gray, scaleVal, neig)


	#Destaca os objetos detectados e descarta os objetos repetidos
	Coords = []
	for(x,y,w,h) in objects:
		area = w*h
		minArea = cv2.getTrackbarPos("Area Min", "Resultado") #Definir
		if area > minArea:
			if Coords:
				for i in range(len(CoordXY)):
					if x - 20 > Coords[i][0] and x + 20 < Coords[i][0] and y - 20 > Coords[i][1] and y + 20 < Coords[i][0]:
						GetDepth(x,y,w,Coords)
					else:
						getObjects()
			else:
				GetDepth(x,y,w,Coords)

# Define os pinos de entrada e configurações de cada servo
servo1 = AngularServo(1, min_angle = -90, max_angle = 90, min_pulse_width = 0.0005, max_pulse_width = 0.0025)
servo2 = AngularServo(2, min_angle = -90, max_angle = 90, min_pulse_width = 0.0005, max_pulse_width = 0.0025)
servo3 = AngularServo(3, min_angle = -90, max_angle = 90, min_pulse_width = 0.0005, max_pulse_width = 0.0025)
servo4 = AngularServo(4, min_angle = -90, max_angle = 90, min_pulse_width = 0.0005, max_pulse_width = 0.0025)
servo5 = AngularServo(5, min_angle = -90, max_angle = 90, min_pulse_width = 0.0005, max_pulse_width = 0.0025)


# Define o ângulo inicial do processo
Ag5 = 90
servo1.angle = 0
servo2.angle = 45
servo3.angle = -45
servo4.angle = 0
servo5.angle = Ag5
sleep(1)
getObjects()


# Define o comprimento dos braços
L = 500


# Define o ponto em que a garra tem que chegar e a largura da maçã
if Coords:
	xg = 100000
	yg = 100000
	zg = 100000
	wg = 100000
	for i in range(len(Coords)):
		if Coords[i][0] < xg:
			xg = Coords[i][0]
			yg = Coords[i][1]
			zg = Coords[i][2]
			wg = Coords[i][3]


	# Equação dos ângulos dos servos
	if zg != 0:
		T = math.degrees(math.atan(-(xg/zg)))
	elif zg == 0 and xg < 0:
		T = 90
	elif zg == 0 and xg == 0:
		T = 0
	else:
		T = -90

	A = math.degrees(math.acos((zg**2 + xg**2 + (yg - L)**2 - 2*L**2)/(2*L**2)))
	if abs(A*(-1)-(-45)) < abs(A-(-45)):	
		A = A*(-1)

	den = ((math.sqrt(zg**2 + xg**2))*(1 + math.cos(math.radians(A))) + math.sin(math.radians(A))*(yg - L))
	if den == 0:
		den = den + 0.001

	B = math.degrees(math.atan(((1 + math.cos(math.radians(A)))*(yg - L) - (math.sqrt(zg**2 + xg**2))*math.sin(math.radians(A)))/den))

	# Executa os ângulos calculados
	servo1.angle = T
	servo2.angle = B
	servo3.angle = A
	sleep(2)


	# Equação do ângulo da garra
	G = math.acos((50 - wg)/100)


	# Executa o ângulo calculado e o último servo
	servo4.angle = G
	sleep(1)
	servo5.angle = Ag5*(-1)
	sleep(1)


	# Reativa a função de objetos
	getObjects()
else:
	getObjects()