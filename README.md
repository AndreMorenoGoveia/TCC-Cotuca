# TCC-Cotuca
O algoritmo possibilita a máquina do projeto a conseguir a localização de maçãs maduras e configura seu braço robótico a 
ir até as coordenadas adiquiridas e coletar a maçã, até que não hajam mais maçãs maduras no escopo da visão do robô.


# Visão inteligente
Com um arquivo .xml com as features gerada pelas imagens do objeto, consegue suas coordenadas dos eixos x e y no espaço utilizando
a biblioteca OpenCV. Para conseguir a distância do objeto em reação ao eixo z é utilizada a biblioteca pyrealsense2 e assim é
possível fornecer as coordenadas da maçã para que a garra a busque


# Movimentação do braço
O braço robótico é um braço RRR, ou seja, com três motores rotativos que, atuando juntos, conseguem levar a garra do robô às
coordenadas fornecidas. Por isso existem no algoritmo diversas operações matemáticas para obter-se as o ângulo de rotação de
cada servo-motor e chegar até as coordenadas. A biblioteca gpiozero é utilizada para o funcionamento adequado dos servo-motores.
