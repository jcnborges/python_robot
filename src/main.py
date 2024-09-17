import time
import os
from core.robo_rasp_zero_w import Robo_Rasp_Zero_W
from sshkeyboard import listen_keyboard
from threading import Thread

def press(key):
    if key == "8":
        robo.mover_frente()
    elif key == "2":
        robo.mover_tras()
    elif key == "6":
        robo.mover_direita()
    elif key == "4":
        robo.mover_esquerda()
    elif key == "5":
        robo.parar_movimento()
    elif key == "*":
        robo.encerrar()
    elif key == "/":
        robo.mostrar_tensao_bateria()

def mostrar_status():
    while True:
        os.system("clear")
        robo.mostrar_tensao_bateria()
        robo.mostrar_estado()
        robo.mostrar_sensor_ultra_distancia()
        robo.mostrar_sensor_obstaculo()
        time.sleep(0.2)

robo = Robo_Rasp_Zero_W()
robo.iniciar()

Thread(target = mostrar_status, args = ()).start()
listen_keyboard(on_press = press)

