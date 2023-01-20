import time
from core.robo_rasp_zero_w import Robo_Rasp_Zero_W
from sshkeyboard import listen_keyboard

robo = Robo_Rasp_Zero_W()
robo.setar_velocidade_motor_esquerdo(150)
robo.setar_velocidade_motor_direito(150)
robo.iniciar()

def press(key):
    global potencia
    global potencia_giro
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
    elif key == "+":
        robo.encerrar()

def release(key):
    robo.parar_movimento()

listen_keyboard(on_press = press)
