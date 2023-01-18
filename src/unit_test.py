import time
from core.robo_rasp_zero_w import Robo_Rasp_Zero_W

robo = Robo_Rasp_Zero_W()

def testar_rodas():

    print("Iniciando testes das rodas...")

    time.sleep(1)

    print("Teste da roda esquerda...")

    time.sleep(1)

    for i in range(0, 110, 10):
        print("Roda esquerda frente ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        robo.mover_generico(False, False, True, False)
        time.sleep(2)

    time.sleep(1)

    for i in range(0, 110, 10):
        print("Roda esquerda tras ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        robo.mover_generico(False, False, False, True)
        time.sleep(2)

testar_rodas()
