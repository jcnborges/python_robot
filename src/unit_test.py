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

    time.sleep(1)

    print("Teste da roda direita...")

    time.sleep(1)

    for i in range(0, 110, 10):
        print("Roda direita frente ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        robo.mover_generico(True, False, False, False)
        time.sleep(2)

    time.sleep(1)

    for i in range(0, 110, 10):
        print("Roda direita tras ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        robo.mover_generico(False, True, False, False)
        time.sleep(2)    

    time.sleep(1)

    print("Encerrando teste dos motores!")

def testar_movimentos(pot):
    print("Iniciando testes dos movimentos...")

    time.sleep(1)   

    print("Ajustando potencia dos motores para {0}%...".format(pot))
    robo.setar_potencia_motor_direito(pot)
    robo.setar_potencia_motor_esquerdo(pot)

    time.sleep(1)   

    print("Movendo robo para frente...")
    robo.mover_frente()

    time.sleep(5)   

    print("Movendo robo para tras...")
    robo.mover_tras()

    time.sleep(5)   

    print("Movendo robo para direita...")
    robo.mover_direita()

    time.sleep(5)   

    print("Movendo robo para esquerda...")
    robo.mover_esquerda()

    time.sleep(5)   

    print("Encerrando teste dos movimentos!")

testar_rodas()
testar_movimentos(30)
