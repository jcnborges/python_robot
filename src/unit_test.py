import time
from core.robo_rasp_zero_w import Robo_Rasp_Zero_W

robo = Robo_Rasp_Zero_W()

def testar_rodas():

    print("Iniciando testes das rodas...")

    robo.mover_generico(False, False, False, False)

    time.sleep(1)

    print("Teste da roda esquerda...")

    time.sleep(1)

    robo.mover_generico(False, False, True, False)
    for i in range(0, 110, 10):
        print("Roda esquerda frente ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        time.sleep(2)

    time.sleep(1)

    robo.mover_generico(False, False, False, True)
    for i in range(0, 110, 10):
        print("Roda esquerda tras ({0}%)...".format(i))
        robo.setar_potencia_motor_esquerdo(i)
        time.sleep(2)

    robo.mover_generico(False, False, False, False)

    time.sleep(1)

    print("Teste da roda direita...")

    time.sleep(1)

    robo.mover_generico(True, False, False, False)
    for i in range(0, 110, 10):
        print("Roda direita frente ({0}%)...".format(i))
        robo.setar_potencia_motor_direito(i)
        time.sleep(2)

    time.sleep(1)

    robo.mover_generico(False, True, False, False)
    for i in range(0, 110, 10):
        print("Roda direita tras ({0}%)...".format(i))
        robo.setar_potencia_motor_direito(i)
        time.sleep(2)    

    robo.mover_generico(False, False, False, False)

    time.sleep(1)

    print("Encerrando teste dos motores!")

def testar_movimentos(pot):

    print("Iniciando testes dos movimentos...")

    robo.parar_movimento()

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

    print("Parando o movimento do robo...")

    robo.parar_movimento()

    time.sleep(2)

    print("Encerrando teste dos movimentos!")

def testar_velocidades():
    
    robo.setar_velocidade_motor_esquerdo(150)
    robo.setar_velocidade_motor_direito(150)
    robo.mover_generico(True, False, True, False)
    robo.iniciar()

    time.sleep(10)

    robo.encerrar()

    

#testar_rodas()
#testar_movimentos(50)
testar_velocidades()
#parar_teste()
