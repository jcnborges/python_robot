import board
import pwmio
import time
import digitalio

# ================================
# Declaracao de constantes
# ================================
FREQUENCIA_PWM = 1000
TEMPO_PAUSA_MOTOR = 0.2

class Robo_Rasp_Zero_W:

    def __init__(self):

        self.esq_pwm = pwmio.PWMOut(board.D17, frequency = FREQUENCIA_PWM, duty_cycle = 0) # pino 11
        self.esq_frente = digitalio.DigitalInOut(board.D19) # pino 35
        self.esq_frente.direction = digitalio.Direction.OUTPUT
        self.esq_tras = digitalio.DigitalInOut(board.D16) # pino 36
        self.esq_tras.direction = digitalio.Direction.OUTPUT

        self.dir_pwm = pwmio.PWMOut(board.D18, frequency = FREQUENCIA_PWM, duty_cycle = 0) # pino 12
        self.dir_frente = digitalio.DigitalInOut(board.D26) # pino 37
        self.dir_frente.direction = digitalio.Direction.OUTPUT
        self.dir_tras = digitalio.DigitalInOut(board.D20) # pino 38
        self.dir_tras.direction = digitalio.Direction.OUTPUT

    def mover_generico(self, dir_frente, dir_tras, esq_frente, esq_tras):
        self.dir_frente.value = dir_frente
        self.dir_tras.value = dir_tras
        self.esq_frente.value = esq_frente
        self.esq_tras.value = esq_tras

    def mover_frente(self):
        self.parar_movimento()
        time.sleep(TEMPO_PAUSA_MOTOR)
        self.mover_generico(True, False, True, False)

    def mover_tras(self):
        self.parar_movimento()
        time.sleep(TEMPO_PAUSA_MOTOR)
        self.mover_generico(False, True, False, True)

    def mover_direita(self):
        self.parar_movimento()
        time.sleep(TEMPO_PAUSA_MOTOR)
        self.mover_generico(False, True, True, False)

    def mover_esquerda(self):
        self.parar_movimento()
        time.sleep(TEMPO_PAUSA_MOTOR)
        self.mover_generico(True, False, False, True)

    def parar_movimento(self):
        self.mover_generico(False, False, False, False)

    def converter_duty_cicle(self, percentual):
        return int(percentual * 2 ** 15 / 100)

    def setar_potencia_motor_esquerdo(self, percentual):
        self.esq_pwm.duty_cicle = self.converter_duty_cicle(percentual)

    def setar_potencia_motor_direito(self, percentual):
        self.dir_pwm.duty_cicle = self.converter_duty_cicle(percentual)        
