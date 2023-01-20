import board
import pwmio
import time
import digitalio
from threading import Thread
from threading import Event

# ================================
# Declaracao de constantes
# ================================
FREQUENCIA_PWM = 5000
TEMPO_PAUSA_MOTOR = 0.2
DISCO_ENCODER_PULSOS = 20
POTENCIA_MINIMA = 60
POTENCIA_MAXIMA = 70

class Robo_Rasp_Zero_W:

    def __init__(self):

        self.esq_pwm = pwmio.PWMOut(board.D17, frequency = FREQUENCIA_PWM, duty_cycle = 0) # pino 11
        self.esq_frente = digitalio.DigitalInOut(board.D16) # pino 36
        self.esq_frente.direction = digitalio.Direction.OUTPUT
        self.esq_tras = digitalio.DigitalInOut(board.D19) # pino 35
        self.esq_tras.direction = digitalio.Direction.OUTPUT

        self.dir_pwm = pwmio.PWMOut(board.D18, frequency = FREQUENCIA_PWM, duty_cycle = 0) # pino 12
        self.dir_frente = digitalio.DigitalInOut(board.D26) # pino 37
        self.dir_frente.direction = digitalio.Direction.OUTPUT
        self.dir_tras = digitalio.DigitalInOut(board.D20) # pino 38
        self.dir_tras.direction = digitalio.Direction.OUTPUT

        self.esq_encoder = digitalio.DigitalInOut(board.D22) # pino 15
        self.esq_encoder.direction = digitalio.Direction.INPUT
        self.esq_contador = 0
        self.esq_rpm = 0
        self.esq_rpm_set_point = 0
        self.esq_potencia = POTENCIA_MINIMA
        self.dir_encoder = digitalio.DigitalInOut(board.D23) # pino 16
        self.dir_encoder.direction = digitalio.Direction.INPUT
        self.dir_contador = 0
        self.dir_rpm = 0
        self.dir_rpm_set_point = 0
        self.dir_potencia = POTENCIA_MINIMA

        self.event = Event()

        self.thread_encoder_esquerdo = Thread(
            target = self.ler_encoder_esquerdo,
            args = (self.event,)
        )

        self.thread_encoder_direito = Thread(
            target = self.ler_encoder_direito,
            args = (self.event,)
        )

        self.thread_controlar_motores = Thread(
            target = self.controlar_motores,
            args = (self.event,)
        )

    def iniciar(self):
        self.thread_encoder_esquerdo.start()
        self.thread_encoder_direito.start()
        self.thread_controlar_motores.start()

    def encerrar(self):
        self.parar_movimento()
        self.event.set()

    def mover_generico(self, dir_frente, dir_tras, esq_frente, esq_tras):
        self.dir_frente.value = dir_frente
        self.dir_tras.value = dir_tras
        self.esq_frente.value = esq_frente
        self.esq_tras.value = esq_tras

    def mover_frente(self):
        self.mover_generico(True, False, True, False)

    def mover_tras(self):
        self.mover_generico(False, True, False, True)

    def mover_direita(self):
        self.mover_generico(False, True, True, False)

    def mover_esquerda(self):
        self.mover_generico(True, False, False, True)

    def parar_movimento(self):
        self.mover_generico(False, False, False, False)

    def converter_duty_cycle(self, percentual):
        return int((2 ** 16 - 1) * percentual / 100)

    def setar_potencia_motor_esquerdo(self, percentual):
        self.esq_pwm.duty_cycle = self.converter_duty_cycle(percentual)

    def setar_potencia_motor_direito(self, percentual):
        self.dir_pwm.duty_cycle = self.converter_duty_cycle(percentual)

    def setar_velocidade_motor_esquerdo(self, rpm):
        self.esq_rpm_set_point = rpm

    def setar_velocidade_motor_direito(self, rpm):
        self.dir_rpm_set_point = rpm

    def ler_encoder_esquerdo(self, event):
        estado = self.esq_encoder.value
        while True:
            if self.esq_encoder.value != estado:
                self.esq_contador = self.esq_contador + 1
                estado = self.esq_encoder.value
            if event.is_set():
                break

    def ler_encoder_direito(self, event):
        estado = self.dir_encoder.value
        while True:
            if self.dir_encoder.value != estado:
                self.dir_contador = self.dir_contador + 1
                estado = self.dir_encoder.value
            if event.is_set():
                break

    def controlar_motores(self, event):
        t = int(1000 * time.time())
        while True:
            if (int(1000 * time.time()) - t >= 1000):
                self.esq_rpm = int((60 * 1000 / DISCO_ENCODER_PULSOS) / (int(1000 * time.time()) - t)) * self.esq_contador
                self.dir_rpm = int((60 * 1000 / DISCO_ENCODER_PULSOS) / (int(1000 * time.time()) - t)) * self.dir_contador
                self.ajustar_motores()
                self.esq_contador = 0
                self.dir_contador = 0
                t = int(1000 * time.time())
            if event.is_set():
                break

    def ajustar_motores(self):
        if self.esq_rpm > self.dir_rpm:
            self.diminuir_potencia_motor_esquerdo()
            self.aumentar_potencia_motor_direito()
        elif self.esq_rpm < self.dir_rpm:
            self.aumentar_potencia_motor_esquerdo()
            self.diminuir_potencia_motor_direito()
        self.setar_potencia_motor_esquerdo(self.esq_potencia)
        self.setar_potencia_motor_direito(self.dir_potencia)
        print("[{0}][{1}][{2}];[{0}][{1}][{2}]".format(self.esq_potencia, self.esq_rpm, self.esq_rpm_set_point, self.dir_potencia, self.dir_rpm, self.dir_rpm_set_point))

    def diminuir_potencia_motor_esquerdo(self):
        if self.esq_potencia > POTENCIA_MINIMA:
            self.esq_potencia = self.esq_potencia - 1

    def aumentar_potencia_motor_esquerdo(self):
        if self.esq_potencia < POTENCIA_MAXIMA:
            self.esq_potencia = self.esq_potencia + 1

    def diminuir_potencia_motor_direito(self):
        if self.dir_potencia > POTENCIA_MINIMA:
            self.dir_potencia = self.dir_potencia - 1

    def aumentar_potencia_motor_direito(self):
        if self.dir_potencia < POTENCIA_MAXIMA:
            self.dir_potencia = self.dir_potencia + 1            

