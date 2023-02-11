import board
import pwmio
import time
import digitalio
import RPi.GPIO as GPIO
from threading import Thread
from threading import Event
from enum import Enum
from Adafruit_ADS1x15 import ADS1115

# ================================
# Declaracao de constantes
# ================================
FREQUENCIA_PWM = 100 # Hz
DISCO_ENCODER_PULSOS = 20
POTENCIA_MINIMA = 10
POTENCIA_MAXIMA = 50
POTENCIA_STEP = 1
DELTA_T = 0.2 # seg
RPM_SET_POINT = [170, 100] # 170 RPM / aprox. 0,62 m/s
ADC_GAIN = 2/3
TENSAO_BATERIA_MAXIMA = 7.40 # V
TENSAO_BATERIA_MINIMA = 5.40 # V
TAMANHO_BUFFER = 4

class Estado(Enum):
    RETA = 0
    CURVA = 1
    PARADO = 2

class Robo_Rasp_Zero_W:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)
        self.estado = Estado.PARADO

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

        GPIO.setup(22, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) # pino 15
        GPIO.add_event_detect(22, GPIO.RISING, callback = self.ler_encoder_motor_esquerdo, bouncetime = 1)
        self.esq_contador = 0
        self.esq_rpm = 0
        self.esq_potencia = [POTENCIA_MINIMA, POTENCIA_MINIMA]
        GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) # pino 16
        GPIO.add_event_detect(23, GPIO.RISING, callback = self.ler_encoder_motor_direito, bouncetime = 1)
        self.dir_contador = 0
        self.dir_rpm = 0
        self.dir_potencia = [POTENCIA_MINIMA, POTENCIA_MINIMA]

        self.esq_ultra_trig = digitalio.DigitalInOut(board.D27) # pino 13
        self.esq_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.esq_ultra_echo = digitalio.DigitalInOut(board.D25) # pino 22
        self.esq_ultra_echo.direction = digitalio.Direction.INPUT
        self.esq_ultra_distancia = [0, 0, 0]
        self.idx_esq_ultra_distancia = 0

        self.meio_ultra_trig = digitalio.DigitalInOut(board.D5) # pino 29
        self.meio_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.meio_ultra_echo = digitalio.DigitalInOut(board.D6) # pino 31
        self.meio_ultra_echo.direction = digitalio.Direction.INPUT
        self.meio_ultra_distancia = [0, 0, 0]
        self.idx_meio_ultra_distancia = 0

        self.dir_ultra_trig = digitalio.DigitalInOut(board.D12) # pino 32
        self.dir_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.dir_ultra_echo = digitalio.DigitalInOut(board.D13) # pino 33
        self.dir_ultra_echo.direction = digitalio.Direction.INPUT
        self.dir_ultra_distancia = [0, 0, 0]
        self.idx_dir_ultra_distancia = 0

        self.adc = ADS1115()
        self.tensao_bateria = 0

        self.event = Event()

        self.thread_controlar_motores = Thread(
            target = self.controlar_motores,
            args = (self.event,)
        )

        self.thread_ler_tensao_bateria = Thread(
            target = self.ler_tensao_bateria,
            args = (self.event,)
        )

        self.thread_ler_sensor_ultra_esquerdo = Thread(
            target = self.ler_sensor_ultra_esquerdo,
            args = (self.event,)
        )

        self.thread_ler_sensor_ultra_meio = Thread(
            target = self.ler_sensor_ultra_meio,
            args = (self.event,)
        )

        self.thread_ler_sensor_ultra_direito = Thread(
            target = self.ler_sensor_ultra_direito,
            args = (self.event,)
        )

    def iniciar(self):
        self.thread_controlar_motores.start()
        self.thread_ler_tensao_bateria.start()
        self.thread_ler_sensor_ultra_esquerdo.start()
        self.thread_ler_sensor_ultra_meio.start()
        self.thread_ler_sensor_ultra_direito.start()

    def encerrar(self):
        self.parar_movimento()
        self.event.set()

    def mover_generico(self, dir_frente, dir_tras, esq_frente, esq_tras):
        self.dir_frente.value = dir_frente
        self.dir_tras.value = dir_tras
        self.esq_frente.value = esq_frente
        self.esq_tras.value = esq_tras

    def mover_frente(self):
        self.estado = Estado.RETA
        self.mover_generico(True, False, True, False)

    def mover_tras(self):
        self.estado = Estado.RETA
        self.mover_generico(False, True, False, True)

    def mover_direita(self):
        self.estado = Estado.CURVA
        self.mover_generico(False, True, True, False)

    def mover_esquerda(self):
        self.estado = Estado.CURVA
        self.mover_generico(True, False, False, True)

    def parar_movimento(self):
        self.estado = Estado.PARADO
        self.mover_generico(False, False, False, False)

    def converter_duty_cycle(self, percentual):
        return int((2 ** 16 - 1) * percentual / 100)

    def setar_potencia_motor_esquerdo(self, percentual):
        self.esq_pwm.duty_cycle = self.converter_duty_cycle(percentual)

    def setar_potencia_motor_direito(self, percentual):
        self.dir_pwm.duty_cycle = self.converter_duty_cycle(percentual)

    def ler_encoder_motor_esquerdo(self, event):
        self.esq_contador = self.esq_contador + 1

    def ler_encoder_motor_direito(self, event):
        self.dir_contador = self.dir_contador + 1

    def controlar_motores(self, event):
        t = time.time()
        while True:
            try:
                if (time.time() - t >= DELTA_T):
                    self.esq_rpm = 60 * self.esq_contador / (DISCO_ENCODER_PULSOS * (time.time() - t))
                    self.dir_rpm = 60 * self.dir_contador / (DISCO_ENCODER_PULSOS * (time.time() - t))
                    self.ajustar_motores()
                    self.esq_contador = 0
                    self.dir_contador = 0
                    t = time.time()
                if event.is_set():
                    break
            except:
                continue

    def ajustar_motores(self):
        if self.estado == Estado.PARADO:
            return
        if self.esq_rpm > RPM_SET_POINT[self.estado.value]:
            self.diminuir_potencia_motor_esquerdo()
        elif self.esq_rpm < RPM_SET_POINT[self.estado.value]:
            self.aumentar_potencia_motor_esquerdo()
        if self.dir_rpm > RPM_SET_POINT[self.estado.value]:
            self.diminuir_potencia_motor_direito()
        elif self.dir_rpm < RPM_SET_POINT[self.estado.value]:
            self.aumentar_potencia_motor_direito()
        self.setar_potencia_motor_esquerdo(self.esq_potencia[self.estado.value])
        self.setar_potencia_motor_direito(self.dir_potencia[self.estado.value])           

    def diminuir_potencia_motor_esquerdo(self):
        if self.esq_potencia[self.estado.value] > POTENCIA_MINIMA:
            self.esq_potencia[self.estado.value] = self.esq_potencia[self.estado.value] - POTENCIA_STEP

    def aumentar_potencia_motor_esquerdo(self):
        if self.esq_potencia[self.estado.value] < POTENCIA_MAXIMA:
            self.esq_potencia[self.estado.value] = self.esq_potencia[self.estado.value] + POTENCIA_STEP

    def diminuir_potencia_motor_direito(self):
        if self.dir_potencia[self.estado.value] > POTENCIA_MINIMA:
            self.dir_potencia[self.estado.value] = self.dir_potencia[self.estado.value] - POTENCIA_STEP

    def aumentar_potencia_motor_direito(self):
        if self.dir_potencia[self.estado.value] < POTENCIA_MAXIMA:
            self.dir_potencia[self.estado.value] = self.dir_potencia[self.estado.value] + POTENCIA_STEP

    def ler_tensao_bateria(self, event):
        while True:
            self.tensao_bateria = 2 * 6.144 * self.adc.read_adc(1, gain = ADC_GAIN) / (2 ** 15 - 1)
            if event.is_set():
                break
            time.sleep(DELTA_T)

    def mostrar_tensao_bateria(self):
        pct = 100 * (self.tensao_bateria - TENSAO_BATERIA_MINIMA) / (TENSAO_BATERIA_MAXIMA - TENSAO_BATERIA_MINIMA)
        if pct > 100:
            pct = 100
        elif pct < 0:
            pct = 0
        print("Bateria: {0:.2f}V ({1:.2f}%)".format(self.tensao_bateria, pct))
 
    def mostrar_estado(self):
        print("Estado: {0}".format(self.estado.name))

    def ler_sensor_ultra_esquerdo(self, event):
        while True:
            try:
                self.disparar_pulso_ultra(self.esq_ultra_trig)
                t = time.time()
                t0 = t
                while self.esq_ultra_echo.value == False and (t0 - t) <= 0.05:
                    t0 = time.time()
                t = time.time()
                t1 = t
                while self.esq_ultra_echo.value == True and (t1 - t) <= 0.05:
                    t1 = time.time()
                self.esq_ultra_distancia[self.idx_esq_ultra_distancia] = 17150 * (t1 - t0)
                self.idx_esq_ultra_distancia = (self.idx_esq_ultra_distancia + 1) % TAMANHO_BUFFER
                if event.is_set():
                    break
                time.sleep(DELTA_T)
            except:
                continue

    def ler_sensor_ultra_meio(self, event):
        while True:
            try:
                self.disparar_pulso_ultra(self.meio_ultra_trig)
                t = time.time()
                t0 = t
                while self.meio_ultra_echo.value == False and (t0 - t) <= 0.05:
                    t0 = time.time()
                t = time.time()
                t1 = t
                while self.meio_ultra_echo.value == True and (t1 - t) <= 0.05:
                    t1 = time.time()
                self.meio_ultra_distancia[self.idx_meio_ultra_distancia] = 17150 * (t1 - t0)
                self.idx_meio_ultra_distancia = (self.idx_meio_ultra_distancia + 1) % TAMANHO_BUFFER
                if event.is_set():
                    break
                time.sleep(DELTA_T)
            except:
                continue

    def ler_sensor_ultra_direito(self, event):
        while True:
            try:
                self.disparar_pulso_ultra(self.dir_ultra_trig)
                t = time.time()
                t0 = t
                while self.dir_ultra_echo.value == False and (t0 - t) <= 0.05:
                    t0 = time.time()
                t = time.time()
                t1 = t
                while self.dir_ultra_echo.value == True and (t0 - t) <= 0.05:
                    t1 = time.time()
                self.dir_ultra_distancia[self.idx_dir_ultra_distancia] = 17150 * (t1 - t0)
                self.idx_dir_ultra_distancia = (self.idx_dir_ultra_distancia + 1) % TAMANHO_BUFFER
                if event.is_set():
                    break
                time.sleep(DELTA_T)
            except:
                continue

    def disparar_pulso_ultra(self, trigger):
        trigger.value = True
        time.sleep(0.00001) # 10 us
        trigger.value = False

    def mostrar_sensor_ultra_distancia(self):
        print("Esquerda: {0:.2f}\nMeio: {1:.2f}\nDireita: {2:.2f}".format(self.calcular_media_movel(self.esq_ultra_distancia), self.calcular_media_movel(self.meio_ultra_distancia), self.calcular_media_movel(self.dir_ultra_distancia)))

    def calcular_media_movel(self, vetor):
        sum = 0
        for v in vetor:
            sum = sum + v
        return sum / TAMANHO_BUFFER

    def mostrar_estado_motores(self):
        if self.estado == Estado.PARADO:
            return
        print("[{0}][{1}];[{2}][{3}]".format(self.esq_potencia[self.estado.value], self.esq_rpm, self.dir_potencia[self.estado.value], self.dir_rpm))        

