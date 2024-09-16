import board
import pwmio
import time
import digitalio
import RPi.GPIO as GPIO
from threading import Thread
from threading import Event
from enum import Enum
from Adafruit_ADS1x15 import ADS1115
from .motor_controller import Motor_Controller

# ================================
# Declaracao de constantes
# ================================
DELTA_T = 0.2 # seg
ADC_GAIN = 2/3
TENSAO_BATERIA_MAXIMA = 7.40 # V
TENSAO_BATERIA_MINIMA = 5.40 # V
TAMANHO_BUFFER = 5
SLAVE_ADDRESS = 4

class Estado(Enum):
    RETA = 0
    CURVA = 1
    PARADO = 2

class Robo_Rasp_Zero_W:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)
        self.estado = Estado.PARADO
        
        self.esq_obs = digitalio.DigitalInOut(board.D19) # pino 35
        self.dir_obs = digitalio.DigitalInOut(board.D16) # pino 36

        self.esq_ultra_trig = digitalio.DigitalInOut(board.D27) # pino 13
        self.esq_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.esq_ultra_echo = digitalio.DigitalInOut(board.D25) # pino 22
        self.esq_ultra_echo.direction = digitalio.Direction.INPUT
        self.esq_ultra_distancia = [0, 0, 0, 0, 0]
        self.idx_esq_ultra_distancia = 0

        self.meio_ultra_trig = digitalio.DigitalInOut(board.D5) # pino 29
        self.meio_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.meio_ultra_echo = digitalio.DigitalInOut(board.D6) # pino 31
        self.meio_ultra_echo.direction = digitalio.Direction.INPUT
        self.meio_ultra_distancia = [0, 0, 0, 0, 0]
        self.idx_meio_ultra_distancia = 0

        self.dir_ultra_trig = digitalio.DigitalInOut(board.D12) # pino 32
        self.dir_ultra_trig.direction = digitalio.Direction.OUTPUT
        self.dir_ultra_echo = digitalio.DigitalInOut(board.D13) # pino 33
        self.dir_ultra_echo.direction = digitalio.Direction.INPUT
        self.dir_ultra_distancia = [0, 0, 0, 0, 0]
        self.idx_dir_ultra_distancia = 0

        self.adc = ADS1115()
        self.tensao_bateria = 0

        self.motor_controller = Motor_Controller(SLAVE_ADDRESS)

        self.event = Event()

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
        self.thread_ler_tensao_bateria.start()
        self.thread_ler_sensor_ultra_esquerdo.start()
        self.thread_ler_sensor_ultra_meio.start()
        self.thread_ler_sensor_ultra_direito.start()

    def mover_frente(self):
        self.estado = Estado.RETA
        self.motor_controller.send_data(1, 150)
        self.motor_controller.send_data(2, 150)

    def mover_tras(self):
        self.estado = Estado.RETA
        self.motor_controller.send_data(1, -150)
        self.motor_controller.send_data(2, -150)


    def mover_direita(self):
        self.estado = Estado.CURVA
        self.motor_controller.send_data(1, 0)
        self.motor_controller.send_data(2, 150)


    def mover_esquerda(self):
        self.estado = Estado.CURVA
        self.motor_controller.send_data(1, 150)
        self.motor_controller.send_data(2, 0)


    def parar_movimento(self):
        self.estado = Estado.PARADO
        self.motor_controller.send_data(1, 0)
        self.motor_controller.send_data(2, 0)

    def encerrar(self):        
        self.event.set()

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
        print("Ultra.Esquerda: {0:.2f}\nUltra.Meio: {1:.2f}\nUltra.Direita: {2:.2f}".format(self.calcular_media_movel(self.esq_ultra_distancia), self.calcular_media_movel(self.meio_ultra_distancia), self.calcular_media_movel(self.dir_ultra_distancia)))

    def mostrar_sensor_obstaculo(self):
        print("Obs.Esquerda: {0}\nObs.Direita: {1}".format(self.esq_obs.value, self.dir_obs.value))

    def calcular_media_movel(self, vetor):
        sum = 0
        for v in vetor:
            sum = sum + v
        return sum / TAMANHO_BUFFER