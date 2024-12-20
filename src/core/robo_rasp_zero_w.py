import board
import time
import digitalio
import RPi.GPIO as GPIO
import math
from threading import Thread
from threading import Event
from .motor_controller import Motor_Controller

# ================================
# Declaracao de constantes
# ================================
DELTA_T = 0.2 # seg
TENSAO_BATERIA_MAXIMA = 7.40 # V
TENSAO_BATERIA_MINIMA = 5.40 # V
TAMANHO_BUFFER = 5
SLAVE_ADDRESS = 4
LINEAR_VELOCITY = 11.3 # cm/s
ANGULAR_VELOCITY = 34 # graus/s

class Robo_Rasp_Zero_W:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)
        self.x = 128
        self.y = 128
        self.linear_velocity = 0
        self.angular_velocity = 0
        
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

    def set_x(self, x):
        self.x = x
        self.set_velocity()

    def set_y(self, y):
        self.y = y
        self.set_velocity()

    def set_velocity(self):
        self.angular_velocity = Robo_Rasp_Zero_W.adjust_value(-ANGULAR_VELOCITY, self.x, True if self.linear_velocity >= 0 else False)
        self.linear_velocity = Robo_Rasp_Zero_W.adjust_value(-LINEAR_VELOCITY, self.y, True if self.linear_velocity >= 0 else False)
        self.motor_controller.set_velocity(self.linear_velocity / 100, math.radians(self.angular_velocity))

    def encerrar(self):        
        self.event.set()

    def ler_tensao_bateria(self, event):
        while True:
            self.tensao_bateria = 15.6383 * self.motor_controller.read_adc_level() / 1023.0
            if event.is_set():
                break
            time.sleep(5 * DELTA_T)

    def mostrar_tensao_bateria(self):
        pct = 100 * (self.tensao_bateria - TENSAO_BATERIA_MINIMA) / (TENSAO_BATERIA_MAXIMA - TENSAO_BATERIA_MINIMA)
        if pct > 100:
            pct = 100
        elif pct < 0:
            pct = 0
        print("Bateria: {0:.2f}V ({1:.2f}%)".format(self.tensao_bateria, pct))
 
    def mostrar_velocidade(self):
        print("Vel.Linear (cm/s): {0:.2f}".format(self.linear_velocity))
        print("Vel.Angular (graus/s): {0:.2f}".format(self.angular_velocity))

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
        print("Ultra.Esquerda (cm): {0:.2f}\nUltra.Meio (cm): {1:.2f}\nUltra.Direita (cm): {2:.2f}".format(Robo_Rasp_Zero_W.calcular_media_movel(self.esq_ultra_distancia), Robo_Rasp_Zero_W.calcular_media_movel(self.meio_ultra_distancia), Robo_Rasp_Zero_W.calcular_media_movel(self.dir_ultra_distancia)))

    def mostrar_sensor_obstaculo(self):
        print("Obs.Esquerda: {0}\nObs.Direita: {1}".format(self.esq_obs.value, self.dir_obs.value))

    @staticmethod
    def adjust_value(amplitude: float, value: float, positive: bool) -> float:        
        if value == 128:
            return +1E-4 if positive else -1E-4
        elif value > 128:
            return Robo_Rasp_Zero_W.sigmoid(amplitude, 192, 32, value)
        else:
            return Robo_Rasp_Zero_W.sigmoid(amplitude, 64, 32, value, True, True)

    @staticmethod
    def calcular_media_movel(vetor):
        sum = 0
        for v in vetor:
            sum = sum + v
        return sum / TAMANHO_BUFFER
    
    @staticmethod
    def sigmoid(amplitude: float, center: float, slope: float, x: float, x_flip=False, y_flip=False) -> float:
        """
        Calculates the sigmoid function value.

        Args:
            amplitude: The maximum value of the sigmoid function.
            center: The x-coordinate of the sigmoid's midpoint (inflection point).
            slope: The steepness of the sigmoid curve.
            x: The input value for the sigmoid function.
            x_flip: If True, flips the sigmoid curve across the x-axis.
            y_flip: If True, flips the sigmoid curve across the y-axis.

        Returns:
            The sigmoid function value for the given input x.
        """

        # Protect against division by zero
        if slope == 0:
            return 0.0

        value = amplitude * (1 / (1 + math.exp(-(x - center) / slope)))

        # Flip the sigmoid curve if requested
        if y_flip:
            value = amplitude - value
        
        if x_flip:
            value = -value

        print(value)

        return value
