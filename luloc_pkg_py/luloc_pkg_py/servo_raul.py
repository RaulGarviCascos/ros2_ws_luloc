import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
from gpiozero import Servo, Button
from time import sleep, time
import sys
from rclpy.node import Node
from std_msgs.msg import Float32

from enum import IntEnum

class Mode(IntEnum):
    BAJANDO = 0
    SUBIENDO = 1
    ABAJO_PARADO = 2
    ARRIBA_PARADO = 3
    DONE = 10
    ERROR = 11
    PARADO = 12

def time_ms():
    return int(round(time() * 1000))

def time_ready_ms(last_time, delay):
    return time_ms() - last_time >= delay

servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
sensor_inf = Button(17, pull_up=False) 
sensor_sup = Button(27, pull_up=False)

print(">>> CONTROL POR BLOQUES ACTIVO <<<")

vel = 0.3



MODE = Mode.PARADO

#tiempos desde que empezo a subir o bajar
t_subida = 0
t_bajada = 0




while True: 
    puls_inf = sensor_inf.is_pressed
    puls_sup = not sensor_sup.is_pressed

    # print(f"{servo.value}   |   {MODE}   |   {puls_inf} | {puls_sup}")
    match MODE:

        case Mode.BAJANDO:
            servo.value = -vel
            if not puls_inf:
                MODE = Mode.PARADO

        case Mode.SUBIENDO:
            servo.value = vel   
            stop = False
            if time_ready_ms(t_subida, 500):
                stop = True         
            if puls_sup or stop:
                MODE = Mode.PARADO

        case Mode.ERROR:
            print("THERE WAS AN ERROR")
            MODE = Mode.DONE
            
        case Mode.PARADO:
            servo.detach()
            siguinte_modo = input("¿Qué quieres hacer? (bajar=0/subir=1/salir=-1): ").strip().lower()
            if siguinte_modo == "0":
                MODE = Mode.BAJANDO
                t_bajada = time_ms()
            elif siguinte_modo == "1":
                MODE = Mode.SUBIENDO
                t_subida = time_ms()
            elif siguinte_modo == "-1":
                MODE = Mode.DONE
            else:
                print("Opción no válida. Por favor, elige 'bajar', 'subir' o 'salir'.")

        case _:
            print("[BOT] Motor detenido. Cerrando programa.")
            sys.exit()

    sleep(0.01)