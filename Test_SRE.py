#Hay que revisar los import por si hace falta alguno nuevo
import serial, time
import struct
from math import copysign, pi, floor
import sys
import select
##Mensajes



class HwClass:
# Function: __init__
#
# Comments
# ----------
# Constructor of the class HW__CLASS
# Define system constants.
#
# Parameters
# ----------
#
# Returns
# -------
    def __init__(self):
        #constant parameters
        #TODO: Estos parametros deberian estar
        #      en un servidor de parametros y leerlos.
            
        self.left = 0
        self.right = 1
        self.kdato = 36.28
        self.read_encoder_freq = 6
        self.pasos_vuelta= 356.3*2
 
        
        #Encoders data
        self.time_enc_left_last=0
        self.time_enc_right_last=0
        self.steps_enc_left_last=0
        self.steps_enc_right_last=0
        #TODO: Se debe permitir la apertura del puerto serie por
        #puerto y velocidad (parametros servidor)
        self.arduino = serial.Serial('COM4', baudrate = 115200,
                                     timeout = 1)
        time.sleep(0.01)


    def __del__(self):
        self.arduino.write(str('?').encode())
        self.arduino.close()
        
    def dum_first_read(self):
        self.arduino.write(str('N').encode())
        #data = self.arduino.readline()
        # TODO: Se puede chequear que N y P son el comienzo y
        #final de 3la trama
        comienzo = self.arduino.readline().decode("utf-8")
        print(comienzo) 
       # if (comienzo[0] != "N"):
       #     print("Error Rececpción"+time.ctime())
        steps_enc_left = struct.unpack('i',self.arduino.read(4))[0]        
        time_enc_left = struct.unpack('i',self.arduino.read(4))[0]     
        steps_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        final = self.arduino.readline().decode("utf-8")
        if (final[0] != "P"):
            print("Error Rececpción"+time.ctime())                       
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right
        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left

# Function: main
#
# Comments
# ----------
# Set up the node, the publications and subsciptions.
#
# Parameters
# ----------
#
# Returns
# -------
    def isData(self):
        try:
            return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        except:
            pass

    def enviar_velocidad(self,contador = 0):

        # Empezando para atrás
        wLeft = [1, 1, -1, -1, 2.3, -1, 2, -2, -2, 3]
        wRight = [1, -2.5, 1, -1.5, 1.5, -0, 2, -1.5, -2.5, 0]

        # Empezando girando a derechas con otro rueda a 0
        # wLeft = [1, 1, -1, 1, 2.3, 1, 2, 2, -2, 3]
        # wRight = [0, 2.5, 1, 1.5, -1.5, 0, 2, -1.5, -2.5, 0]
        #
        # # Empezando girando a izquierdas con otro rueda a 0
        # wLeft = [0, 1, -1, 1, 2.3, 1, 2, 2, -2, 3]
        # wRight = [1, 2.5, 1, 1.5, -1.5, 0, 2, -1.5, -2.5, 0]
        #
        # # Cambios de dirección
        # wLeft = [-1, 1, -1, 2.5]
        # wRight = [-1, 1, -1, 2.5]
        #
        # # Empezar girando sobre su eje de rotación izquierdas
        # wLeft = [-1, 1, -1, 1, 2.3, 1, 2, 2, -2, 3]
        # wRight = [1, 2.5, 1, 1.5, -1.5, 0, 2, -1.5, -2.5, 0]
        #
        # # Empezar girando sobre su eje de rotación derechas
        # wLeft = [1, 1, -1, 1, 2.3, 1, 2, 2, -2, 3]
        # wRight = [-1, 2.5, 1, 1.5, -1.5, 0, 2, -1.5, -2.5, 0]

        #Repetimos durante 10 ciclos la misma velocidad
        index = floor(contador/10)
        codigoLeft = self.velocidad2codigo(wLeft[index])
        codigoRight = self.velocidad2codigo(wRight[index])
        self.arduino.write(('V'+format(int(codigoRight),'03d')+
                           format(codigoLeft,'03d')).encode())
        return wRight[index], wLeft[index]
        
    def velocidad2codigo(self,w):
        sign = lambda x: int(copysign(1,x))
        dato = int(round(w * self.kdato))
        if abs(dato) > 127:
            
            dato = 127 * sign(dato)
        # No permitimos velocidades ingeriores a un código 10
        # porque no arrancan los motores
        # Solo en caso de que lo queramos parar
        if (abs(dato) < 10 and dato != 0):
            dato = 10 * sign(dato)
        #Negative velocity convert
        if dato<0:
            dato=256 + dato
        return dato
        
    def main(self):
        data = self.arduino.readline()
        print(data)
        contador = 0
        self.dum_first_read()
        last_time = time.time()*1000
        #Subscriber
        # old_settings = termios.tcgetattr(sys.stdin)
        try:
            # tty.setcbreak(sys.stdin.fileno())
            while(1):
                wRight, wLeft = self.enviar_velocidad(contador)
                contador += 1#Enviar velocidad
                if(contador >90):
                    contador = 0
                self.check()
                p = self.read_encoders()
                time.sleep((1/self.read_encoder_freq))
                if (contador%5 == 0):
                    print(time.time()*1000-last_time)
                    last_time = time.time()*1000
                    print(f'Velocidad Comando: R ({wRight:.2f}),\t L ({wLeft:.2f})')
                    print(f'Velocidad Lectura: R ({p[0]:.2f}),\t L ({p[1]:.2f})')
                    
                #Comprueba si se ha pulsado una tecla, en cuyo caso 
                #termina el test
                if self.isData():
                    c = sys.stdin.read(1)
                    if c == '\x1b':         # x1b is ESC
                        break
        finally:
            pass
            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# Function: check
#
# Comments
# -------------------
# Check if there has been no velocity cmd_wheel message in a defined time
# if so, stop the robot
# Parameters
# ----------
#
# Returns
# -------
    def check(self):
        time.sleep(0.3)


# Function: read_enc
#
# Comments
# ----------
# Read the encoders of the robot
# Calculate the possition and veocity of the wheels with the encoders data
# and publish in a JointState message
#
# Parameters
# ----------
#
# Returns
# -------
    def read_encoders(self):
        
        self.arduino.write(str('N').encode())
        #data = self.arduino.readline()
        # TODO: Se puede chequear que N y P son el comienzo y
        #final de 3la trama
        comienzo = self.arduino.readline().decode("utf-8")
        if (comienzo[0] != "N"):
            print("Error Rececpción"+time.ctime())
            return 1
        steps_enc_left = struct.unpack('i',self.arduino.read(4))[0]        
        time_enc_left = struct.unpack('i',self.arduino.read(4))[0]     
        steps_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        final = self.arduino.readline().decode("utf-8")
        if (final[0] != "P"):
            print("Error Rececpción"+time.ctime())  
            return 1
       #Increment calc
        dt_right = float((time_enc_right - self.time_enc_right_last) * (10**-6))
        dsteps_right = float(steps_enc_right - self.steps_enc_right_last)
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right
        dt_left = float((time_enc_left - self.time_enc_left_last) * (10**-6))
        dsteps_left = float(steps_enc_left - self.steps_enc_left_last)
        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left
        #Posicion
        posD = (steps_enc_right/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        wD = ((dsteps_right/self.pasos_vuelta) * 2 * pi) / dt_right
        posI = (steps_enc_left/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        wI = ((dsteps_left/self.pasos_vuelta) * 2 * pi) / dt_left
        
        return (wD, wI, posD, posI)
       


if __name__ == '__main__':
	foo = HwClass()
	foo.main()
