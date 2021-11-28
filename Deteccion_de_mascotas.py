import cv2
import numpy as np
import time
import socket
import subprocess
import threading


TCP_IP = '192.168.100.15'
TCP_PORT_Py = 65432  # Escritura desde Python
TCP_PORT_NR = 65431  # Escritura desde Node-Red

# Envio de datos hacia node-red
s_Py = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_Py.connect((TCP_IP, TCP_PORT_Py))
#
# # Recepcion de datos desde node-red
#
s_NR = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_NR.connect((TCP_IP, TCP_PORT_NR))

modoManual = False
cmdPuerta = False
estadoMascota = ['Adentro'] * 2

class NodeRedRx(threading.Thread):
    def run(self):
        global cmdPuerta, s_NR, modoManual
        while True:
            data = str(s_NR.recv(512))
            if data.find('manual')!=-1:
                modoManual = True
            if data.find('auto')!=-1:
                modoManual = False
            if data.find('abrir') != -1:
                cmdPuerta = True
            if data.find('cerrar') != -1:
                cmdPuerta = False
            print(cmdPuerta)
            time.sleep(1)


class CmdMotor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.cmdPuertaPrev = False
        self.estadoPrev = [''] * 2
        self.firstCmd = True
        s_Py.send('Puerta Cerrada'.encode('utf-8'))
    def run(self):
        global cmdPuerta, s_Py, estadoMascota
        cmdCloseInterval = time.time()
        while True:
            if self.estadoPrev != estadoMascota:
                self.estadoPrev = estadoMascota[:]
                print(self.estadoPrev)
                cmdCloseInterval = time.time() if all(estado == 'Afuera' for estado in estadoMascota) else cmdCloseInterval
                if all(estado == 'Adentro' for estado in estadoMascota) and time.time() - cmdCloseInterval > 5:
                    cmdPuerta = False
                    #print('cierro yo')

            if cmdPuerta != self.cmdPuertaPrev and (self.firstCmd or time.time() - cmdInterval > 10):
                cmdInterval = time.time()
                if cmdPuerta:
                    subprocess.run(["/home/pi/433Utils/RPi_utils/codesend", "12"])
                    #print('abrir')
                    s_Py.send('Puerta Abierta'.encode('utf-8'))
                if not cmdPuerta:
                    subprocess.run(["/home/pi/433Utils/RPi_utils/codesend", "11"])
                    #print('cerrar')
                    s_Py.send('Puerta Cerrada'.encode('utf-8'))
                self.firstCmd = False
                self.cmdPuertaPrev = cmdPuerta
            time.sleep(1)

nodeRedRxInstance = NodeRedRx()
nodeRedRxInstance.start()

cmdMotorInstance = CmdMotor()
cmdMotorInstance.start()

salida = cv2.VideoWriter('pruebaultimaopcion.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, (640, 480))#640 48

# Load Yolo
#net = cv2.dnn.readNet("../YoloV4_Weigths/yolov4-tiny-custom_6000.weights", "../YoloV4_Weigths/yolov4-tiny-custom.cfg")
net = cv2.dnn.readNet("train2411last.weights", "yolov4-tiny-custom.cfg")
classes = []
with open("obj.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading camera
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN
frame_id = 0
starting_time = time.time()
started = [False] * 4
initTime = [0] * 4
elapsedTime = [0] * 4
contarByClass = [False] * 2
objetoAdentro = [False] * 2
objetoAfuera = [False] * 2
returnBreak = False
xyI = [0, 100, 550, 260]  # Coordenadas Caja Adentro
xyO = [0, 261, 550, 450]  # Coordenadas Caja Afuera


def decisionPos(arrBox, arrObject):
    """
    :rtype: bool
    :param arrBox: Coordenadas de caja | arrBox=[x1, y1, x2, y2]
    :param arrObject: Coordenadas de objeto | arrObject=[x, y]
    :return: True si Objeto dentro de caja, False si fuera de caja
    """
    return True if arrBox[0] < arrObject[0] < arrBox[2] and arrBox[1] < arrObject[1] < arrBox[3] else False


def cuadrosAdentroAfuera(frame):
    cv2.rectangle(frame, (xyI[0], xyI[1]), (xyI[2], xyI[3]), (0, 255, 0), 3)
    cv2.putText(frame, "Adentro", (xyI[0] + 10, xyI[1] + 30), font, 2, (0, 240, 0), 2)
    cv2.rectangle(frame, (xyO[0], xyO[1]), (xyO[2], xyO[3]), (0, 0, 255), 3)
    cv2.putText(frame, "Afuera", (xyO[0] + 10, xyO[1] + 30), font, 2, (0, 0, 240), 2)


def temporizadorObjetos(clase, frame):
    global cmdPuerta, initTime, started, elapsedTime
    if clase == 'luna':
        if not started[0]:
            initTime[0] = time.time()
            started[0] = True
        elapsedTime[0] = time.time() - initTime[0]
        elapsedTime[1] = 0
        started[1] = False
        #cv2.putText(frame, ("Luna Seg:" + str(round(elapsedTime[0], 2))), (320, 90), font, 2,
         #           (10, 50, 210), 3)
    if clase == 'ciro':
        if not started[2]:
            initTime[2] = time.time()
            started[2] = True
        elapsedTime[2] = time.time() - initTime[2]
        elapsedTime[3] = 0
        started[3] = False
        #cv2.putText(frame, ("Ciro Seg:" + str(round(elapsedTime[2], 2))), (320, 70), font, 2,
         #           (10, 50, 210), 3)
    if elapsedTime[0] > 3:
        if clase == "luna":
            luna_detected = 1
            ciro_detected = 0
        cmdPuerta = True
    if elapsedTime[2] > 3:
        if clase == 'ciro':
            luna_detected = 0
            ciro_detected = 1
        cmdPuerta = True


def resetTemporizadorObjetos(clase, confidence):
    global cmdPuerta, initTime, started, elapsedTime
    if clase == 'luna' and started[0] and confidence < 0.7:
        if not started[1]:
            initTime[1] = time.time()
            started[1] = True
        elapsedTime[1] = time.time() - initTime[1]
        if elapsedTime[1] > 2:
            started[0] = False
            elapsedTime[1] = 0
            elapsedTime[0] = 0
            # cmdPuerta = False
    if clase == 'ciro' and started[2] and confidence < 0.7:
        if not started[3]:
            initTime[3] = time.time()
            started[3] = True
        elapsedTime[3] = time.time() - initTime[3]
        if elapsedTime[3] > 2:
            started[2] = False
            elapsedTime[3] = 0
            elapsedTime[2] = 0
            # cmdPuerta = False


def estadoMascotaMethod(objetoAdentro, objetoAfuera):
    for idx, mascotaAdentro in enumerate(objetoAdentro): estadoMascota[idx] = 'Adentro' if mascotaAdentro else \
        estadoMascota[idx]
    for idx, mascotaAfuera in enumerate(objetoAfuera): estadoMascota[idx] = 'Afuera' if mascotaAfuera else \
        estadoMascota[idx]

def deteccion():
    global frame_id, empieza_timer, frame_id2
    #frame_id2=0
    _, frame = cap.read()
    frame_id += 1    
    if frame is None:
        return True
    if frame_id%10 != 0:
        return False
    frame_id2 += 1
    print(frame_id2)
    height, width, channels = frame.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (224, 224), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            resetTemporizadorObjetos(classes[class_id], confidence)
            if confidence > 0.6:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                contarByClass[class_id] = decisionPos(xyI, [center_x, center_y])
                objetoAdentro[class_id] = decisionPos(xyI, [x + w, y + h])
                objetoAfuera[class_id] = decisionPos(xyO, [x + w, y + h])
                estadoMascotaMethod(objetoAdentro, objetoAfuera)
                #cv2.putText(frame, (classes[class_id] + ": Contando") if contarByClass[class_id] else "",
                #            (xyI[0] + 300, xyI[1] + (60 if class_id else 90)), font, 2, (0, 240, 0), 2)
                #cv2.putText(frame, (classes[class_id] + ": Adentro") if objetoAdentro[class_id] else "",
                 #           (xyI[0] + 10, xyI[1] + (60 if class_id else 90)), font, 2, (0, 240, 0), 2)
                #cv2.putText(frame, (classes[class_id] + ": Afuera") if objetoAfuera[class_id] else "",
                 #           (xyO[0] + 10, xyO[1] + (60 if class_id else 90)), font, 2, (0, 0, 240), 2)
                if frame_id > 2 and contarByClass[class_id]:
                    temporizadorObjetos(classes[class_id], frame)
    #if cmdPuerta:
    #    cv2.putText(frame, "Abriendo", (150, 300), font, 6, (255, 0, 0), 3)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = colors[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.rectangle(frame, (x, y), (x + w, y + 30), color, -1)
            cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 3, (255, 255, 255), 2)
    # Cuadros 'Adentro' 'Afuera'
    cuadrosAdentroAfuera(frame)
    salida.write(frame)
    termina_timer=time.time()-empieza_timer
    elapsed_time = time.time() - starting_time
    fps = frame_id2 / termina_timer
    #cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), font, 3, (0, 0, 0), 3)
    cv2.imshow("Image", frame)
    key = cv2.waitKey(1)
    if key == 27:
        return True
    else:
        return False
    #return True if cv2.waitKey(1) & 0xFF == ord('s') else False


while not returnBreak:
    if not modoManual:
        frame_id2=0
        empieza_timer = time.time()
        returnBreak = deteccion()
cap.release()
salida.release()
cv2.destroyAllWindows()
s_NR.close()
s_Py.close()
