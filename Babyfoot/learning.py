import serial
import time

ser = serial.Serial("COM5", timeout=1)

nb_axes = 3
nb_echantillons = 128

file = open("C:\\Users\\stage_cartesiam\\Documents\\Babyfoot\\learning_data.txt", "r")
contenu = file.read()
contenu = contenu.split()

nb_lignes = len(contenu)/(nb_axes*nb_echantillons)

for ligne in range(5):
	for i in range(nb_axes*nb_echantillons):
		for caract in contenu[ligne*nb_axes*nb_echantillons + i]:
			code = caract
			ser.write(code.encode('utf-8'))
		ser.write("\n".encode('utf-8'))
	time.sleep(1)
