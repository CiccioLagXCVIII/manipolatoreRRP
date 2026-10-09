#!/usr/bin/env python3

# AA Importazioni Necessarie
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import tf2_ros
import numpy as np
import kinematicsUtils

# AA Codici Colore ANSI Per Uniformare Lo Stile Con jointController
COLOR_TITOLO    = "\033[1;36m"      # Ciano Grassetto
COLOR_RISULTATO = "\033[1;32m"      # Verde Grassetto
COLOR_VALORE    = "\033[1;37m"      # Bianco Grassetto
COLOR_RESET     = "\033[0m"         # Reset Stile Standard

# AA Impostazioni Stampa Di NumPy
# BB Precisione, Notazione Scientifica, Larghezza Massima Di Stampa
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# AA Inizializzazione
rospy.init_node('tfPrinter')

# AA Definizione Frame Di Riferimento
TARGET_FRAME = 'world'
SOURCE_FRAME = 'endEffector'

# AA Creazione Buffer E Listener TF
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# AA Frequenza Di Aggiornamento
# BB L'Argomento Indica Il Numero Di Aggiornamenti Da Eseguire In Un Secondo
rate = rospy.Rate(1) # Frequenza 1 Hz

# AA Ciclo Principale Del Nodo ROS
try:
    while not rospy.is_shutdown():
        try:
            # BB Intercettazione Messaggio Di Trasformazione Tra Frame Source E Target
            # CC rospy.Time(0) Permette Di Ottenere L'Ultima Trasformazione Disponibile
            dataTF = tf_buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(0), rospy.Duration(1.0))

            # BB Estrazione Della Traslazione E Della Rotazione Dal Messaggio Di Trasformazione
            translation = dataTF.transform.translation
            rotation = dataTF.transform.rotation

            # BB Stampa Dati Grezzi
            print()
            print(f"{COLOR_TITOLO}[tfPrinter] -----------------------------------------------------{COLOR_RESET}")

            print(f"{COLOR_TITOLO}[tfPrinter] Trasformazione Da '{SOURCE_FRAME}' A '{TARGET_FRAME}':{COLOR_RESET}")

            print(f"{COLOR_TITOLO}[tfPrinter] Timestamp:{COLOR_RESET} {COLOR_VALORE}{dataTF.header.stamp.to_sec():.4f}{COLOR_RESET}")

            print(f"{COLOR_RISULTATO}[tfPrinter] Posizione End Effector (Traslazione X, Y, Z):{COLOR_RESET} {COLOR_VALORE} [ {translation.x:.4f}, {translation.y:.4f}, {translation.z:.4f} ]{COLOR_RESET}")

            print(f"{COLOR_RISULTATO}[tfPrinter] Quaternione (Orientamento X, Y, Z, W):{COLOR_RESET} {COLOR_VALORE} [ {rotation.x:.4f}, {rotation.y:.4f}, {rotation.z:.4f}, {rotation.w:.4f} ]{COLOR_RESET}")

            # BB Calcolo Della Matrice Di Trasformazione Omogenea
            transformationMatrix = kinematicsUtils.getTransformationMatrix(translation, rotation)

            # BB Stampa Della Matrice Di Trasformazione Omogenea
            print(f"\n{COLOR_RISULTATO}[tfPrinter] Matrice Trasformazione Omogenea {TARGET_FRAME} - {SOURCE_FRAME}:{COLOR_RESET}")
            print(F"{COLOR_VALORE}{np.round(transformationMatrix, 3)}{COLOR_RESET}")

        except tf2_ros.TransformException:
            # BB Avviso Quando La Trasformazione Non È Ancora Disponibile
            # CC rospy.logwarn_throttle(2.5, ...) Limita La Frequenza Di Stampa Dell'Avviso A Una Volta Ogni 2.5 Secondi
            rospy.logwarn_throttle(2.5, "[tfPrinter] In attesa Che La Trasformazione Sia Disponibile...")

        rate.sleep()

# BB Gestione Chiusura Pulita Senza Traceback Su Interruzione Utente O Chiusura Di ROS
except (rospy.ROSInterruptException, KeyboardInterrupt):
    pass
