#!/usr/bin/env python3

# AA Importazioni Necessarie
import os
import sys
# BB Path Per Consentire L'Importazione Dei Moduli Dalla Cartella Scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import numpy as np
import kinematicsUtils

# AA Funzione Che Calcola Le Matrici Di Trasformazione Necessarie Per Calcolare La Matrice Di Trasformazione
def calculateTransformationMatrix(theta, d, a, alpha):
    # BB $R_z(\theta_i)$: Rotazione Attorno A $Z_{i-1}$ Di Angolo $\theta_i$ (*Variabile Di Giunto*)
    cosTheta = np.cos(theta)
    sinTheta = np.sin(theta)
    # Rz_theta = np.array([
    #     [cosTheta, -sinTheta, 0.0, 0.0],
    #     [sinTheta,  cosTheta, 0.0, 0.0],
    #     [0.0, 0.0, 1.0, 0.0],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])
    # BB $T_z(d_i)$: Traslazione Lungo $Z_{i-1}$ Di Distanza $d_i$ (Variabile Di Giunto Se Prismatico, Costante Se Rotoidale), Ovvero Finché $Z_{i-1}$ E $Z_i$ Sono Complanari.
    # Tz_d = np.array([
    #     [1.0, 0.0, 0.0, 0.0],
    #     [0.0, 1.0, 0.0, 0.0],
    #     [0.0, 0.0, 1.0, d],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])
    # BB $T_x(a_i)$: Traslazione Lungo $X_i$ Di Distanza $a_i$ (Costante), Ovvero Finché Le Origini Dei Due Frame Coincidono.
    # Tx_a = np.array([
    #     [1.0, 0.0, 0.0, a],
    #     [0.0, 1.0, 0.0, 0.0],
    #     [0.0, 0.0, 1.0, 0.0],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])
    # BB $R_x(\alpha_i)$: Rotazione Attorno A $X_i$ Di Angolo $\alpha_i$ Finché $Z_{i-1}$ Si Allinea A $Z_i$.
    cosAlpha = np.cos(alpha)
    sinAlpha = np.sin(alpha)
    # Rx_alpha = np.array([
    #    [1.0, 0.0, 0.0, 0.0],
    #    [0.0, cosAlpha, -sinAlpha, 0.0],
    #    [0.0, sinAlpha,  cosAlpha, 0.0],
    #    [0.0, 0.0, 0.0, 1.0]
    # ])

    # DD Combinando Tutte Le Matrici In Un'unica Matrice Di Trasformazione Omogenea, Si Ottiene
    # DD La Matrice Di Trasformazione Dal Frame i-1 Al Frame i, Ovvero $T_{i-1}^{i} = R_z(\theta_i) T_z(d_i) T_x(a_i) R_x(\alpha_i)$
    T = np.array([
        [cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta],
        [sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta],
        [0.0,       sinAlpha,             cosAlpha,            d],
        [0.0,       0.0,                  0.0,                 1.0]
    ])

    return T

# AA Funzione Che Restituisce La Matrice Di Trasformazione Omogenea
def computeTransformMatrix(q1, q2, q3):
    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    kinematicsUtils.loadRobotParameters()
    l1 = kinematicsUtils.l1
    l2 = kinematicsUtils.l2
    l3 = kinematicsUtils.l3
    jointRadius = kinematicsUtils.jointRadius
    boxSize = kinematicsUtils.boxSize

    # BB Definizione Tabella DH Per Il Robot RRP
    # CC Dizionario Chiave-Valore Dove La Chiave Rappresenta Il Numero Del Link E Il Valore È La Lista Dei Parametri DH [Theta, D, A, Alpha]
    d1 = 2*jointRadius + l1 + jointRadius       # Distanza Da RF0 Al RF1
    a2 = jointRadius + l2 + (boxSize/2)         # Distanza Da RF1 Al RF2
    d3 = q3 - (l3 + boxSize/2)                  # Distanza Da RF2 Al RF End Effector (A Riposo Vale -(l3 + boxSize/2) Ma Se q3 > 0 L'EE Sale Verso RF2)
    # [theta, d, a, alpha]
    tableDH = {
        1: [q1, d1, 0.0, -np.pi / 2.0],  # Da RF0 A RF1
        2: [q2, 0.0, a2, np.pi / 2.0],   # Da RF1 A RF2
        3: [0.0, d3, 0.0, -np.pi]        # Da RF2 A End Effector
    }
    # L'EE Si Trova Alla Fine Del Link RF2 Ma La Distanza Effettiva Dal Frame RF2 Dipende Sia Da q3 Che Dal Box
    # (A Riposo Vale -(l3 + boxSize/2) Infatto L'EE È Verso Il Basso).
    # Il Suo RF È Ruotato Ruotato Di 180° Attorno All'Asse X Per Orientare La Pinza Verso Il Basso

    # BB Calcolo Singole Matrici Di Trasformazione
    # CC L'Operatore * Permette Di Passare Gli Elementi Della Lista Come Argomenti Separati Alla Funzione
    # DD Facendo Una Lista Di Liste E Iterando Su Di Essa, Si Può Calcolare La Matrice Di Trasformazione
    # DD Di Ogni Lista Sempre Con * E Non È Necessario Definire Le Varibili Intermedie Ma Tanto Per Soli
    # DD Tre Link Non Cambia Niente In Termini Di Costo Computazionale
    T01 = calculateTransformationMatrix(*tableDH[1])
    T12 = calculateTransformationMatrix(*tableDH[2])
    T23 = calculateTransformationMatrix(*tableDH[3])

    # BB Calcolo Della Matrice Di Trasformazione Complessiva Dal Frame 0 Al Frame 3 (End Effector)
    T03 = T01 @ T12 @ T23

    return T03

# AA Funzione Che Calcola La Matrice Di Trasformazione Omogenea T03 E La Posizione Dell'End Effector
def computeDirectKinematics(q1, q2, q3):
    # BB Calcolo Della Cinematica Diretta Dal Frame Zero Al Frame Tre
    # CC La Funzione All'Interno Del Modulo Restituisce La Matrice Quattro Per Quattro T03
    T03 = computeTransformMatrix(q1, q2, q3)

    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    # CC La Chiamata A loadRobotParameters() Viene Effettuata All'Interno Di computeTransformMatrix() Ma La Ripeto Per Evitare Errore Di Dipendenza Tra Funzioni
    kinematicsUtils.loadRobotParameters()
    baseHeight = kinematicsUtils.baseHeight
    worldBase = kinematicsUtils.worldBase

    # BB Creazione Della Matrice Di Trasformazione Omogenea Da RF0 A RF Base (Posa Di RF0 Rispetto Al Frame Base)
    TBase0 = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, baseHeight],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # BB Creazione Della Matrice Di Trasformazione Omogenea Da RF Base A RF World (Posa Del Frame Base Rispetto A World)
    TWorldBase = np.array([
        [1.0, 0.0, 0.0, worldBase],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # BB Calcolo Della Matrice Da World A End Effector
    TWorldEE = TWorldBase @ TBase0 @ T03

    # BB Estrazione Posizione X, Y, Z Dalla Matrice Di Trasformazione
    # CC (Primi Tre Elementi Della Quarta Colonna)
    positionWorld = TWorldEE[:3, 3]

    return TWorldEE, positionWorld

# AA Blocco Di Test Per Verificare La Funzionalità Della Cinematica Diretta
if __name__ == "__main__":
    rospy.init_node("testDirectKinematics", anonymous=True)
    print("[directKinematics] [TEST CINEMATICA DIRETTA]")
    q1Test, q2Test, q3Test = 0.0, 0.0, 0.0
    T, pos = computeDirectKinematics(q1Test, q2Test, q3Test)
    print(f"[directKinematics] Configurazione Giunti Di Test: q1={q1Test}, q2={q2Test}, q3={q3Test}")
    print("[directKinematics] Matrice T_world_EE Calcolata:")
    print(np.round(T, 3))
    print(f"[directKinematics] Posizione Cartesiana World: {np.round(pos, 4)}")
