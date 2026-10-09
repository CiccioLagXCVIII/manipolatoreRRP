#!/usr/bin/env python3

# AA Importazioni Necessarie
import os
import sys
# BB Path Per Consentire L'Importazione Dei Moduli Dalla Cartella Scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import yaml
import rospkg
import numpy as np
import tf.transformations as tft
from sensor_msgs.msg import JointState

# AA Codici Colore ANSI Per Terminale
COLOR_WARN      = "\033[33m"        # Giallo Normale        (Avvisi)
COLOR_ERR       = "\033[31m"        # Rosso Normale         (Errori)
COLOR_RESET     = "\033[0m"         # Reset Stile           (Ripristina Colore Standard)

# AA Definizione Dei Parametri Fisici Del Robot Come Variabili Globali
# BB Siccome Poi Chiamo La Funzione loadRobotParameters() Per Caricare I Parametri Dal Server ROS
# BB Verranno Sovrascritti, Ma Comunque È Necessario Dichiararli Come Variabili Globali Per Evitare Errori
# CC Questi Valori Per I Parametri Fisici Del Robot (Che Poi Sono Quelli Veri) Sono Definiti Solo Per Sicurezza
# CC Se Si Modifica Il File Di Configurazione YAML Non Sarà Necessario Modificarle Manualmente
# BB Base
baseWidth = 0.3             # Larghezza Base
baseLength = 0.3            # Lunghezza Base
baseHeight = 0.05           # Altezza Base
worldBase = 2.0             # Distanza World Base

# BB Link
l1 = 0.55                   # Lunghezza Link 1
l2 = 0.45                   # Lunghezza Link 2
l3 = 0.35                   # Lunghezza Link 3
linkRadius = 0.035          # Raggio Link (Diametro 7cm)

# BB Giunti
jointRadius = 0.05          # Raggio Sfere Giunti Rotoidale (Diametro 10cm)
boxSize = 0.10              # Dimensione Box Giunto Prismatico (Lato 10cm)

# BB End Effector
eeBaseWidth = 0.05          # Larghezza Base Pinza 5 cm
eeBaseLength = 0.10         # Lunghezza Totale Base Pinza 10 cm
eeBaseHeight = 0.015        # Spessore Piastra Base Pinza 1.5cm
eeFingerLength = 0.05       # Lunghezza Dita Pinza 5cm
eeFingerThickness = 0.01    # Spessore Singolo Dito Pinza 1cm

# BB Limiti Giunti
# CC Siccome Nel File YAML Sono Definiti Numericamente (Senza NumPy), Anche Queste Variabili Globali Devono Essere Definite Allo Stesso Modo (Anche Avendo NumPy) Per Evitare Differenze Tra I Valori Nel File YAML E Quelli Iniziali Prima Di Eseguire loadRobotParameters
limitMinQ1 = -3.14159265359
limitMaxQ1 = 3.14159265359
limitMinQ2 = -1.57079632679
limitMaxQ2 = 0.78539816339
limitMinQ3 = 0.0
limitMaxQ3 = 0.35

# AA Funzione Che Carica I Parametri Fisici Del Robot Dal Server ROS E Li Rende Disponibili Come Variabili Globali
paramsLoaded = False  # Variabile Di Controllo Per Evitare Di Caricare I Parametri Più Volte
def loadRobotParameters(forceReload=False):
    global paramsLoaded
    global baseWidth, baseLength, baseHeight, worldBase
    global l1, l2, l3, linkRadius
    global jointRadius, boxSize
    global eeBaseWidth, eeBaseLength, eeBaseHeight, eeFingerLength, eeFingerThickness
    global limitMinQ1, limitMaxQ1, limitMinQ2, limitMaxQ2, limitMinQ3, limitMaxQ3

    if paramsLoaded and not forceReload:
        # I Parametri Sono Già Stati Caricati, Non È Necessario Ricaricarli
        return
    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    loadedFromROS = False
    if rospy.core.is_initialized():
        try:
            # CC Lettura Obbligatoria Dal Server ROS (Se Manca Anche Solo Un Parametro, Solleva KeyError E Legge Dal File YAML)
            # DD Quando Si Utilizza rospy.get_param(chiave, default) Passando Un Valore Di Default, La Funzione Non Genera Mai KeyError,
            # DD Neanche Se La Chiave Non Esiste Sul Server ROS E Restituisce Il Valore Di Default, Quindi Se Si Esegue Senza Parametri
            # DD Sul Server Anche Modificando Il File YAML Verranno Usati I Valori Di Default
            # EE Usando rospy.get_param(/chiave) ROS Guarda Solo Nel Namespace Globale, Quindi Se Si Esegue Il Nodo In Un Namespace Diverso Non Trova I Parametri
            # EE Usando rospy.get_param(~chiave) ROS Cerca Nel Namespace Del Nodo
            # EE Usando rospy.get_param(chiave) ROS Cerca Nel Namespace Attivo Del Nodo (Globale Se È Nel Root, Altrimenti Quello Del Gruppo), Quindi È Più Flessibile
            baseWidth = rospy.get_param("baseWidth")          # Larghezza Base
            baseLength = rospy.get_param("baseLength")        # Lunghezza Base
            baseHeight = rospy.get_param("baseHeight")        # Altezza Base
            worldBase = rospy.get_param("worldBase")          # Distanza World Base

            l1 = rospy.get_param("l1")
            l2 = rospy.get_param("l2")
            l3 = rospy.get_param("l3")
            linkRadius = rospy.get_param("linkRadius")

            jointRadius = rospy.get_param("jointRadius")
            boxSize = rospy.get_param("boxSize")

            eeBaseWidth = rospy.get_param("eeBaseWidth")
            eeBaseLength = rospy.get_param("eeBaseLength")
            eeBaseHeight = rospy.get_param("eeBaseHeight")
            eeFingerLength = rospy.get_param("eeFingerLength")
            eeFingerThickness = rospy.get_param("eeFingerThickness")

            limitMinQ1 = rospy.get_param("limitMinQ1")
            limitMaxQ1 = rospy.get_param("limitMaxQ1")
            limitMinQ2 = rospy.get_param("limitMinQ2")
            limitMaxQ2 = rospy.get_param("limitMaxQ2")
            limitMinQ3 = rospy.get_param("limitMinQ3")
            limitMaxQ3 = rospy.get_param("limitMaxQ3")

            loadedFromROS = True
        except (KeyError, rospy.ROSException):
            loadedFromROS = False
            print(f"{COLOR_WARN}[kinematicsUtils] Parametri Non Trovati Nel Server ROS, Caricamento Dal File YAML Di Configurazione...")

    # CC Se ROS Non Riesce A Raggiungere ROS O A Caricare I Parametri, Legge I Parametri Dal File YAML
    if not loadedFromROS:
        try:
            # DD Prova A Trovare Il Percorso Del Pacchetto ROS E Caricare Il File YAML Dei Parametri
            rospack = rospkg.RosPack()
            pkgPath = rospack.get_path('manipolatoreRRP')
            yamlPath = os.path.join(pkgPath, 'config', 'robotParameters.yaml')
        except Exception:
            # DD Se Non È Possibile Trovare Il Pacchetto ROS, Risale Dalla Posizione Dello Script
            scriptDir = os.path.dirname(os.path.abspath(__file__))
            yamlPath = os.path.abspath(os.path.join(scriptDir, '..', 'config', 'robotParameters.yaml'))

        with open(yamlPath, 'r') as f:
            params = yaml.safe_load(f)

        baseWidth = params['baseWidth']
        baseLength = params['baseLength']
        baseHeight = params['baseHeight']
        worldBase = params['worldBase']
        l1 = params['l1']
        l2 = params['l2']
        l3 = params['l3']
        linkRadius = params['linkRadius']
        jointRadius = params['jointRadius']
        boxSize = params['boxSize']
        eeBaseWidth = params['eeBaseWidth']
        eeBaseLength = params['eeBaseLength']
        eeBaseHeight = params['eeBaseHeight']
        eeFingerLength = params['eeFingerLength']
        eeFingerThickness = params['eeFingerThickness']
        limitMinQ1 = params["limitMinQ1"]
        limitMaxQ1 = params["limitMaxQ1"]
        limitMinQ2 = params["limitMinQ2"]
        limitMaxQ2 = params["limitMaxQ2"]
        limitMinQ3 = params["limitMinQ3"]
        limitMaxQ3 = params["limitMaxQ3"]

    paramsLoaded = True

# BB Limiti
# Giunto 1: Da -3.142 a 3.142 (360 gradi)
# Giunto 2: Da -1.571 a 0.785 (Non Serve Andare Oltre La Verticale Per Evitare Ridondanza)
# Giunto 3: Da 0 a L3 (Per Evitare Che La Pinza Entri Nel Box)

# AA Funzione Che Verifica Se I Valori Dei Giunti INSERITI Rientrano Nei Limiti
def checkSingleJointLimit(jointName, jointValue):
    loadRobotParameters()
    # BB Definizione Tolleranza Per Prevenire Imprecisioni Di Floating-Point Ai Confini Del Workspace (Dava Errore Quando Era Vicino Al Confine Del WS)
    tolerance = 1e-6

    # BB Definizione Dei Limiti Per Ogni Giunto Con Tolleranza
    # CC Giunto 1
    minQ1 = limitMinQ1 - tolerance
    maxQ1 = limitMaxQ1 + tolerance
    # CC Giunto 2
    minQ2 = limitMinQ2 - tolerance
    maxQ2 = limitMaxQ2 + tolerance
    # CC Giunto 3
    minQ3 = limitMinQ3 - tolerance
    maxQ3 = limitMaxQ3 + tolerance

    if jointName == "q1":
        if minQ1 <= jointValue <= maxQ1:
            # EE Per Valori Vicino Al Bordo Del WS Da Errore, Assegno A q1 Il Valore Limitato Tra I Limiti Fisici Del Giunto
            clippedQ1 = float(np.clip(jointValue, limitMinQ1, limitMaxQ1))
            return True, clippedQ1, ""
        elif jointValue < limitMinQ1:
            return False, jointValue, f"Valore Giunto 1 Inferiore Al Limite Minimo [{limitMinQ1:.3f}]"
        else:
            return False, jointValue, f"Valore Giunto 1 Superiore Al Limite Massimo [{limitMaxQ1:.3f}]"

    elif jointName == "q2":
        if minQ2 <= jointValue <= maxQ2:
            # EE Per Valori Vicino Al Bordo Del WS Da Errore, Assegno A q2 Il Valore Limitato Tra I Limiti Fisici Del Giunto
            clippedQ2 = float(np.clip(jointValue, limitMinQ2, limitMaxQ2))
            return True, clippedQ2, ""
        elif jointValue < limitMinQ2:
            return False, jointValue, f"Valore Giunto 2 Inferiore Al Limite Minimo [{limitMinQ2:.3f}]"
        else:
            return False, jointValue, f"Valore Giunto 2 Superiore Al Limite Massimo [{limitMaxQ2:.3f}]"

    elif jointName == "q3":
        if minQ3 <= jointValue <= maxQ3:
            # EE Per Valori Vicino Al Bordo Del WS Da Errore, Assegno A q3 Il Valore Limitato Tra I Limiti Fisici Del Giunto
            clippedQ3 = float(np.clip(jointValue, limitMinQ3, limitMaxQ3))
            return True, clippedQ3, ""
        elif jointValue < limitMinQ3:
            return False, jointValue, f"Valore Giunto 3 Inferiore Al Limite Minimo [{limitMinQ3:.3f}]"
        else:
            return False, jointValue, f"Valore Giunto 3 Superiore Al Limite Massimo [{limitMaxQ3:.3f}]"

    else:
        return False, jointValue, f"Giunto '{jointName}' Non Riconosciuto."

# AA Funzione Che Verifica Se I Valori Dei Giunti Superano I Limiti
def checkJointLimits(q1, q2, q3):
    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    loadRobotParameters()

    if q1 is None or q2 is None or q3 is None:
        return False, None, None, None

    try:
        q1, q2, q3 = float(q1), float(q2), float(q3)
    except (ValueError, TypeError):
        print(f"{COLOR_ERR}[kinematicsUtils] Ricevuti Valori Dei Giunti Non Numerici In checkJointLimits{COLOR_RESET}")
        return False, None, None, None

    # BB Controllo Del Giunto Uno Con Tolleranza Utilizzando checkSingleJointLimit
    # CC Verifica E Limita Il Primo Giunto Rotatorio Tra Meno Pi Greco E Piu Pi Greco
    validQ1, q1, errQ1 = checkSingleJointLimit("q1", q1)
    if not validQ1:
        print(f"{COLOR_ERR}[kinematicsUtils] {errQ1}")

    # BB Controllo Del Giunto Due Con Tolleranza Utilizzando checkSingleJointLimit
    # CC Verifica E Limita Il Secondo Giunto Rotatorio Tra Meno Pi Greco Mezzi E Pi Greco Quarti
    validQ2, q2, errQ2 = checkSingleJointLimit("q2", q2)
    if not validQ2:
        print(f"{COLOR_ERR}[kinematicsUtils] {errQ2}")

    # BB Controllo Del Giunto Tre Con Tolleranza Utilizzando checkSingleJointLimit
    # CC Verifica Limiti Giunto Prismatico Tra 0.0 (Massima Estensione) E La Lunghezza Del Braccio l3 (Massima Retrazione)
    # CC Tramite I Parametri Caricati Da File YAML
    validQ3, q3, errQ3 = checkSingleJointLimit("q3", q3)
    if not validQ3:
        print(f"{COLOR_ERR}[kinematicsUtils] {errQ3}")

    # BB Se Tutti E Tre I Valori Dei Giunti Sono Validi L'AND Restituisce True,
    # BB Altrimenti Anche Se Uno Solo Non È ValidO, L'AND Restituisce False
    isValid = validQ1 and validQ2 and validQ3

    return isValid, q1, q2, q3

# AA Funzione Che Genera Il Messaggio ROS Con I Nomi E Le Posizioni Validati In Formato JointState
def createJointStateMsg(q1, q2, q3):
    msg = JointState()

    # BB Timestamp Header Corrente
    msg.header.stamp = rospy.Time.now()

    # BB Definizione Nomi Dei Giunti Coerenti Con Il File URDF
    msg.name = ["giunto1", "giunto2", "giunto3"]

    # BB Definizione Posizioni Dei Giunti Nel Messaggio
    msg.position = [q1, q2, q3]

    # BB Definizione Velocità Dei Giunti Nel Messaggio
    msg.velocity = []

    # BB Definizione Sforzi Dei Giunti Nel Messaggio
    msg.effort = []

    return msg

# AA Funzione Che Calcola La Matrice Di Trasformazione Omogenea 4x4 A Partire Dal Vettore Di Traslazione E Dal Quaternione Di Rotazione
def getTransformationMatrix(translation, rotation):

    # BB Conversione Vettore Di Traslazione E Quaternione In Array NumPy
    translationVector = np.array([translation.x, translation.y, translation.z])
    quaternionVector = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

    # BB Calcolo Matrice Di Rotazione 3x3 A Partire Dal Quaternione
    quaternionMatrix = tft.quaternion_matrix(quaternionVector)
    rotationMatrix   = quaternionMatrix[:3, :3]

    # print("\nMatrice Di Rotazione 3x3:\n")
    # print(np.round(rotationMatrix, 4))

    # print("\nVettore Di Traslazione:\n")
    # print(np.round(translationVector, 4))

    # BB Calcolo Matrice Di Trasformazione Omogenea 4x4
    transformationMatrix = np.identity(4)
    # CC Matrice Di Rotazione Nelle Prime Tre Righe E Tre Colonne Colonne
    transformationMatrix[:3, :3] = rotationMatrix
    # CC Vettore Di Traslazione Nelle Prime Tre Righe Dell'Ultima Colonna
    transformationMatrix[:3, 3] = translationVector
    # CC L'Ultima Riga Resta [0, 0, 0, 1] Perché È Una Matrice Omogenea

    return transformationMatrix

# AA Funzione Che Controlla Se Il Target È Raggiungibile All'Interno Del Workspace Del Robot
def checkWorkspace(xTarget, yTarget, zTarget):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    loadRobotParameters()

    # CC Definizioni Valori Della Tabella Di Denavit Hartenberg
    d1 = 2*jointRadius + l1 + jointRadius               # Distanza Verticale Dal Frame 0 Al Frame 1
    a2 = jointRadius + l2 + (boxSize / 2.0)             # Distanza Verticale Dal Frame 1 Al Frame 2 (Lunghezza Link Orizzontale)
    joint2Center = baseHeight + d1                      # Coordinata Z Del Centro Del Giunto 2 Rispetto Alla Base Del Robot

    # CC Calcolo Dei Raggi Minimo E Massimo Del Workspace Del Robot Tramite I Limiti Del Giunto 3
    d3MaxDist = limitMinQ3 - (l3 + (boxSize / 2.0))     # Massima Estensione Verso Il Basso
    d3MinDist = limitMaxQ3 - (l3 + (boxSize / 2.0))     # Massima Estensione Verso L'Alto
    rDMax = np.sqrt(a2**2 + d3MaxDist**2)               # Raggio Sfera Esterna
    rDMin = np.sqrt(a2**2 + d3MinDist**2)               # Raggio Sfera Interna

    # BB Traslazione E Calcolo Delle Coordinate Relative Rispetto Al Giunto 2
    # CC Coordinate Relative Rispetto Al Centro Della Spalla (xRel, yRel, zRel)
    xRel = xTarget - worldBase
    yRel = yTarget
    zRel = zTarget - joint2Center

    # CC Distanza Radiale Dal Centro Del Giunto 2 Al Target
    rD = np.sqrt(xRel**2 + yRel**2 + zRel**2)

    # BB Verifiche Di Appartenenza Al Workspace

    # CC Tolleranza Per Prevenire Imprecisioni Di Floating-Point Ai Confini Del Workspace (Dava Errore Quando Era Vicino Al Confine Del WS)
    tolerance = 1e-6

    # CC Verifica 1: Distanza Sferica Dal Centro Del Giunto 2 Con Margine Di Tolleranza
    if rD < (rDMin - tolerance):
        print(f"{COLOR_WARN}[kinematicsUtils] Target Troppo Vicino Al Giunto 2 Del Robot: Distanza {rD:.4f} < {rDMin:.4f}")
        return False
    elif rD > (rDMax + tolerance):
        print(f"{COLOR_WARN}[kinematicsUtils] Target Fuori Dal Raggio Massimo: Distanza {rD:.4f} > {rDMax:.4f}")
        return False

    # CC Verifica 2: Raggio Minimo Nel Piano XY Con Tolleranza
    # DD Si Verifica Quando q2 = limitMinQ2 E q3 Alla Massima Retroazione
    rXY = np.sqrt(xRel**2 + yRel**2)
    d3Retracted = limitMaxQ3 - (l3 + (boxSize / 2.0))
    minRadiusXY = np.abs(d3Retracted)
    if rXY < (minRadiusXY - tolerance):
        print(f"{COLOR_WARN}[kinematicsUtils] Target Troppo Vicino Al Link Verticale: Raggio XY {rXY:.4f} < {minRadiusXY:.4f}")
        return False

    # CC Verifica 3: Limiti Di Escursione Angolare Della Spalla (Giunto 2)
    # DD Il Valore squareRootArg È Positivo Per Costruzione Poiché rD >= rDMin Con rDMin = sqrt(a2^2 + d3MinDist^2) Ovvero Circa 0.5523
    # DD Sapendo Che a2 = 0.5500 Si Ha Che rDMin > a2, E Poiché rD >= rDMin, Si Ha Che rD > a2, Quindi rD^2 - a2^2 > 0 Di Conseguenza,
    # DD squareRootArg È Sempre Positivo Per Qualsiasi Target Che Rientra Nel Workspace Del Robot
    squareRootArg = rD**2 - a2**2

    # EE Prendendo Il Massimo Tra 0 E squareRootArg Si Evita Che La Funzione np.sqrt() Generi Warning Per Valori Negativi (Che Possono Accadere Per Imprecisioni Di Floating-Point)
    squareRootArg = max(0.0, squareRootArg)

    # DD Il Valore d3 È Negativo Perché Il Giunto Prismatico Si Estende Lontano Dal Giunto 2, Quindi La Direzione Del Vettore Dal Giunto 2 Al Target Ha Componente Negativa Lungo l'Asse Z
    d3 = -np.sqrt(squareRootArg)

    # DD Calcolo Delle Configurazioni Per Il Giunto 2 (Frontale E All'Indietro)
    # CC Calcolo Valore q2 (Raggio Positivo) Quando Il Link 2 È Nella Direzione Del Target
    q2Front = np.arctan2(d3 * rXY - a2 * zRel, a2 * rXY + d3 * zRel)
    # CC Calcolo Valore q2 (Raggio Negativo) Quando Il Link 2 È Nella Direzione Opposta A Quella Del Target
    q2Back = np.arctan2(d3 * (-rXY) - a2 * zRel, a2 * (-rXY) + d3 * zRel)

    # DD Selezione Della Configurazione Del Giunto 2 Che Rispetta I Limiti Del Secondo Giunto
    # EE La Soluzione q2Front (Raggio Positivo) Si Verifica Quando Il Giunto 1 Orienta Il Link 2 Nella Direzione Del Target,
    # EE Mentre La Soluzione q2Back (Raggio Negativo) Si Verifica Quando Il Giunto 1 Orienta Il Link 2 Nella Direzione
    # EE Opposta A Quella Del Target E Quindi Il Giunto 2 Dovrebbe Andare Verso Valori Ancora Più Negativi (Oltre -3.14)
    # EE Scendendo Verso Il Basso E Raggiungere Il Punto "Da Sotto"
    # EE Nel Caso Di Questo Manipolatore, Poichè q2 È Limitato Tra [-pi/2, +pi/4], L'Unico Modo Fisicamente Possibile
    # EE Per Raggiungere Il Target È Fare In Modo Che Il Link 2 Punti Direttamente Verso Il Target (Orientando Il
    # EE Giunto 1 Verso Di Esso) Ovvero Usando La Soluzione q2Front.
    # EE Di Conseguenza La Soluzione q2Back Non Rientra Mai Nei Limiti Fisici, Ma La Controllo Per Completezza Algebrica

    # EE Controllo Che Almeno Una Delle Due Soluzioni Rientri Nei Limiti Del Giunto 2 Con Tolleranza (Dava Errore Quando Era Vicino Al Confine Del WS)
    minQ2 = limitMinQ2 - tolerance
    maxQ2 = limitMaxQ2 + tolerance

    q2Selected = None
    if minQ2 <= q2Front <= maxQ2:
        q2Selected = q2Front
    elif minQ2 <= q2Back <= maxQ2:
        q2Selected = q2Back
    else:
        print(f"{COLOR_WARN}[kinematicsUtils] Target Non Raggiungibile: Angolo q2 ({q2Front:.4f} rad) Fuori Dai Limiti [{limitMinQ2:.3f}, {limitMaxQ2:.3f}]")
        return False

    # CC Verifica 4: Limiti Di Escursione Angolare Della Base (Giunto 1)
    # DD Calcolo Del Segno Del Denominatore Con L'Arcotangente A Quattro Quadranti Come In inverseKinematics.py (Riga 117)
    denom = d3 * np.sin(q2Selected) + a2 * np.cos(q2Selected)

    if np.abs(denom) < 1e-6:
        # EE Target Sull'Asse Verticale: Singolarità Di Spalla
        rospy.logwarn("[inverseKinematics] Target Sull'Asse Verticale: q1 Indeterminato, Impostato a 0.0 rad")
        q1Target = 0.0
    else:
        if denom >= 0.0:
            signDenom = 1.0
        else:
            signDenom = -1.0

        q1Target = np.arctan2(yRel * signDenom, xRel * signDenom)

    minQ1 = limitMinQ1 - tolerance
    maxQ1 = limitMaxQ1 + tolerance
    if not (minQ1 <= q1Target <= maxQ1):
        print(f"{COLOR_WARN}[kinematicsUtils] Target Non Raggiungibile: Angolo q1 ({q1Target:.4f} rad) Fuori Dai Limiti [{limitMinQ1:.3f}, {limitMaxQ1:.3f}]")
        return False

    return True

# AA Funzione Che Verifica Le Singolarità Cinematiche Tramite Il Determinante Jacobiano
# BB La Matrice Jacobiana J Relaziona Le Velocità Dei Giunti Con Le Velocità Cartesiane:
# BB Ponendo:
# BB A = d3 sin(q2) + a2 cos(q2)
# BB B = -d3 cos(q2) - a2 sin(q2)
# BB [-A sin(q1),  B cos(q1), sin(q2)cos(q1)  ],
# BB [ A cos(q1),  B sin(q1), sin(q2) sin(q1) ],
# BB [0,           -A,        cos(q2)         ]
# BB Le Prime Due Righe Della Matrice Dipendono Da q1. Tuttavia, Per Identificare Le
# BB Singolarità Cinematiche, È Necessario Calcolare Il Determinante Della Matrice Jacobiana.
# BB La Matrice Jacobiana Può Essere Espressa Come J(q1, q2, q3) = Rz(q1) * J(q2, q3),
# BB Dove Rz(q1) È La Matrice Di Rotazione Attorno All'Asse z. Poiché Una Matrice Di
# BB Rotazione Ha Determinante Unitario E Il Determinante Di Un Prodotto Di Matrici
# BB È Uguale Al Prodotto Dei Loro Determinanti, Si Ottiene:
# BB det(J(q1, q2, q3)) = det(Rz(q1)) * det(J(q2, q3))
# BB                    = 1 * det(J(q2, q3))
# BB                    = det(J(q2, q3)).
# BB Di Conseguenza, Il Determinante Della Matrice Jacobiana Non Dipende Da q1 E Può
# BB Essere Calcolato Considerando Soltanto Le Variabili q2 E q3.
# CC Per Una Derivazione Approfondita E Motivata, Vedere Il File
# CC "6) cinematicaInversaProgetto.tex".

def checkSingularity(q1, q2, q3):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    loadRobotParameters()

    # CC Definizione Dei Parametri Della Tabella Di Denavit Hartenberg
    a2 = jointRadius + l2 + (boxSize / 2.0)
    d3 = q3 - (l3 + (boxSize / 2.0))

    # BB Calcolo Del Determinante Della Matrice Jacobiana
    # BB Calcolata Nel File "6) cinematicaInversaProgetto.tex" (Punti Di Singolarità)
    detJ = -d3 * (d3 * np.sin(q2) + a2 * np.cos(q2))

    # CC Analisi Delle Condizioni Di Singolarità Geometrica
    # DD Condizione 1: d3 = 0 (Giunto Prismatico Sull'Asse Del Giunto 2)
    firstCond = np.abs(d3)
    # DD Condizione 2: d3 * sin(q2) + a2 * cos(q2) = 0 (Raggio Nel Piano Orizzontale Nullo)
    secondCond = np.abs(d3 * np.sin(q2) + a2 * np.cos(q2))

    # BB Valutazione Dello Stato Di Singolarità Con Tolleranza Numerica
    tolerance = 1e-3
    singularityStatus = "Sicuro"

    # DD d3 = 0 È Fisicamente Irraggiungibile Per Finecorsa (d3 <= -boxSize / 2 = -0.05 m)
    if firstCond < tolerance:
        singularityStatus = "Singolarità Giunto Prismatico d3 Vicino A Zero"
        print(f"{COLOR_WARN}[kinematicsUtils] Attenzione: Robot Vicino A Singolarità Prismatica Con d3 = {d3:.4f} m (Giunto q1 = {q1:.4f} rad)")
    elif secondCond < tolerance:
        singularityStatus = "Singolarità Giunto 2 Con Raggio Vicino A Zero"
        print(f"{COLOR_WARN}[kinematicsUtils] Attenzione: Robot Vicino A Singolarità Di Spalla Con Raggio = {secondCond:.4f} m (Giunto q1 = {q1:.4f} rad)")

    return detJ, singularityStatus

# AA Blocco Di Test Per Verificare La Funzionalità Della Cinematica Diretta
if __name__ == "__main__":
    loadRobotParameters()
    print("[kinematicsUtils] [TEST PARAMETRI DEL MANIPOLATORE]")
    print(f"[kinematicsUtils] Dimensioni Base: {baseWidth}x{baseLength}x{baseHeight} m")
    print(f"[kinematicsUtils] Lunghezze Link: l1={l1} m, l2={l2} m, l3={l3} m")
    print(f"[kinematicsUtils] Limiti Giunto 1: [{limitMinQ1:.3f}, {limitMaxQ1:.3f}] rad")
    print(f"[kinematicsUtils] Limiti Giunto 2: [{limitMinQ2:.3f}, {limitMaxQ2:.3f}] rad")
    print(f"[kinematicsUtils] Limiti Giunto 3: [{limitMinQ3:.3f}, {limitMaxQ3:.3f}] m")
