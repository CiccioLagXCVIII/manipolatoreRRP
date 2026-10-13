#!/usr/bin/env python3

# AA Importazioni Necessarie
import os
import signal
import sys
import select
# BB Libreria Sympy Per Gestire Input Simbolici
import sympy as sp

# BB Inserimento Del Percorso Corrente Nel System Path Per Consentire L'Importazione Dei Moduli Dalla Cartella Scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# AA Codici Colore ANSI Per Terminale
COLOR_TITOLO    = "\033[1;36m"      # Ciano Grassetto       (Titoli Principali)
COLOR_DIRETTA   = "\033[1;32m"      # Verde Grassetto       (Opzione Diretta)
COLOR_INVERSA   = "\033[1;33m"      # Giallo Grassetto      (Opzione Inversa)
COLOR_USCITA    = "\033[1;31m"      # Rosso Grassetto       (Opzione Esci)
COLOR_RISULTATO = "\033[1;32m"      # Verde Grassetto       (Titoli Risultati)
COLOR_DATO      = "\033[1;35m"      # Magenta Grassetto     (Dati Input)
COLOR_VALORE    = "\033[1;37m"      # Bianco Grassetto      (Valori Numerici, Vettori E Matrici)
COLOR_WARN      = "\033[33m"        # Giallo Normale        (Avvisi)
COLOR_ERR       = "\033[31m"        # Rosso Normale         (Errori)
COLOR_RESET     = "\033[0m"         # Reset Stile           (Ripristina Colore Standard)

# AA Importazione Delle Librerie ROS E Di Calcolo Numerico
import rospy
import numpy as np
from sensor_msgs.msg import JointState

# AA Importazione Dei Moduli Cinematici E Utilità Del Progetto
import kinematicsUtils
from directKinematics import computeDirectKinematics
from inverseKinematics import computeInverseKinematics


# AA Configurazione Delle Impostazioni Di Stampa Per La Libreria NumPy
# BB Precisione Decimale, Soppressione Notazione Scientifica E Larghezza Massima Riga
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# AA Inizializzazione Del Nodo ROS
rospy.init_node("jointController")

# AA Configurazione Del Publisher Per La Pubblicazione Dello Stato Dei Giunti (Topic Relativo Standard)
publisher = rospy.Publisher("joint_states", JointState, queue_size=10)

# AA Definizione Delle Variabili Dei Giunti Iniziali A Riposo
# q1 = 0.0
# q2 = 0.0
# q3 = 0.0

# AA Posizione Cartesiana Dell'End Effector Di Default A Riposo
# x = 2.55
# y = 0.0
# z = 0.35

# AA Definizione Delle Variabili Di Stato Corrente Dei Giunti
currentQ1 = 0.0
currentQ2 = 0.0
currentQ3 = 0.0

# AA Definizione Delle Variabili Di Posa Obiettivo (Target) Dei Giunti
targetQ1 = 0.0
targetQ2 = 0.0
targetQ3 = 0.0

# AA Callback Periodica Per L'Aggiornamento E La Pubblicazione Dello Stato Dei Giunti
# BB Esecuzione A 20 Hz Tramite Timer Per Garantire Il Flusso Continuo Verso robot_state_publisher E RViz
def publishJointStatesCallback(event):
    global currentQ1, currentQ2, currentQ3

    # CC Interpolazione Lineare Con Soglia Dei Giunti Per Evitare Salti Bruschi In RViz
    # DD Il Nuovo Valore Corrente È Dato Dall'Ultimo Valore Pubblicato Più Un Piccolo
    # DD Avanzamento Verso Il Target, Ottenuto Prendendo Il 15% Della Distanza Rimanente

    alpha = 0.15
    currentQ1 += alpha * (targetQ1 - currentQ1)
    currentQ2 += alpha * (targetQ2 - currentQ2)
    currentQ3 += alpha * (targetQ3 - currentQ3)

    # CC Tentativo Di Generazione E Pubblicazione Del Messaggio JointState Sul Topic Dedicato
    try:
        msg = kinematicsUtils.createJointStateMsg(currentQ1, currentQ2, currentQ3)
        publisher.publish(msg)
    except rospy.ROSException:
        pass

rospy.Timer(rospy.Duration(0.05), publishJointStatesCallback)

# AA Funzione Di Input Non Bloccante Compatibile Con Lo Shutdown Di ROS
# BB Monitoraggio Dello Standard Input Periodico Tramite Select
def rosInput(prompt=""):
    # CC Stampa Del Messaggio Senza Andare A Capo Con Svuotamento Del Buffer
    print(prompt, end="", flush=True)

    while not rospy.is_shutdown():
        try:
            # DD Interrogazione Dello stdin Con Timeout Di 0.1 Secondi Per Rilasciare Il Thread
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        except (select.error, InterruptedError):
            raise KeyboardInterrupt

        if rospy.is_shutdown():
            raise KeyboardInterrupt

        # DD Se Ci Sono Caratteri Pronti Nel Buffer Di Ingresso Vengono Acquisiti
        if rlist:
            line = sys.stdin.readline()
            # EE Se La Riga E Vuota Significa Che E Stato Inviato Il Segnale Di Fine (EOF)
            if not line:
                raise EOFError
            # EE Restituisce La Stringa Letta Rimuovendo Gli Spazi E I Caratteri Di Ritorno A Capo
            return line.strip()

    # CC Se Il Nodo ROS Riceve Un Segnale Di Arresto Solleva L'Eccezione Per Uscire
    raise KeyboardInterrupt

# AA Funzione Per La Conversione Flessibile Di Espressioni Matematiche Con Pi Greco
def parseAngleInput(inputStr):
    # BB Pulizia Della Stringa In Ingresso
    cleanedInput = inputStr.strip().lower().replace(" ", "").replace("np.pi", "pi")
    if not cleanedInput:
        raise ValueError("Stringa Vuota")

    if cleanedInput.startswith("+"):
        cleanedInput = cleanedInput[1:]

    # BB Parsing Simbolico Tramite SymPy
    try:
        parsedExpr = sp.sympify(cleanedInput, locals={"pi": sp.pi})
        numericValue = float(parsedExpr.evalf())
        return numericValue
    except (sp.SympifyError, TypeError, ValueError, AttributeError) as parseError:
        raise ValueError(f"Espressione Non Valida: '{inputStr}'") from parseError

# AA Ciclo Principale Di Input Utente Da Terminale
try:
    while not rospy.is_shutdown():
        # BB Formattazione Grafica Del Menu Delle Modalità Con Codici Colore ANSI
        choiceMenu = (
            f"\n{COLOR_TITOLO}[jointController] Inserisci Il Tipo Di Cinematica:{COLOR_RESET}\n"
            f"\t{COLOR_DIRETTA}[1] Diretta{COLOR_RESET}  (Inserisci \"1\" Oppure \"Diretta\")\n"
            f"\t{COLOR_INVERSA}[2] Inversa{COLOR_RESET}  (Inserisci \"2\" Oppure \"Inversa\")\n"
            f"\t{COLOR_USCITA}[3] Esci{COLOR_RESET}     (Inserisci \"3\" Oppure \"Esci\")\n"
            f"[jointController] Seleziona Opzione: "
        )

        # BB Lettura Input Utente
        rawChoice = rosInput(choiceMenu).strip().lower()

        # BB Mappatura Delle Scorciatoie Alla Modalità Corrispondente
        if rawChoice in ("1", "d", "diretta"):
            kinematicType = "diretta"
        elif rawChoice in ("2", "i", "inversa"):
            kinematicType = "inversa"
        elif rawChoice in ("3", "e", "esci"):
            kinematicType = "esci"
        else:
            kinematicType = rawChoice

        if not kinematicType:
            print(f"{COLOR_WARN}[jointController] Nessun Input Rilevato. Riprovare.{COLOR_RESET}")
            continue

        # BB ---------- Modalità Di Cinematica Diretta ----------
        if kinematicType == "diretta":
            print(f"\n{COLOR_TITOLO}[jointController] Modalità Cinematica Diretta{COLOR_RESET}")

            # CC Acquisizione E Validazione Giunto 1
            while not rospy.is_shutdown():
                try:
                    inputQ1 = parseAngleInput(rosInput("Inserisci Il Valore Per Il Giunto 1 (q1 In Radianti): "))
                except ValueError:
                    print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Numero O Costanti Come pi, Oppure -pi/4{COLOR_RESET}")
                    continue

                isValid, clippedQ1, errMsg = kinematicsUtils.checkSingleJointLimit("q1", inputQ1)
                if isValid:
                    inputQ1 = clippedQ1
                    break
                rospy.logwarn(f"[jointController] {errMsg}. Riprovare.")

            # CC Acquisizione E Validazione Giunto 2
            while not rospy.is_shutdown():
                try:
                    inputQ2 = parseAngleInput(rosInput("Inserisci Il Valore Per Il Giunto 2 (q2 In Radianti): "))
                except ValueError:
                    print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Numero O Costanti Come pi, Oppure -pi/4{COLOR_RESET}")
                    continue

                isValid, clippedQ2, errMsg = kinematicsUtils.checkSingleJointLimit("q2", inputQ2)
                if isValid:
                    inputQ2 = clippedQ2
                    break
                rospy.logwarn(f"[jointController] {errMsg}. Riprovare.")

            # CC Acquisizione E Validazione Giunto 3
            while not rospy.is_shutdown():
                try:
                    inputQ3 = float(rosInput("Inserisci Il Valore Per Il Giunto 3 (q3 In Metri):    "))
                except ValueError:
                    print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Numero In Metri{COLOR_RESET}")
                    continue

                isValid, clippedQ3, errMsg = kinematicsUtils.checkSingleJointLimit("q3", inputQ3)
                if isValid:
                    inputQ3 = clippedQ3
                    break
                rospy.logwarn(f"[jointController] {errMsg}. Riprovare.")

            # CC Calcolo E Stampa Della Posa Dell'End Effector
            tWorldEE, positionWorld = computeDirectKinematics(inputQ1, inputQ2, inputQ3)
            # CC Valutazione Dell Indice Di Singolarita Sulla Configurazione Raggiunta
            detJ, statusSing = kinematicsUtils.checkSingularity(inputQ1, inputQ2, inputQ3)

            # DD Stampa Delle Informazioni
            #print()
            print(f"{COLOR_TITOLO}[jointController] Soluzione Cinematica Diretta:{COLOR_RESET}")
            # EE Dati (Valori Giunti)
            print(f"{COLOR_DATO}Giunto 1 (q1):{COLOR_RESET} {COLOR_VALORE}{inputQ1:.4f} Rad{COLOR_RESET}")
            print(f"{COLOR_DATO}Giunto 2 (q2):{COLOR_RESET} {COLOR_VALORE}{inputQ2:.4f} Rad{COLOR_RESET}")
            print(f"{COLOR_DATO}Giunto 3 (q3):{COLOR_RESET} {COLOR_VALORE}{inputQ3:.4f} Metri{COLOR_RESET}")
            # EE Risultati (Posizione, Matrice Trasformazione, Indice Singolarità)
            print(f"{COLOR_RISULTATO}Matrice Trasformazione Omogenea world - endEffector:{COLOR_RESET}")
            print(f"{COLOR_VALORE}{np.round(tWorldEE, 3)}{COLOR_RESET}")
            print(f"{COLOR_RISULTATO}Posizione End Effector In World (X, Y, Z):{COLOR_RESET} {COLOR_VALORE}{np.round(positionWorld, 4)}{COLOR_RESET}")
            print(f"{COLOR_RISULTATO}Determinante Jacobiano:{COLOR_RESET} {COLOR_VALORE}{detJ:.6f} ({statusSing}){COLOR_RESET}")

            # CC Aggiornamento Dello Stato Globale Per RViz
            targetQ1, targetQ2, targetQ3 = inputQ1, inputQ2, inputQ3

        # BB ---------- Modalità Di Cinematica Inversa ----------
        elif kinematicType == "inversa":
            print(f"\n{COLOR_TITOLO}[jointController] Modalità Cinematica Inversa{COLOR_RESET}")

            targetValid = False
            while not rospy.is_shutdown() and not targetValid:
                # CC Acquisizione E Validazione Coordinata X
                while not rospy.is_shutdown():
                    try:
                        targetX = float(rosInput("Inserisci La Coordinata Target X (In Metri): "))
                        break
                    except ValueError:
                        print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Valore Numerico{COLOR_RESET}")

                # CC Acquisizione E Validazione Coordinata Y
                while not rospy.is_shutdown():
                    try:
                        targetY = float(rosInput("Inserisci La Coordinata Target Y (In Metri): "))
                        break
                    except ValueError:
                        print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Valore Numerico{COLOR_RESET}")

                # CC Acquisizione E Validazione Coordinata Z
                while not rospy.is_shutdown():
                    try:
                        targetZ = float(rosInput("Inserisci La Coordinata Target Z (In Metri): "))
                        break
                    except ValueError:
                        print(f"{COLOR_ERR}[jointController] Inserimento Non Valido. Digitare Un Valore Numerico{COLOR_RESET}")

                # CC Controllo Della Raggiungibilita Dello Spazio Di Lavoro
                if kinematicsUtils.checkWorkspace(targetX, targetY, targetZ):
                    targetValid = True
                else:
                    print(f"{COLOR_INVERSA}[jointController] Target Non Valido. Riprovare Ad Inserire Le Coordinate.{COLOR_RESET}")

            # CC Esecuzione Dei Calcoli Per Risolvere La Cinematica Inversa
            solQ1, solQ2, solQ3 = computeInverseKinematics(targetX, targetY, targetZ)

            if (solQ1 is not None) and (solQ2 is not None) and (solQ3 is not None):
                # DD Aggiornamento Delle Variabili Locali Dei Giunti Con I Nuovi Valori
                q1, q2, q3 = solQ1, solQ2, solQ3
                # DD Calcolo Della Posa Raggiunta Per Verifica
                tWorldEE, positionWorld = computeDirectKinematics(q1, q2, q3)
                # DD Valutazione Dell Indice Di Singolarita Sulla Configurazione Raggiunta
                detJ, statusSing = kinematicsUtils.checkSingularity(q1, q2, q3)
                # DD Stampa Delle Informazioni
                print(f"\n{COLOR_TITOLO}[jointController] Soluzione Cinematica Inversa:{COLOR_RESET}")
                # EE Dati (Posizione)
                print(f"{COLOR_DATO}Posizione Target End Effector In World (X, Y, Z):{COLOR_RESET} {COLOR_VALORE}[ {targetX:.4f}, {targetY:.4f}, {targetZ:.4f} ]{COLOR_RESET}")
                # EE Risultati (Valori Giunti, Matrice Trasformazione, Indice Singolarità)
                print(f"{COLOR_RISULTATO}Giunto 1 (q1):{COLOR_RESET} {COLOR_VALORE}{q1:.4f} Rad{COLOR_RESET}")
                print(f"{COLOR_RISULTATO}Giunto 2 (q2):{COLOR_RESET} {COLOR_VALORE}{q2:.4f} Rad{COLOR_RESET}")
                print(f"{COLOR_RISULTATO}Giunto 3 (q3):{COLOR_RESET} {COLOR_VALORE}{q3:.4f} Metri{COLOR_RESET}")
                print(f"{COLOR_RISULTATO}Matrice Trasformazione Omogenea world - endEffector:{COLOR_RESET}")
                print(f"{COLOR_VALORE}{np.round(tWorldEE, 3)}{COLOR_RESET}")
                print(f"{COLOR_RISULTATO}Determinante Jacobiano:{COLOR_RESET} {COLOR_VALORE}{detJ:.6f} ({statusSing}){COLOR_RESET}")

                # DD Pubblicazione Del Nuovo Stato Per Aggiornare RViz
                targetQ1, targetQ2, targetQ3 = q1, q2, q3
            else:
                print(f"{COLOR_ERR}[jointController] Impossibile Trovare Una Soluzione Valida Per I Giunti{COLOR_RESET}")

        # BB ---------- Modalità Uscita ----------
        elif kinematicType == "esci":
            # CC Spegnimento Del Nodo ROS E Chiusura Forzata Di Tutti I Thread
            print(f"{COLOR_USCITA}[jointController] Richiesta Chiusura Utente{COLOR_RESET}\n")
            # DD Configurazione Avanzata: Invia Un Segnale SIGINT Al Processo Genitore (roslaunch) Per Terminare Tutti I Nodi Come Se Si Premesse Ctrl+C Per Uscire Dal Ciclo Principale
            os.kill(os.getppid(), signal.SIGINT)
            sys.exit(0)

        else:
            # CC Messaggio Di Avviso Per Scelte Del Menu Principale Non Valide
            print(f"{COLOR_WARN}[jointController] Scelta Non Valida. Selezionare [1] Diretta, [2] Inversa Oppure [3] Esci.{COLOR_RESET}")

except (KeyboardInterrupt, rospy.ROSInterruptException, EOFError):
    print(f"{COLOR_USCITA}[jointController] Interruzione Da Terminale Ricevuta (Ctrl+C). Chiusura In Corso...{COLOR_RESET}")
    sys.exit(0)
