#!/usr/bin/env python3

# AA Spiegazione Approfondita Nel File "6) cinematicaInversaProgetto.tex"

# AA Importazioni Necessarie
import os
import sys
# BB Path Per Consentire L'Importazione Dei Moduli Dalla Cartella Scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import numpy as np
import kinematicsUtils

# AA Codici Colore ANSI Per Terminale
COLOR_WARN      = "\033[33m"        # Giallo Normale        (Avvisi)
COLOR_ERR       = "\033[31m"        # Rosso Normale         (Errori)
COLOR_RESET     = "\033[0m"         # Reset Stile           (Ripristina Colore Standard)

def computeInverseKinematics(xTarget, yTarget, zTarget):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    kinematicsUtils.loadRobotParameters()

    # CC Assegnazione Delle Variabili
    worldBase = kinematicsUtils.worldBase
    baseHeight = kinematicsUtils.baseHeight
    l1 = kinematicsUtils.l1
    l2 = kinematicsUtils.l2
    l3 = kinematicsUtils.l3
    jointRadius = kinematicsUtils.jointRadius
    boxSize = kinematicsUtils.boxSize

    # CC Calcolo Valori Della Tabella Di Denavit Hartenberg
    d1 = 2*jointRadius + l1 + jointRadius       # Distanza Da RF0 Al RF1
    a2 = jointRadius + l2 + (boxSize / 2.0)     # Distanza Da RF1 Al RF2

    # DD Traslazione Delle Coordinate Target Dal Frame World Al Frame Zero Della Base Rotante
    x = xTarget - worldBase
    y = yTarget
    z = zTarget - baseHeight

    # AA Calcolo Variabile Di Giunto Prismatico q3
    squareRootArg = x**2 + y**2 + (z - d1)**2 - a2**2

    # BB Tolleranza Per Prevenire Imprecisioni Di Floating-Point Ai Confini Del Workspace (Dava Errore Quando Era Vicino Al Confine Del WS)
    tolerance = 1e-6

    # CC Verifica Della Condizione Di Esistenza Per Evitare Radici Di Numeri Negativi
    if squareRootArg < -tolerance:
        print(f"{COLOR_WARN}[inverseKinematics] Target Non Raggiungibile. Si Trova Fuori Dal Workspace Del Robot{COLOR_RESET}")
        return None, None, None

    # DD Prendendo Il Massimo Tra 0 E squareRootArg Si Evita Che La Funzione np.sqrt() Generi Warning Per Valori Negativi (Che Possono Accadere Per Imprecisioni Di Floating-Point)
    squareRootArg = max(0.0, squareRootArg)

    # BB Calcolo Delle Due Possibili Soluzioni Per Il Giunto Prismatico q3
    # CC Da d3^2 = squareRootArg Si Ottengono Due Possibili Soluzioni Per d3, Una Con Radice Positiva E Una Con Radice Negativa
    # DD Soluzione Con Radice Positiva (Link 3 E End Effector Lungo La Direzione Positiva Dell'Asse Z Locale )
    q3P = (l3 + boxSize / 2.0) + np.sqrt(squareRootArg)
    # DD Soluzione Con Radice Negativa (Link 3 E End Effector Lungo La Direzione Negativa Dell'Asse Z Locale )
    q3N = (l3 + boxSize / 2.0) - np.sqrt(squareRootArg)

    # CC Nel Caso Di Questo Manipolatore Il Link 3 E L'End Effector Si Estendono Entrambi Verso Il Basso, Il Valore d3 È Sempre Negativo,
    # CC E Come Limite Massimo Ha l3 La Soluzione q3P Non Rientra Nei Limiti Del Giunto Prismatico
    # CC In Quanto q3P = l3 + [ (boxSize / 2.0) + sqrt(squareRootArg) ] > l3
    # CC Quindi La Metto Solo Per Completezza, Ma Non Verrà Mai Selezionata

    # DD Scelta Della Soluzione Che Rientra Nei Limiti Del Giunto Prismatico

    q3 = None
    minQ3 = kinematicsUtils.limitMinQ3 - tolerance
    maxQ3 = kinematicsUtils.limitMaxQ3 + tolerance

    if minQ3 <= q3N <= maxQ3:
        # EE Sempre Per Il Motivo Che Per Valori Vicino Al Bordo Del WS Da Errore, Assegno A q3 Il Valore Limitato Tra I Limiti Fisici Del Giunto Prismatico
        q3 = float(np.clip(q3N, kinematicsUtils.limitMinQ3, kinematicsUtils.limitMaxQ3))
    elif minQ3 <= q3P <= maxQ3:
        # EE Come Detto Sopra, Questa Soluzione Non Verrà Mai Selezionata, Ma La Controllo Per Completezza Algebrica
         q3 = float(np.clip(q3P, kinematicsUtils.limitMinQ3, kinematicsUtils.limitMaxQ3))
    else:
        rospy.logwarn("[inverseKinematics] Nessuna Delle Due Soluzioni Di q3 Rientra Nei Limiti Fisici Del Giunto 3")
        return None, None, None

    # BB Calcolo Distanza d3 Corrispondente Alla Soluzione Selezionata
    d3 = q3 - (l3 + boxSize / 2.0)

    # AA Calcolo Della Variabile Di Giunto Rotatorio q2
    # BB Calcolo Raggio Workspace (Ovviamente Si Deve Considerare Il Valore Posivo E Il Valore Negativo)
    r = np.sqrt(x**2 + y**2)

    # BB Definizione Configurazioni Per Il Giunto 2 (Frontale E All'Indietro)
    # CC Calcolo Valore q2 (Raggio Positivo) Quando Il Link 2 È Nella Direzione Del Target
    q2Front = np.arctan2(d3 * r - a2 * (z - d1), a2 * r + d3 * (z - d1))
    # CC Calcolo Valore q2 (Raggio Negativo) Quando Il Link 2 È Nella Direzione Opposta A Quella Del Target
    q2Back = np.arctan2(d3 * (-r) - a2 * (z - d1), a2 * (-r) + d3 * (z - d1))

    # DD Selezione Della Configurazione Del Giunto 2 Che Rispetta I Limiti Del Secondo Giunto
    # EE La Soluzione q2Front (Raggio Positivo) Si Verifica Quando Il Giunto 1 Orienta Il Link 2 Nella Direzione Del Target,
    # EE Mentre La Soluzione q2Back (Raggio Negativo) Si Verifica Quando Il Giunto 1 Orienta Il Link 2 Nella Direzione
    # EE Opposta A Quella Del Target E Quindi Il Giunto 2 Dovrebbe Andare Verso Valori Ancora Più Negativi (Oltre -3.14)
    # EE Scendendo Verso Il Basso E Raggiungere Il Punto "Da Sotto"
    # EE Nel Caso Di Questo Manipolatore, Poichè q2 È Limitato Tra [-pi/2, +pi/4], L'Unico Modo Fisicamente Possibile
    # EE Per Raggiungere Il Target È Fare In Modo Che Il Link 2 Punti Direttamente Verso Il Target (Orientando Il
    # EE Giunto 1 Verso Di Esso) Ovvero Usando La Soluzione q2Front.
    # EE Di Conseguenza La Soluzione q2Back Non Rientra Mai Nei Limiti Fisici, Ma La Controllo Per Completezza Algebrica
    q2 = None
    minQ2 = kinematicsUtils.limitMinQ2 - tolerance
    maxQ2 = kinematicsUtils.limitMaxQ2 + tolerance

    # EE Con Il Primo If Anche Se q2Back Rientrasse Nei Limiti, Essendo Verificato Il Primo If Non Verrebbe Mai Selezionato (Anche Perchè q2Front È Sempre La Soluzione Corretta Per Questo Manipolatore)
    if minQ2 <= q2Front <= maxQ2:
      q2 = q2Front
    elif minQ2 <= q2Back <= maxQ2:
        q2 = q2Back
    else:
        rospy.logwarn("[inverseKinematics] Nessuna Configurazione Per q2 Che Rientra Nei Limiti Del Giunto")
        return None, None, None

    # AA Calcolo Della Variabile Di Giunto Rotatorio q1
    # CC Si Calcola Tramite Arcotangente A Quattro Quadranti
    denom = d3 * np.sin(q2) + a2 * np.cos(q2)

    # DD Dividere Per Un Valore Molto Piccolo (Vicino A Zero) Può Portare A Risultati Numerici Molto Grandi E Instabili
    # DD Quindi Per Evitare Divisioni Per Zero, Controllo Se Il Denominatore È Vicino A Zero E In Tal Caso
    # DD Imposto q1 A Zero (Ovvero Oriento Il Giunto 1 Verso L'Asse X)
    if np.abs(denom) < 1e-6:
        # EE Target Sull'Asse Verticale: Singolarità Di Spalla
        rospy.logwarn("[inverseKinematics] Target Sull'Asse Verticale: q1 Indeterminato, Impostato a 0.0 rad")
        q1 = 0.0
    else:
        # DD Inoltre Dividere Per Una Costante Entrambi I Membri Dell'Arcotangente A Quattro Quadranti Non Cambia Il Risultato,
        # DD Il Denominatore Serve Solo Per Il Segno. Poichè Il Controllo Della Singolarità Avviene Solo Per Valori Minori Di 1e-6,
        # DD Potrebbero Esserci Problemi Con Valori Molto Piccoli Ma Maggiori Di 1e-6, Quindi Estraggo Il Segno Dal Denominatore
        # DD E Lo Uso Per Calcolare La Rotazione Della Base
        if denom >= 0.0:
            signDenom = 1.0
        else:
            signDenom = -1.0

        q1 = np.arctan2(y * signDenom, x * signDenom)

    # CC Controllo Dei Valori Dei Giunti Rispetto Ai Limiti Fisici Del Robot
    isValid, q1, q2, q3 = kinematicsUtils.checkJointLimits(q1, q2, q3)

    # DD Se isValid È False, Significa Che Almeno Uno Dei Valori Dei Giunti Non Rientra Nei Limiti Fisici Del Robot
    # DD Quindi Restituisco None Per Tutti E Tre I Valori Dei Giunti Per Indicare Che La Soluzione Non È Valida
    if not isValid:
        return None, None, None
    else:
        return q1, q2, q3

# AA Blocco Di Test Per Verificare La Funzionalità Della Cinematica Inversa
if __name__ == "__main__":
    rospy.init_node("testInverseKinematics", anonymous=True)
    print("[inverseKinematics] [TEST CINEMATICA INVERSA]")
    xTest, yTest, zTest = 2.55, 0.0, 0.35
    print(f"[inverseKinematics] Target Di Test: X={xTest}, Y={yTest}, Z={zTest}")
    q1, q2, q3 = computeInverseKinematics(xTest, yTest, zTest)
    if q1 is not None:
        print(f"[inverseKinematics] Soluzione Trovata: q1={q1:.4f} rad, q2={q2:.4f} rad, q3={q3:.4f} m")
    else:
        print("[inverseKinematics] Target Non Raggiungibile.")
