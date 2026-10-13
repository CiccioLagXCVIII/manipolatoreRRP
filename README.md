# 🦾 Modellazione Geometrica e Controllo Cinematico: Manipolatore Seriale RRP

Questo progetto propone uno studio applicato e rigoroso per la modellazione tridimensionale, l'analisi cinematica (diretta e inversa analitica in forma chiusa) e la simulazione real-time di un braccio robotico seriale a 3 gradi di libertà in configurazione **RRP** (Rotatorio-Rotatorio-Prismatico), equipaggiato con un end-effector a pinza rigida per compiti di manipolazione su piano.

A differenza delle soluzioni basate su librerie di cinematica numerica black-box (come KDL o TRAC-IK), questo lavoro implementa una **risoluzione analitica esplicita** dei vincoli geometrici, integrando la validazione preliminare dello spazio di lavoro (*workspace validation*), il monitoraggio continuo delle singolarità differenziali tramite il determinante Jacobiano e un controllore interattivo con interpolazione fluida del profilo di moto a 20 Hz.

---

## 🔬 La Sfida Ingegneristica: Risoluzione Analitica e Prevenzione delle Singolarità

La principale sfida metodologica risiede nella corretta derivazione analitica e nel disaccoppiamento algebrico delle variabili di giunto $\mathbf{q} = [q_1, q_2, q_3]^T$, garantendo robustezza numerica anche in prossimità dei confini fisici del robot.

### Vincoli Progettuali e Limitazioni Meccaniche:
*   **Limiti di Giunto Rigidi:** Escursioni angolari calibrate per prevenire collisioni meccaniche tra segmenti (es. $q_2 \in [-\pi/2, +\pi/4]$ impedisce l'impatto distale contro il montante verticale).
*   **Comportamento Telescopico Negativo:** L'asta prismatica opera a sbalzo verso il basso lungo l'asse $Z$ locale ($d_3 < 0$), richiedendo la selezione analitica della sola radice coerente col moto dell'end-effector.
*   **Gestione della Singolarità di Spalla:** Trattamento esplicito della perdita di rango dello Jacobiano sul montante verticale ($r_{xy} \to 0$), prevenendo instabilità numeriche nella funzione trigonometrica a quattro quadranti $\text{atan2}$.

---

## 🛠️ Stack Tecnologico e Ambiente di Esecuzione

*   **Piattaforma e Middleware:** ROS 1 Noetic Ninjemys su Ubuntu 20.04 LTS (compatibile con architetture Docker e WSL2)
*   **Modellazione 3D e Dinamica:** URDF, XACRO (XML Macros parametrizzato)
*   **Calcolo Scientifico e Simbolico:** Python 3.8+, NumPy, SymPy
*   **Visualizzazione e Trasformazioni Spaziali:** RViz, TF2 (`tf2_ros`, `tf.transformations`)
*   **Core ROS Packages:** `rospy`, `sensor_msgs`, `robot_state_publisher`, `joint_state_publisher_gui`

---

## 🏗️ Architettura del Manipolatore e Modellazione XACRO

L'architettura geometrica del robot è completamente parametrizzata tramite il file di configurazione `config/robotParameters.yaml`, che governa dinamicamente sia il parsing delle macro XACRO sia le classi di utilità cinematica in Python.

```
[world] ➔ (2.0m lungo X) ➔ [base] ➔ (q1: Rot Z) ➔ [RF0] ➔ (q2: Rot Y) ➔ [RF1] ➔ (q3: Prism Z) ➔ [RF2] ➔ (Fisso: -180° X) ➔ [endEffector]
```

### Componenti Strutturali della Catena:
*   **Terna World & Offset di Base:** Origine cartesiana fissa con montante robotico traslato di $X = 2.0\text{ m}$.
*   **Giunto 1 e Colonna Verticale (`RF0`):** Giunto rotoidale attorno all'asse $Z$ ($q_1 \in [-\pi, +\pi]\text{ rad}$) con link cilindrico di lunghezza $l_1 = 0.55\text{ m}$.
*   **Giunto 2 e Braccio Radiale (`RF1`):** Giunto rotoidale di spalla attorno all'asse $Y$ ($q_2 \in [-\pi/2, +\pi/4]\text{ rad}$) e link orizzontale di estensione $l_2 = 0.45\text{ m}$, terminante con un alloggiamento a guida prismatica di lato $0.10\text{ m}$.
*   **Giunto 3 e Asta Telescopica (`RF2`):** Giunto lineare lungo l'asse $Z$ ($q_3 \in [0.0, 0.35]\text{ m}$) con asta a sezione circolare di corsa utile $l_3 = 0.35\text{ m}$.
*   **Pinza Terminale (`endEffector`):** Piastra base con coppia di dita parallele di presa, orientata a $180^\circ$ rispetto alla flangia per indirizzare l'approccio operazionale verso il suolo.

---

## 📐 Pipeline Cinematica I: Modello di Denavit-Hartenberg (DH)

La cinematica diretta adotta la convenzione classica di Denavit-Hartenberg dal frame di giunto `RF0` al frame terminale `endEffector`:

| Giunto / Link $i$ | $\theta_i$ (rad) | $d_i$ (m) | $a_i$ (m) | $\alpha_i$ (rad) | Note Descrizione |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** (`RF0` $\rightarrow$ `RF1`) | $q_1$ | $2r_{\text{joint}} + l_1 + r_{\text{joint}}$ | $0$ | $-\pi/2$ | Asse di rotazione verticale ($d_1 = 0.70\text{ m}$) |
| **2** (`RF1` $\rightarrow$ `RF2`) | $q_2$ | $0$ | $r_{\text{joint}} + l_2 + \frac{\text{box}}{2}$ | $+\pi/2$ | Asse di beccheggio spalla orizzontale ($a_2 = 0.55\text{ m}$) |
| **3** (`RF2` $\rightarrow$ `EE`) | $0$ | $q_3 - (l_3 + \frac{\text{box}}{2})$ | $0$ | $-\pi$ | Estensione prismatica verso il basso con ribaltamento pinza |

Ciascuna matrice di trasformazione omogenea parziale $T_{i-1}^{i}$ è calcolata tramite la composizione canonica:
$$T_{i-1}^i = R_z(\theta_i) \, T_z(d_i) \, T_x(a_i) \, R_x(\alpha_i)$$

La posa globale dell'organo terminale rispetto al frame inerziale `world` è ottenuta assemblando l'albero cinematico:
$$T_{\text{world}}^{\text{EE}} = T_{\text{world}}^{\text{base}} \cdot T_{\text{base}}^{\text{RF0}} \cdot T_0^1(q_1) \cdot T_1^2(q_2) \cdot T_2^3(q_3)$$

---

## 🔄 Pipeline Cinematica II: Risoluzione Analitica della Cinematica Inversa

Il solutore `inverseKinematics.py` determina la terna di variabili $\mathbf{q} = [q_1, q_2, q_3]$ per qualsiasi target $\mathbf{p} = [X_t, Y_t, Z_t]^T$ in coordinate `world` mediante derivazione analitica in tre stadi disaccoppiati:

### 1. Calcolo del Giunto Prismatico $q_3$
Traslando l'obiettivo nella terna base $(x, y, z)$, la norma euclidea rispetto al centro di rotazione del secondo giunto soddisfa:
$$x^2 + y^2 + (z - d_1)^2 = a_2^2 + d_3^2 \implies d_3 = -\sqrt{x^2 + y^2 + (z - d_1)^2 - a_2^2}$$
Dato il vincolo costruttivo dell'asta telescopica ($d_3 < 0$), viene scartata la radice positiva e selezionata l'unica soluzione fisicamente compatibile entro i limiti meccanici:
$$q_3 = \left(l_3 + \frac{\text{box}}{2}\right) - \sqrt{x^2 + y^2 + (z - d_1)^2 - a_2^2}$$

### 2. Disaccoppiamento di Spalla: Giunto Rotoitale $q_2$
Posto il raggio orizzontale $r = \sqrt{x^2 + y^2}$, la proiezione cinematica sul piano verticale genera due configurazioni teoriche (frontale e dorsale). L'algoritmo isola la configurazione ammissibile nei limiti $[-\pi/2, +\pi/4]$ mediante:
$$q_2 = \text{atan2}\big(d_3 r - a_2 (z - d_1), \; a_2 r + d_3 (z - d_1)\big)$$

### 3. Azimut di Base: Giunto Rotoidale $q_1$
Noto l'angolo $q_2$, la rotazione di base è ricavata esplicitando il denominatore geometrico $\Delta = d_3 \sin(q_2) + a_2 \cos(q_2)$:
$$q_1 = \text{atan2}\big(y \cdot \text{sgn}(\Delta), \; x \cdot \text{sgn}(\Delta)\big)$$
Se $|\Delta| < 10^{-6}$ (singolarità di spalla con bersaglio allineato all'asse di rotazione verticale), il sistema rileva l'indeterminazione dell'azimut, imponendo per convenzione $q_1 = 0.0\text{ rad}$ ed evitando divisioni per zero.

---

## ⚠️ Analisi dello Spazio di Lavoro (Workspace) e Singolarità

Il modulo di sicurezza preventiva `kinematicsUtils.py` effettua verifiche rigorose prima dell'invio dei target di giunto ai publisher di ROS:

### Geometria del Workspace Ammissibile
*   **Inviluppo Sferico:** La distanza euclidea $r_D$ dal centro della spalla è delimitata da due sfere concentriche imposte dalla corsa del prismatico:
    $$r_{D,\min} = \sqrt{a_2^2 + d_{3,\min}^2} \approx 0.552\text{ m}, \qquad r_{D,\max} = \sqrt{a_2^2 + d_{3,\max}^2} \approx 0.680\text{ m}$$
*   **Raggio Minimo Cilindrico:** Per prevenire penetrazioni fisiche nel volume del montante verticale, il raggio planare deve soddisfare $r_{xy} \ge |d_{3,\text{retracted}}| = 0.05\text{ m}$.

### Determinante Jacobiano Differenziale
La matrice Jacobiana $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{3 \times 3}$ correla le velocità di giunto $\mathbf{\dot{q}}$ alla velocità lineare dell'organo terminale $\mathbf{v}_{\text{EE}}$. Sfruttando la proprietà di invarianza di norma rispetto a rotazioni $Z$, il determinante è dato in forma chiusa da:
$$\det(\mathbf{J}) = -d_3 \cdot \big(d_3 \sin(q_2) + a_2 \cos(q_2)\big)$$

Il sistema classifica in tempo reale due condizioni critiche:
1.  **Singolarità Prismatica ($d_3 \to 0$):** Irraggiungibile fisicamente grazie alla battuta meccanica dell'alloggiamento guida ($d_3 \le -0.05\text{ m}$).
2.  **Singolarità di Spalla ($d_3 \sin(q_2) + a_2 \cos(q_2) \to 0$):** Corrisponde all'allineamento verticale dell'end-effector con l'asse del giunto 1, identificata con soglia di allerta numerica $\varepsilon = 10^{-3}$.

---

## 🎛️ Architettura Software e Nodi di Controllo

```sh
manipolatoreRRP
├── config
│   └── robotParameters.yaml        # File YAML centrale per quote metriche e limiti
├── launch
│   └── manipolatoreRRP.launch      # Launch file con switch di modalità GUI attiva/disattiva
├── rviz
│   └── manipolatoreRRP.rviz        # Configurazione visualizzatore RViz
├── scripts
│   ├── directKinematics.py         # Risoluzione della cinematica diretta e composizione matriciale
│   ├── inverseKinematics.py        # Algoritmo in forma chiusa di cinematica inversa
│   ├── jointController.py          # Controller CLI interattivo con interpolatore a 20 Hz
│   ├── kinematicsUtils.py          # Utility: caricamento parametri, limiti, workspace e Jacobiano
│   └── tfPrinter.py                # Listener e display terminale delle trasformazioni TF (1 Hz)
├── urdf
│   └── manipolatoreRRP.urdf.xacro  # Modello 3D modulare con macro XACRO
├── CMakeLists.txt
└── package.xml
```

### Funzionalità Avanzate dei Nodi:
*   **Parsing Simbolico degli Angoli (`jointController.py`):** L'utente può inserire costanti trigonometriche simboliche in linguaggio naturale (es. `pi`, `pi/2`, `-pi/4`, `0.5*pi`) valutate a runtime dal modulo SymPy.
*   **Interpolatore Esponenziale per RViz:** Per sopprimere scatti geometrici nella visualizzazione 3D causati da salti di coordinate, un timer a 20 Hz aggiorna lo stato secondo il filtro IIR del primo ordine:
    $$q_{\text{current}} \leftarrow q_{\text{current}} + 0.15 \cdot (q_{\text{target}} - q_{\text{current}})$$
*   **Gestione Asincrona del Terminatore:** Implementazione di I/O non bloccante via `select.select` con propagazione di `SIGINT` al processo genitore di `roslaunch` alla chiusura dell'interfaccia.
*   **Doppia Modalità di Esecuzione:** Gestione nativa del parametro booleano `gui` per separare la modalità manuale (slider grafici + display matrici TF) dalla modalità computazionale ad alto livello (CLI con cinematica diretta/inversa).

---

## 🚀 Istruzioni per la Compilazione e l'Esecuzione

### 1. Compilazione del Workspace Catkin
Assicurarsi che le cartelle del pacchetto risiedano nella directory `src` del proprio workspace ROS:

```bash
# Esecuzione del build con Catkin
catkin_make

# Caricamento delle variabili d'ambiente
source devel/setup.bash

# Assegnazione permessi di esecuzione agli script Python
roscd manipolatoreRRP/scripts && chmod +x *.py
```

### 2. Modalità A: Controllo Manuale da Interfaccia Grafica (GUI)
Avvia RViz assieme all'interfaccia a cursori di `joint_state_publisher_gui`. In parallelo, il nodo `tfPrinter` stampa a terminale (frequenza 1 Hz) la matrice di trasformazione omogenea $T_{\text{world}}^{\text{endEffector}}$:

```bash
roslaunch manipolatoreRRP manipolatoreRRP.launch gui:=true
```

### 3. Modalità B: Controller Interattivo da Terminale (CLI)
Disabilita i cursori manuali e attiva il nodo interattivo `jointController.py` su terminale, mantenendo RViz sincronizzato per la visualizzazione dinamica della posa:

```bash
roslaunch manipolatoreRRP manipolatoreRRP.launch gui:=false
```

Il menu da terminale permette di selezionare:
1.  **[1] Cinematica Diretta:** Inserimento variabili $q_1, q_2, q_3$ (supporta costanti simboliche con $\pi$) $\rightarrow$ Restituisce la matrice omogenea $4 \times 4$, le coordinate cartesiane risultanti e l'indice di singolarità.
2.  **[2] Cinematica Inversa:** Inserimento delle coordinate target $X, Y, Z$ in metri $\rightarrow$ Esegue il check di appartenenza al workspace, risolve il set di giunti e comanda la transizione interpolata del manipolatore in RViz.
3.  **[3] Esci:** Arresto pulito e simultaneo di tutti i nodi e processi attivi.

---
*Progetto realizzato per l'esame di Industrial Robotics - Corso di Laurea in Ingegneria Informatica - Curriculum Robotics*
