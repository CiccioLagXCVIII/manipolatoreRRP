# manipolatoreRRP (Work In Progress 🚧)

Questo pacchetto ROS 1 Noetic contiene il modello geometrico, la configurazione e gli script di controllo per la simulazione e l'analisi cinematica di un manipolatore antropomorfo seriale a tre gradi di libertà con configurazione **RRP** (Rotatorio, Rotatorio, Prismatico).

Il progetto è attualmente in fase di sviluppo (**Work In Progress**), con la cinematica diretta e la visualizzazione interamente operative, e la cinematica inversa predisposta per i futuri sviluppi.

---

## 1. Descrizione del Manipolatore

Il manipolatore è sviluppato secondo un'analogia meccanica realistica e si compone dei seguenti elementi:
* **Base fissa:** Posizionata a un offset di $2.0\text{ m}$ lungo l'asse $X$ rispetto al frame globale `world`.
* **Giunto 1 (Rotatorio Z):** Consente la rotazione attorno all'asse verticale della base.
* **Link 1 (Colonna verticale - `RF0`):** Elemento cilindrico ad altezza variabile.
* **Giunto 2 (Rotatorio Y):** Gestisce l'angolo di spalla dell'asse trasversale, con limiti software impostati per prevenire auto-collisioni con il primo link.
* **Link 2 (Braccio orizzontale - `RF1`):** Estensione radiale che ospita all'estremità distale l'alloggiamento fisso (scatola di guida) del giunto prismatico.
* **Giunto 3 (Prismatico Z):** Giunto lineare che scorre internamente all'alloggiamento.
* **Link 3 (Asta prismatica - `RF2`):** Asta cilindrica interna telescopica.
* **End Effector (`endEffector`):** Pinza fissa con dita simmetriche orientate verso il basso per le operazioni sul piano di lavoro.

---

## 2. Struttura del Workspace

La struttura interna del pacchetto `manipolatoreRRP` è organizzata come segue:

```text
manipolatoreRRP/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── robotParameters.yaml       # Parametri geometrici e limiti dei giunti
├── launch/
│   └── manipolatoreRRP.launch     # File di lancio principale del sistema
├── urdf/
│   └── manipolatoreRRP.urdf.xacro # Modello 3D parametrizzato del robot
├── scripts/
│   ├── kinematicsUtils.py         # Funzioni di utilità, limiti e formattazione messaggi
│   ├── directKinematics.py        # Risoluzione della cinematica diretta (DH)
│   ├── inverseKinematics.py       # Algoritmi di cinematica inversa (WIP)
│   ├── jointController.py         # Nodo di controllo e pubblicazione a 50 Hz
│   └── tfPrinter.py               # Listener TF per la stampa della matrice omogenea
└── rviz/
    └── manipolatoreRRP.rviz       # Configurazione dell'interfaccia di visualizzazione
```

---

## 3. Stato dello Sviluppo (WIP)

### Funzionalità Implementate
- [x] Generazione del modello 3D parametrizzato in formato XACRO con accoppiamenti geometrici ed estetici per i giunti.
- [x] Caricamento dinamico dei parametri fisici dal Parameter Server di ROS tramite file di configurazione YAML.
- [x] Calcolo in tempo reale della cinematica diretta mediante la convenzione di Denavit-Hartenberg.
- [x] Nodo listener per la ricezione delle trasformazioni TF e la stampa a terminale (1 Hz) della matrice di trasformazione omogenea $4 \times 4$.
- [x] Interfaccia grafica di controllo manuale dei giunti (tramite `joint_state_publisher_gui`).

### Sviluppi Futuri (In Corso)
- [ ] Implementazione e validazione analitica/numerica degli algoritmi di cinematica inversa all'interno del modulo `inverseKinematics.py`.

---

## 4. Istruzioni per l'Uso

### Compilazione del Workspace
Per compilare il pacchetto all'interno dell'ambiente di sviluppo ROS Noetic:

```bash
# AA Comando Per Eseguire Il Build Del Workspace
catkin_make

# AA Comando Per Eseguire Il Caricamento Delle Variabili Di Ambiente
source devel/setup.bash
```

### Avvio della Simulazione con Interfaccia Grafica (GUI)
Per visualizzare il robot in RViz e utilizzare gli slider per controllare i giunti in modo manuale:

```bash
# AA Comando Per Avviare Il File Launch Principale Con Interfaccia Grafica
roslaunch manipolatoreRRP manipolatoreRRP.launch
```
Durante l'esecuzione con GUI, il nodo `tfPrinter` si avvierà in background, stampando periodicamente sul terminale la matrice di trasformazione omogenea cumulativa tra il frame `world` e l'End Effector.

### Avvio con Controllo da Terminale
Per disabilitare gli slider grafici e interagire direttamente con il nodo di controllo dei giunti tramite immissione di coordinate:

```bash
# AA Comando Per Avviare Il Robot Con Controllo Tramite Terminale
roslaunch manipolatoreRRP manipolatoreRRP.launch gui:=false
```