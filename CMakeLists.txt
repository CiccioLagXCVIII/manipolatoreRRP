## AA Inizializzazione e Requisiti CMake
## BB Definisce La Versione Minima Di CMake E Imposta Il Nome Del Progetto ROS
cmake_minimum_required(VERSION 3.0.2)
project(manipolatoreRRP)

## Compilazione Come Pacchetto C++11
# add_compile_options(-std=c++11)

## AA Ricerca Pacchetti e Macro Catkin
## BB Individua Le Macro E Le Librerie Catkin Specificate Nel Blocco COMPONENTS
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf
  tf2_ros
)

## AA Dipendenze di Sistema
## BB Gestisce La Ricerca Delle Librerie Esterne E Di Sistema Tramite Le Convenzioni Standard Di CMake
# find_package(Boost REQUIRED COMPONENTS system)

## AA Configurazione Moduli Python
## BB Abilita Il Supporto A setup.py Per Registrare E Installare Moduli Python Globali
## Da Decommentare Se Il Pacchetto Ha Un File setup.py Per Assicurarsi Che I Moduli E
## Gli Script Globali Vengano Installati
## Per La Documentazione Vedere: http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## AA Dichiarazione Messaggi, Servizi e Azioni ROS
## BB Setup Della Generazione E La Compilazione Di Messaggi, Servizi E Azioni Personalizzati

## Per Dichiarare E Compilare Messaggi, Servizi O Azioni, Seguire Questi Passaggi:
## 1. Inserire In MSG_DEP_SET L'Insieme Dei Pacchetti Con I Tipi Di Messaggi Che Vengono Usati (Ad Es. std_msgs, actionlib_msgs, ...).
## 2. Nel File package.xml:
##    2.1 Aggiungere Un Tag build_depend Per "message_generation"
##    2.2 Aggiungere Un Tag build_depend E Un Tag exec_depend Per Ciascun Pacchetto In MSG_DEP_SET
##    2.3 Aggiungere Un Tag exec_depend Per "message_runtime"
## 3. Nel File CMakeLists.txt:
##    3.1 Aggiungere "message_generation" E Ogni Pacchetto Presente In MSG_DEP_SET A
##     find_package(catkin REQUIRED COMPONENTS ...)
##   3.2 Aggiungere "message_runtime" E Ogni Pacchetto Presente In MSG_DEP_SET A
##     catkin_package(CATKIN_DEPENDS ...)
##   3.3 Decommentare Le Sezioni add_*_files Necessarie Ed Elencare I File .msg/.srv/.action Da Elaborare
##   3.4 Decommentare generate_messages
##   3.5 Aggiungere In generate_messages(DEPENDENCIES ...) Ogni Pacchetto Presente In MSG_DEP_SET

## Genera Messaggi Nella Cartella 'msg'
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Genera Servizi Nella Cartella 'srv'
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Genera Azioni Nella Cartella 'action'
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Genera I Messaggi E I Servizi Aggiunti Con Qualsiasi Dipendenza Elencata
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # O altri pacchetti contenenti messaggi
# )

## AA Parametri di Dynamic Reconfigure
## BB Configura La Generazione Delle Opzioni Per La Riconfigurazione Dinamica Dei Nodi

## Per Dichiarare E Compilare I Parametri Di Riconfigurazione Dinamica Dei Nodi
## 1. Nel File Package.xml:
##   1.1 Aggiungere Un Tag build_depend E Un exec_depend Per "dynamic_reconfigure"
## 2. Nel File CMakeLists.txt:
##   2.1 Aggiungere "dynamic_reconfigure" A
##     find_package(catkin REQUIRED COMPONENTS ...)
##   2.2 Decommentare La Sezione "generate_dynamic_reconfigure_options" Ed Inserire Ogni File .cfg Da Elaborare

## Genera I Parametri Di Riconfigurazione Dinamica Nella Cartella 'cfg'
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

## AA Configurazione Specifica Catkin
## BB Genera I File Di Configurazione CMake Ed Esporta Dipendenze E Percorsi

## La Macro catkin_package Genera I File Di Configurazione Per Il Pacchetto
## Dichiara Gli Elementi Da Passare Ai Progetti Dipendenti
## INCLUDE_DIRS: Decommentare Se Il Pacchetto Contiene File Header
## LIBRARIES: Librerie Create In Questo Progetto Che Servono Anche Ai Progetti Dipendenti
## CATKIN_DEPENDS: Pacchetti Catkin Di Cui Anche I Progetti Dipendenti Hanno Bisogno
## DEPENDS: Dipendenze Di Sistema Di Questo Progetto Di Cui Anche I Progetti Dipendenti Hanno Bisogno
# Esportazione Delle Dipendenze Catkin Del Pacchetto
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs tf tf2_ros
)

## AA Compilazione ed Header Files
## BB Definisce Le Directory Degli include, I Target Di Compilazione C++, Eseguibili E Librerie

## Specifica Posizioni Aggiuntive Per I File Header
# Le Posizioni Dei Pacchetti Devono Essere Elencate Prima Di Altre Posizioni
# include_directories(
#  ${catkin_INCLUDE_DIRS}
# )

## Dichiarazione Librerie C++
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/manipolatoreRRP.cpp
# )

## Aggiunge Le Dipendenze Della Libreria
## Per Esempio, Potrebbe Essere Necessario Generare Il Codice Delle Librerie, Prima Sia Della Generazione
## Di Messaggi Che Della Riconfigurazione Dinamica
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Dichiarazione Eseguibile C++
## Con catkin_make Tutti I Pacchetti Vengono Compilati All'Interno Di Un Unico Contesto. Il Prefisso Garantisce Che
## I Nomi Dei Target Tra I Vari Pacchetti Non Entrino In Conflitto
# add_executable(${PROJECT_NAME}_node src/robot_node.cpp)

## Rinomina L'eseguibile C++ Senza Prefisso
## Il Prefisso Consigliato Causa Nomi Di Target Lunghi, E La Seguente Istruzione Rinomina Il Target Alla Versione
## Più Breve Per Facilità D'Uso. Per Esempio: "Rosrun Nome_pacchetto Nodo" Invece Di "Rosrun Nome_pacchetto Nome_pacchetto_nodo"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Aggiunge Le Dipendenze Dei Target
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specifica Le Librerie Con Cui Collegare Una Libreria O Un Target Eseguibile
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## AA Regole di Installazione
## BB Specifica La Destinazione Di Installazione Per Script Python, File Ausiliari, Risorse E Configurazioni

# Tutti I Target Di Installazione Dovrebbero Usare Le Variabili catkin DESTINATION
# Per La Documentazione Vedere: http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Individua Gli Script Eseguibili Per L'Installazione E A Differenza di setup.py, Si Può Scegliere La Destinazione

## Individua Gli Script Eseguibili E Moduli Di Supporto Per L'Installazione
catkin_install_python(PROGRAMS
  scripts/directKinematics.py
  scripts/inverseKinematics.py
  scripts/jointController.py
  scripts/kinematicsUtils.py
  scripts/tfPrinter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Contrassegna Gli Eseguibili Per L'Installazione
## Per La Documentazione Vedere: http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Contrassegna Le Librerie Per L'Installazione
## Per La Documentazione Vedere: http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Contrassegna I File Header cpp Per L'Installazione
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Contrassegna Altri File Per L'Installazione (Per Esempio: File Launch, ROS Bag, Ecc.)
install(DIRECTORY config launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## AA Sezione Test e Collaudo
## BB Configura I Test Automatici Basati Su gtest  Per C++ E nosetests Per Python

## Aggiunge Un Target Di Test Cpp Basato Su gtest e Collega Le Librerie
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Aggiunge Cartelle Da Eseguire Con I Nosetest Di Python
# catkin_add_nosetests(test)
