# Le fichier OpenNIDevEnvironement est généré par "sudo ./install.sh" dans le
# dossier de OpenNI. Ce script est dépendant de l'emplacement d'installation.
source ~/groupeDufour/OpenNI-Linux-x86-2.2/OpenNIDevEnvironment

# Importation des .so de OpenNI2, nécessaire à l'exécution du programme tracker.
export LD_LIBRARY_PATH=~/groupeDufour/OpenNI-Linux-x86-2.2/Redist

# Alias utiles pour compiler
alias cm="(cd ~/groupeDufour/catkin && catkin_make)"
alias sr="pkill tracker & rostopic pub -1 /cmdmotors md49test/MotorCmd -- 0 0"
