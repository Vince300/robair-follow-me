#! /bin/bash
#on compile
cd ../../ && catkin_make --source src/suivi
#problèmes de PATH , on recopie à l'endroit ou il peut lire
for i in /home/robair/catkin_ws/devel/lib/suivi/* 
do
	cp $i /home/robair/catkin_ws/install/share/suivi/
done
#on revient à l'endroit ou l'on est
cd -
