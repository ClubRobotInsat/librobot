initSidebarItems({"enum":[["Command","Une commande pour un moteur : une direction et une vitesse sur 16 bits (0 : vitesse nulle)."]],"struct":[["Coord","Les coordonnées x,y d'un point sur la table"],["Motor","Un moteur avec ses deux broches : vitesse et direction."],["PIDParameters","Les paramètres d'un PID"],["RealWorldPid","Le module central de la navigation, qui permet de controller le robot avec les unités du monde physique, et d'avoir un retour sur la position du robot. Il contient: * un PID basé sur la distance parcourue par le robot en millimètres * un module d'odométrie capable de retrouver la position et l'angle du robot * les informations nécessaires pour passer du monde des ticks de roue codeuses au monde physique * les qei gauche et droite correspondant aux deux roues codeuses * la commande à appliquer aux moteurs gauche et droit"]]});