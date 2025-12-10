## Description du projet

Ce projet développe des agents autonomes capables de piloter des robots e-puck dans un environnement simulé sous Webots.
L’objectif est d’explorer deux niveaux d’autonomie en intelligence artificielle :

Autonomie réactive – niveau 2
Autonomie cognitive – niveau 3

Les agents doivent être capables de :

-naviguer dans un environnement pouvant contenir des obstacles ;
-détecter et atteindre une cible ;
-fonctionner de manière autonome, individuellement ou collectivement.

Le projet repose sur un ensemble de capteurs et actuateurs du robot e-puck :
8 capteurs infrarouges, caméra, 10 LEDs, moteurs différentiels et module WiFi.

## Architecture générale

Le projet propose deux implémentations fondamentales :

### 1. Agent réactif — Niveau d’autonomie 2 (autonomyTwo.java)

Un agent sans mémoire, sans planification, ni représentation interne.
Il réagit uniquement aux stimuli sensoriels.

Caractéristiques :

-évitement d’obstacles via capteurs infrarouges ;
-détection de la cible via la caméra ;
-comportement d’approche directe ;
-mécanisme anti-blocage pour éviter la stagnation.

Résultats :

-comportement robuste malgré l'absence de planification ;
-le robot atteint la cible dans tous les environnements testés ;
-temps d'arrivée variables en fonction des obstacles ;
-trajectoires parfois non optimales, ce qui est typique d’un agent réactif.

### 2. Agent cognitif — Niveau d’autonomie 3 (autonomyThree.java)

Une approche multi-agents dans laquelle plusieurs e-puck coopèrent pour trouver et atteindre une cible.

Capacités supplémentaires :

-représentation interne minimale de l’environnement ;
-communication WiFi entre robots ;
-partage d’informations (broadcast) ;
-auto-organisation du groupe ;
-utilisation combinée :
-odométrie
-détection visuelle
-communication inter-robots

Résultats :

-grande robustesse du système multi-agents ;
-un seul robot ayant détecté la cible peut guider les autres ;
-convergence fiable même dans des environnements complexes.

## Technologies utilisées

-Langage : Java
-Simulateur : Webots
-Robot simulé : e-puck
-Modèle : Systèmes multi-agents réactifs & cognitifs
