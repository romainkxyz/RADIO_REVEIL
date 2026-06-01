# Radio-Réveil STM32

Réveil embarqué réalisé sur **NUCLEO-L152RE** avec le shield **ISEN32** et le shield capteurs **X-NUCLEO-IKS01A3**.

Il affiche l'heure, la date, la température et l'humidité sur un afficheur 7 segments,
et déclenche une alarme programmable (buzzer + moteur vibrant) que l'on peut couper
avec un bouton ou en retournant le réveil.

## Fonctionnalités

- Affichage cyclique : heure, date, température, humidité
- Alarme programmable (heure + minute) avec sonnerie et vibration
- Arrêt de l'alarme par bouton ou en retournant le réveil (accéléromètre)
- Volume et intensité de vibration réglables par potentiomètres
- Luminosité de l'afficheur réglable

## Matériel

- Carte STM32 NUCLEO-L152RE (STM32L152RET6)
- Shield ISEN32 (afficheur, buzzer, moteur, boutons, potentiomètres, LED)
- Shield X-NUCLEO-IKS01A3 (capteurs HTS221 et LSM6DSO)

## Compilation

Projet STM32CubeIDE : ouvrir le dossier, compiler (Build) puis flasher la carte via le ST-LINK intégré.
