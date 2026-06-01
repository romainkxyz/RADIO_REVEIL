# Radio-Réveil STM32

Réveil embarqué réalisé sur **NUCLEO-L152RE** avec le shield **ISEN32** et le shield capteurs **X-NUCLEO-IKS01A3**.

Il affiche l'heure, la date, la température et l'humidité sur un afficheur 7 segments,
et déclenche une alarme programmable que l'on peut couper
avec un bouton ou en retournant le réveil.

## Fonctionnalités

- Affichage cyclique : heure, date, température, humidité
- Alarme programmable (heure + minute) avec sonnerie et vibration
- Arrêt de l'alarme par bouton ou en retournant le réveil (accéléromètre)
- Volume et intensité de vibration réglables par potentiomètres
- Luminosité de l'afficheur réglable

## Matériel

- Carte STM32 NUCLEO-L152RE
- Shield ISEN32
- Shield X-NUCLEO-IKS01A3

## Notice d'emploi

### Boutons

| Bouton | Rôle |
|--------|------|
| **BP1** | MODE / Valider (entre dans les réglages et passe au champ suivant) |
| **BP2** | +1 (incrémente le champ qui clignote) / Activer-désactiver l'alarme |
| **BP3** | Diminuer la luminosité de l'afficheur |
| **BP4** | Augmenter la luminosité de l'afficheur |

### Affichage normal

L'afficheur fait défiler automatiquement toutes les 5 secondes :
heure `HH.MM` → date `JJ.MM` → température `xx°C` → humidité `xxhu`.

### Régler l'heure, l'alarme et la date

1. Appuyer sur **BP1** pour entrer dans les réglages.
2. Le champ en cours d'édition **clignote**.
3. Appuyer sur **BP2** pour incrémenter la valeur.
4. Appuyer sur **BP1** pour valider et passer au champ suivant :
   `Heure → Minute → Alarme (heure) → Alarme (minute) → Jour → Mois → retour normal`.

### Activer / désactiver l'alarme

En mode normal, appuyer sur **BP2** : l'afficheur indique `A ON` ou `AOFF`.
Quand l'alarme est armée, la **LED témoin** est allumée.

### Quand l'alarme sonne

- Le buzzer joue une mélodie et le moteur vibre.
- **Pour l'arrêter** : appuyer sur n'importe quel bouton **ou retourner le réveil** (face vers le bas).

### Réglages analogiques

- **Potentiomètre RV1** : intensité de la vibration du moteur.
- **Potentiomètre RV2** : volume du buzzer.

## Compilation

Projet STM32CubeIDE : ouvrir le dossier, compiler puis flasher la carte via le ST-LINK intégré.

## Console série (debug)

Le programme envoie des logs sur l'UART (température, humidité, gyroscope toutes les 30 s, et messages d'initialisation des capteurs).

- Liaison : **USART2** (PA2 = TX, PA3 = RX), via le ST-LINK / port COM virtuel.
- Configuration du terminal (PuTTY, CoolTerm, moniteur CubeIDE) :

| Paramètre | Valeur |
|-----------|--------|
| Vitesse (baud) | **115200** |
| Bits de données | 8 |
| Parité | Aucune |
| Bits de stop | 1 |
| Contrôle de flux | Aucun |

> Format résumé : **115200 8N1**.
