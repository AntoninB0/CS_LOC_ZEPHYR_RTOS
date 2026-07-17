#ifndef CS_RANGING_H
#define CS_RANGING_H

#include <zephyr/kernel.h>
#include "beacon.h"

/* Setup Channel Sounding complet pour un beacon fraîchement sécurisé (L2) :
 * découverte RAS, capacités, default settings, config CS, sécurité CS,
 * paramètres de procédure. BLOQUANT (sémaphores internes).
 * À appeler depuis le thread de pairing, séquentiellement.
 * Retourne 0 si le beacon est prêt pour le ranging. */
int cs_setup_beacon(struct beacon_state *beacon);

/* Init des états de mesure par beacon. À appeler une fois avant la boucle. */
void cs_ranging_init(void);

/* FREE-RUNNING (max_procedure_count = 0 : le contrôleur répète les procédures
 * tous les procedure_interval, le handshake LL de ~495 ms n'est payé qu'à
 * l'armement) :
 *  - cs_enable_beacon(i)  : ARME les procédures du beacon i, UNE FOIS (et
 *                           après reconnexion). Retourne 0 si accepté.
 *  - cs_collect_beacon(i) : attend la prochaine mesure appariée (steps locaux
 *                           + ranging data pair du même compteur) et émet le
 *                           dump IQ horodaté (IQL/IQP). Retourne 0 si une
 *                           mesure a été émise, < 0 sinon. *rearmed passe à
 *                           false si le beacon doit être ré-armé (chemin
 *                           on-demand uniquement). AUCUN calcul de distance
 *                           (fait hors carte).
 * La collecte ne consomme pas d'airtime : les procédures des N liens tournent
 * en fond, la cadence par beacon = procedure_interval × intervalle de
 * connexion (mesures non collectées à temps : écrasées, pas accumulées). */
/* Dump IQ horodaté vers l'UART data (ON par défaut, désactivable par commande
 * UART IQOFF/réactivable IQON, format des lignes documenté dans IQ_DUMP.md). */
extern bool cs_iq_dump;

int cs_enable_beacon(int i);
int cs_collect_beacon(int i, bool *rearmed);

#endif /* CS_RANGING_H */
