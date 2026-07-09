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

/* PIPELINE (recouvrement maximal du startup, pure attente de scheduling) :
 *  - cs_enable_beacon(i)    : lance la procédure CS du beacon i SANS attendre.
 *                             Retourne 0 si l'enable est accepté (< 0 sinon).
 *  - cs_wait_done_beacon(i) : attend la fin de la procédure et snapshotte les
 *                             steps locaux. Retourne 0 si données valides.
 *  - cs_fetch_beacon(i)     : récupère le RAS du pair pour le snapshot et
 *                             calcule la distance (m), ou < 0 si échec.
 * Usage optimal par beacon : wait_done → RÉ-enable immédiat → fetch.
 * Le startup de la procédure N+1 recouvre alors le fetch de N (et ceux des
 * autres beacons).
 *  - cs_collect_beacon(i)   : compat, wait_done + fetch enchaînés. */
/* Dump IQ horodaté vers l'UART data (activable par commande UART IQON/IQOFF,
 * format des lignes documenté dans IQ_DUMP.md). */
extern bool cs_iq_dump;

int   cs_enable_beacon(int i);
int   cs_wait_done_beacon(int i);
float cs_fetch_beacon(int i);
float cs_collect_beacon(int i);

#endif /* CS_RANGING_H */
