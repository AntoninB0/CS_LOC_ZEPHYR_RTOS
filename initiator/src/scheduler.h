#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>
#include "beacon.h"

/* ── Scheduler de mesures CS ────────────────────────────────────────────────
 * Construit, round par round, la séquence des beacons à mesurer.
 *
 * Mode AUTO (défaut) : chaque beacon prêt reçoit CS_SCHED_DEFAULT_REPEAT
 * slot(s) de base, puis des slots BONUS sont attribués aux beacons dont
 * l'erreur estimée est la plus FAIBLE (jitter EWMA des distances successives
 * + pénalité d'échec). Rationale : une ancre stable porte plus d'information
 * par mesure — on la rafraîchit plus souvent ; l'ancre bruitée reste mesurée
 * (>= 1 slot) pour suivre son état et lui rendre des slots si elle s'améliore.
 *
 * Mode MANUAL : la séquence est imposée par UART (ordre de passage explicite,
 * répétitions permises). Les beacons non prêts sont sautés au déroulé.
 *
 * Protocole UART (uart21, lignes ASCII terminées par '\n') :
 *   "ORDER:0,1,1"  → mode MANUAL, séquence appliquée à chaque round
 *   "AUTO"         → retour au mode AUTO
 * Toute ligne invalide est ignorée (log WRN). */

#define SCHED_ROUND_MAX 16  /* 5 beacons x repeat 2 + bonus 4 = 14 max */

void scheduler_init(void);

/* Construit la séquence du prochain round dans seq[] (indices de beacons,
 * uniquement des beacons prêts). Retourne la longueur (0 si aucun prêt). */
int scheduler_build_round(uint8_t seq[SCHED_ROUND_MAX]);

/* À appeler après chaque mesure : met à jour l'erreur estimée du beacon.
 * ok=false pour un échec (procédure/fetch). */
void scheduler_report(int i, float distance, bool ok);

/* Parse une ligne de commande UART (contexte thread, pas ISR). */
void scheduler_uart_line(const char *line);

#endif /* SCHEDULER_H_ */
