#include "scheduler.h"
#include "cs_config.h"
#include "cs_ranging.h"

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

/* Paramètres du score de fiabilité (mode AUTO) : EWMA du taux d'échec, dans
 * [0,1]. Plus BAS = plus fiable → priorité pour les slots bonus. */
#define ERR_EWMA_ALPHA   0.2f  /* poids d'une nouvelle observation */
#define ERR_FAIL_PENALTY 1.0f  /* cible EWMA d'un échec */
#define ERR_INIT         0.5f  /* a priori neutre au démarrage */
#define ERR_FLOOR        0.02f /* évite qu'un beacon parfait fige les autres */

/* Slots bonus par round en AUTO, attribués aux meilleurs beacons.
 * 0 depuis le passage en procédures FREE-RUNNING : les mesures de chaque
 * beacon arrivent toutes les procedure_interval quel que soit le round — un
 * slot dupliqué ne mesure pas plus vite le beacon favorisé (cadence plafonnée
 * par le contrôleur), il ne fait qu'allonger le round et JETER des mesures
 * des autres beacons (écrasées avant collecte). En one-shot c'était pire :
 * chaque doublon payait un startup LL complet (~495 ms) à découvert. */
#define SCHED_BONUS_SLOTS 0

enum sched_mode { SCHED_AUTO, SCHED_MANUAL };

static struct {
	struct k_mutex  lock;
	enum sched_mode mode;
	uint8_t         manual_seq[SCHED_ROUND_MAX];
	int             manual_len;
	/* état par beacon : score de fiabilité EWMA (0 = fiable, 1 = échoue) */
	float           err_ewma[CS_MAX_BEACONS];
} S;

void scheduler_init(void)
{
	k_mutex_init(&S.lock);
	S.mode = SCHED_AUTO;
	S.manual_len = 0;
	for (int i = 0; i < CS_MAX_BEACONS; i++) {
		S.err_ewma[i] = ERR_INIT;
	}
}

static bool beacon_ready(int i)
{
	bool r;

	k_mutex_lock(&beacons_mutex, K_FOREVER);
	r = beacons[i].active && beacons[i].cs_ready && beacons[i].conn != NULL;
	k_mutex_unlock(&beacons_mutex);
	return r;
}

void scheduler_report(int i, bool ok)
{
	if (i < 0 || i >= CS_MAX_BEACONS) {
		return;
	}
	k_mutex_lock(&S.lock, K_FOREVER);
	/* EWMA du taux d'échec : un échec tire le score vers ERR_FAIL_PENALTY
	 * (le beacon perd ses slots bonus mais garde son slot de base, il reste
	 * observé) ; un succès le tire vers 0 (candidat aux slots bonus). */
	float target = ok ? 0.0f : ERR_FAIL_PENALTY;

	S.err_ewma[i] += ERR_EWMA_ALPHA * (target - S.err_ewma[i]);
	if (S.err_ewma[i] < ERR_FLOOR) {
		S.err_ewma[i] = ERR_FLOOR;
	}
	k_mutex_unlock(&S.lock);
}

/* Construit le round AUTO : base + bonus aux beacons les moins bruités, puis
 * ENTRELACE (round-robin sur les compteurs de slots) pour éviter deux mesures
 * consécutives du même beacon — deux procédures ne s'empilent pas sur un même
 * lien, un doublon adjacent paierait son startup à découvert. */
static int build_auto(uint8_t seq[SCHED_ROUND_MAX], const bool ready[CS_MAX_BEACONS],
		      int n_ready)
{
	int slots[CS_MAX_BEACONS] = { 0 };

	for (int i = 0; i < CS_MAX_BEACONS; i++) {
		if (ready[i]) {
			slots[i] = CS_SCHED_DEFAULT_REPEAT;
		}
	}

	/* Bonus : un par tour au beacon prêt d'erreur minimale. RÉPARTIS entre
	 * beacons distincts (moins de bonus d'abord, erreur ensuite) : l'ancien
	 * critère erreur seule envoyait TOUS les bonus au même beacon (l'EWMA ne
	 * bouge pas pendant la boucle, égalité → plus petit index), d'où des
	 * rounds [0,1,2,0,0] avec doublon adjacent — exactement ce que
	 * l'entrelacement ci-dessous doit éviter. */
	int bonus[CS_MAX_BEACONS] = { 0 };

	for (int b = 0; b < SCHED_BONUS_SLOTS && n_ready > 1; b++) {
		int   best = -1;
		float best_err = 0.0f;

		for (int i = 0; i < CS_MAX_BEACONS; i++) {
			if (ready[i] &&
			    (best < 0 || bonus[i] < bonus[best] ||
			     (bonus[i] == bonus[best] && S.err_ewma[i] < best_err))) {
				best     = i;
				best_err = S.err_ewma[i];
			}
		}
		if (best >= 0) {
			bonus[best]++;
			slots[best]++;
		}
	}

	int len = 0;
	bool remaining = true;

	while (remaining && len < SCHED_ROUND_MAX) {
		remaining = false;
		for (int i = 0; i < CS_MAX_BEACONS && len < SCHED_ROUND_MAX; i++) {
			if (slots[i] > 0) {
				slots[i]--;
				seq[len++] = i;
				remaining = remaining || (slots[i] > 0);
			}
		}
	}
	return len;
}

int scheduler_build_round(uint8_t seq[SCHED_ROUND_MAX])
{
	bool ready[CS_MAX_BEACONS];
	int  n_ready = 0;
	int  len = 0;

	for (int i = 0; i < CS_MAX_BEACONS; i++) {
		ready[i] = beacon_ready(i);
		n_ready += ready[i] ? 1 : 0;
	}
	if (n_ready == 0) {
		return 0;
	}

	k_mutex_lock(&S.lock, K_FOREVER);
	if (S.mode == SCHED_MANUAL && S.manual_len > 0) {
		for (int k = 0; k < S.manual_len; k++) {
			uint8_t i = S.manual_seq[k];

			if (i < CS_MAX_BEACONS && ready[i]) {
				seq[len++] = i;
			}
		}
		/* Séquence manuelle vidée par les déconnexions : filet AUTO. */
		if (len == 0) {
			len = build_auto(seq, ready, n_ready);
		}
	} else {
		len = build_auto(seq, ready, n_ready);
	}
	k_mutex_unlock(&S.lock);
	return len;
}

void scheduler_uart_line(const char *line)
{
	if (strncmp(line, "AUTO", 4) == 0) {
		k_mutex_lock(&S.lock, K_FOREVER);
		S.mode = SCHED_AUTO;
		k_mutex_unlock(&S.lock);
		LOG_INF("sched: mode AUTO");
		return;
	}

	if (strncmp(line, "ORDER:", 6) == 0) {
		uint8_t tmp[SCHED_ROUND_MAX];
		int     n = 0;
		const char *p = line + 6;

		while (*p != '\0' && n < SCHED_ROUND_MAX) {
			char *end;
			long  v = strtol(p, &end, 10);

			if (end == p) {
				break; /* pas un nombre */
			}
			if (v < 0 || v >= CS_MAX_BEACONS) {
				LOG_WRN("sched: index %ld invalide (0..%d), ligne ignoree",
					v, CS_MAX_BEACONS - 1);
				return;
			}
			tmp[n++] = (uint8_t)v;
			p = (*end == ',') ? end + 1 : end;
		}
		if (n == 0) {
			LOG_WRN("sched: ORDER vide, ligne ignoree");
			return;
		}
		k_mutex_lock(&S.lock, K_FOREVER);
		S.mode       = SCHED_MANUAL;
		S.manual_len = n;
		memcpy(S.manual_seq, tmp, n);
		k_mutex_unlock(&S.lock);
		LOG_INF("sched: mode MANUAL, %d slot(s)/round", n);
		return;
	}

	if (strncmp(line, "IQON", 4) == 0) {
		cs_iq_dump = true;
		LOG_INF("sched: dump IQ ON");
		return;
	}
	if (strncmp(line, "IQOFF", 5) == 0) {
		cs_iq_dump = false;
		LOG_INF("sched: dump IQ OFF");
		return;
	}

	LOG_WRN("sched: commande UART inconnue: %s", line);
}
