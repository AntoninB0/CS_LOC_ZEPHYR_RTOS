#!/bin/bash
set -e

NRFUTIL=/home/antoninbo/ncs/toolchains/911f4c5c26/nrfutil/bin/nrfutil
NCS_WORKSPACE=/home/antoninbo/ncs/v3.3.0
BOARD=nrf54l15dk/nrf54l15/cpuapp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INITIATOR_DIR=$SCRIPT_DIR/initiator
REFLECTOR_DIR=$SCRIPT_DIR/reflector
INITIATOR_BUILD=$INITIATOR_DIR/build
REFLECTOR_BUILD=$REFLECTOR_DIR/build

FLASH=1
CLEAN=0
precision=0
TARGET=all            # all | initiator | reflectors | full-reflector
# SNR de l'initiateur FIGÉ (étiquette du DK). Sans ça, "premier SNR trié" =
# loterie : le firmware initiateur peut partir sur un réflecteur.
INITIATOR_SNR_PIN=1057702942

prev=""
for arg in "$@"; do
    case $arg in
        --boat-1)          precision=1 ;;
        --boat-n)          precision=2 ;;
        --no-flash)        FLASH=0 ;;
        --clean)           CLEAN=1 ;;
        --reflectors-only) TARGET=reflectors ;;
        --initiator-only)  TARGET=initiator ;;
        --full-reflector)  TARGET=full-reflector ;;
        --init-snr)        prev=init_snr ; continue ;;
        --help)
            echo "Usage: $0 [--boat-1] [--boat-n] [--no-flash] [--clean] [--reflectors-only|--initiator-only] [--init-snr SNR]"
            echo "  --boat-1            précision appliquée pour un beacon"
            echo "  --boat-n            précision appliquée pour n beacon"
            echo "  --no-flash          build seulement"
            echo "  --clean             nettoie les build dirs avant"
            echo "  --reflectors-only   build+flash uniquement les réflecteurs"
            echo "                      (ex: changement de N ou de scheduler → inutile ;"
            echo "                       changement de FAMILLE/CS_EVENT_LEN → nécessaire)"
            echo "  --initiator-only    build+flash uniquement l'initiateur"
            echo "                      (cas courant : profil, N, scheduler, IQ...)"
            echo "  --full-reflector    TOUTES les cartes connectées deviennent des"
            echo "                      réflecteurs (initiateur inclus) — utile pour"
            echo "                      préparer un lot de beacons ou requalifier la"
            echo "                      carte initiateur. Pour revenir :"
            echo "                      ./build.sh --initiator-only"
            echo "  --init-snr SNR      SNR de l'initiateur (défaut: $INITIATOR_SNR_PIN)"
            exit 0 ;;
        *)
            if [ "$prev" = "init_snr" ]; then INITIATOR_SNR_PIN=$arg; prev=""; fi ;;
    esac
done

LAUNCH="$NRFUTIL toolchain-manager launch --ncs-version v3.3.0 --"

# ── Detect connected J-Link serial numbers ────────────────────────────────
detect_snrs() {
    lsusb -v -d 1366:1069 2>/dev/null \
        | grep "iSerial" \
        | awk '{print $3}' \
        | sed 's/^0*//' \
        | sort -u
}

# ── Build ─────────────────────────────────────────────────────────────────
cd "$NCS_WORKSPACE"

if [ "$CLEAN" -eq 1 ]; then
    echo ">>> Cleaning build directories..."
    rm -rf "$INITIATOR_BUILD" "$REFLECTOR_BUILD"
fi

if [ "$TARGET" != "reflectors" ] && [ "$TARGET" != "full-reflector" ]; then
    echo ">>> Building initiator..."
    case $precision in
        1) $LAUNCH west build -b "$BOARD" "$INITIATOR_DIR" --build-dir "$INITIATOR_BUILD" --pristine=auto -- -DEXTRA_CONF_FILE=overlay-precision-single.conf;;
        2) $LAUNCH west build -b "$BOARD" "$INITIATOR_DIR" --build-dir "$INITIATOR_BUILD" --pristine=auto -- -DEXTRA_CONF_FILE=overlay-precision-multi.conf;;
        0) $LAUNCH west build -b "$BOARD" "$INITIATOR_DIR" --build-dir "$INITIATOR_BUILD" --pristine=auto;;
    esac
fi

# Force clean reflector build if app CMakeCache points to a different source path
if [ -f "$REFLECTOR_BUILD/reflector/CMakeCache.txt" ]; then
    CACHED_SRC=$(grep "^CMAKE_HOME_DIRECTORY" "$REFLECTOR_BUILD/reflector/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
    if [ "$CACHED_SRC" != "$REFLECTOR_DIR" ]; then
        echo ">>> Stale reflector build dir (was: $CACHED_SRC), cleaning..."
        rm -rf "$REFLECTOR_BUILD"
    fi
fi

if [ "$TARGET" != "initiator" ]; then
    echo ">>> Building reflector (beacon)..."
    $LAUNCH west build -b "$BOARD" "$REFLECTOR_DIR" --build-dir "$REFLECTOR_BUILD" --pristine=auto
fi

# ── Flash ─────────────────────────────────────────────────────────────────
if [ "$FLASH" -eq 0 ]; then
    echo ">>> Build complete (--no-flash)."
    exit 0
fi

echo ">>> Detecting connected boards..."
SNRS=$(detect_snrs)
COUNT=$(echo "$SNRS" | wc -l)

if [ -z "$SNRS" ]; then
    echo "ERROR: No J-Link board detected. Check USB connections."
    exit 1
fi

echo "    Found $COUNT board(s):"
i=1
for snr in $SNRS; do
    echo "      [$i] SNR $snr"
    i=$((i+1))
done

if [ "$TARGET" = "full-reflector" ]; then
    echo ""
    echo ">>> Mode FULL-REFLECTOR : toutes les cartes deviennent des réflecteurs."
    for snr in $SNRS; do
        tag=""
        [ "$snr" = "$INITIATOR_SNR_PIN" ] && tag="  (ex-initiateur !)"
        echo ">>> Flashing reflector → SNR $snr$tag"
        $LAUNCH west flash --build-dir "$REFLECTOR_BUILD" --dev-id "$snr"
    done
    echo ">>> Done (full-reflector, $COUNT carte(s)). Rappel : plus AUCUN"
    echo "    initiateur sur le banc — './build.sh --initiator-only' pour en refaire un."
    exit 0
fi

# Initiateur = SNR figé s'il est branché, sinon premier SNR (avec warning)
if echo "$SNRS" | grep -q "^${INITIATOR_SNR_PIN}$"; then
    INITIATOR_SNR=$INITIATOR_SNR_PIN
else
    INITIATOR_SNR=$(echo "$SNRS" | head -1)
    echo "WARNING: SNR initiateur $INITIATOR_SNR_PIN absent, repli sur $INITIATOR_SNR"
fi
BEACON_SNRS=$(echo "$SNRS" | grep -v "^${INITIATOR_SNR}$" || true)

echo ""
if [ "$TARGET" != "reflectors" ]; then
    echo ">>> Flashing initiator → SNR $INITIATOR_SNR"
    $LAUNCH west flash --build-dir "$INITIATOR_BUILD" --dev-id "$INITIATOR_SNR"
fi

if [ "$TARGET" != "initiator" ]; then
    if [ -z "$BEACON_SNRS" ]; then
        echo "WARNING: No additional boards found for beacon flashing."
    else
        for snr in $BEACON_SNRS; do
            echo ">>> Flashing beacon → SNR $snr"
            $LAUNCH west flash --build-dir "$REFLECTOR_BUILD" --dev-id "$snr"
        done
    fi
fi

echo ">>> Done ($TARGET). Initiator: SNR $INITIATOR_SNR | Beacons: $(echo $BEACON_SNRS | tr '\n' ' ')"
