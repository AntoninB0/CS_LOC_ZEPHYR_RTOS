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
# PINNED initiator SNR (DK label). Without it, "first sorted SNR" = a lottery:
# the initiator firmware could end up on a reflector.
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
            echo "  --boat-1            precision overlay for one beacon (single)"
            echo "  --boat-n            precision overlay for n beacons (multi)"
            echo "  --no-flash          build only"
            echo "  --clean             clean the build dirs first"
            echo "  --reflectors-only   build+flash the reflectors only"
            echo "                      (e.g. change of N or scheduler -> not needed;"
            echo "                       change of FAMILY/CS_EVENT_LEN -> needed)"
            echo "  --initiator-only    build+flash the initiator only"
            echo "                      (common case: profile, N, scheduler, IQ...)"
            echo "  --full-reflector    ALL connected boards become reflectors"
            echo "                      (initiator included) — useful to prepare a"
            echo "                      batch of beacons or requalify the initiator"
            echo "                      board. To revert:"
            echo "                      ./build.sh --initiator-only"
            echo "  --init-snr SNR      initiator SNR (default: $INITIATOR_SNR_PIN)"
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
    echo ">>> FULL-REFLECTOR mode: all boards become reflectors."
    for snr in $SNRS; do
        tag=""
        [ "$snr" = "$INITIATOR_SNR_PIN" ] && tag="  (ex-initiator!)"
        echo ">>> Flashing reflector → SNR $snr$tag"
        $LAUNCH west flash --build-dir "$REFLECTOR_BUILD" --dev-id "$snr"
    done
    echo ">>> Done (full-reflector, $COUNT board(s)). Reminder: NO initiator"
    echo "    left on the bench — './build.sh --initiator-only' to make one again."
    exit 0
fi

# Initiator = pinned SNR if plugged in, otherwise the first SNR (with a warning)
if echo "$SNRS" | grep -q "^${INITIATOR_SNR_PIN}$"; then
    INITIATOR_SNR=$INITIATOR_SNR_PIN
else
    INITIATOR_SNR=$(echo "$SNRS" | head -1)
    echo "WARNING: initiator SNR $INITIATOR_SNR_PIN absent, falling back to $INITIATOR_SNR"
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
