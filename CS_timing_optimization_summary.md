# Channel Sounding Timing Optimization — Summary

**Goal:** reduce the time of one distance measurement between the initiator (drone) and the
reflectors (beacons), on nRF54L15 with the Nordic SoftDevice Controller (SDC).

## Starting point

Following the requested performance targets, I set up the Channel Sounding configuration in
the initiator code. At first I had many timing problems, and one misunderstanding about the
BLE **peripheral latency** parameter. I thought the peripheral would wake up instantly after
each connection parameter update, but this is not how latency works: latency is the number
of connection events the peripheral is *allowed to skip*. With a high latency value, the
reflector woke up late, so every exchange was blocked while waiting for it. The fix was to
use latency = 0 on links that are actively measuring.

Even after that, a full measurement cycle took about **1.5 s for two beacons**. So I added
timing instrumentation (timestamps around each phase) and split the cycle into three parts:
startup (procedure enable → first radio subevent), measurement (the subevents themselves),
and data fetch (reading the reflector's results over GATT). The result was surprising: the
real measurement takes only ~10 ms. Almost all the time was **waiting**: ~700 ms of startup
before the radio did anything. Changing the CS procedure parameters (steps, channels,
subevent length) had almost no effect on this delay.

## Root cause

After a lot of searching, the problem was not in the code but in the **controller
configuration** (`prj.conf`). When Channel Sounding is enabled, the SDC silently changes the
default of `CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT` to **90 000 µs**: it
reserves 90 ms of timeline per central connection. With two links at a 60 ms connection
interval, this 90 ms grid cannot fit. The scheduler then delays the first CS subevent for a
long time while it looks for a free slot, and it also refuses short connection intervals.
Two other reservations made it worse: the CS event reservation was oversized (16 000 µs for
a real subevent of 4 500 µs) and each ACL link reserved 7 500 µs.

## Fix

I resized the three reservations to the real needs: ACL spacing 90 000 → 15 000 µs, CS
event 16 000 → 5 000 µs, ACL event 7 500 → 3 750 µs. The connection interval could then go
down from 60 ms to 30 ms. Measured result: **startup 700 → ~340 ms**, data fetch
340 → ~150 ms.

The remaining ~340 ms cannot be compressed: it is a fixed number (~11) of connection events
used by the link-layer handshake that starts a CS procedure, plus the controller's internal
margin. But this time is pure waiting — the radio is free — so it can be **hidden**.
`CONFIG_BT_CTLR_SDC_CS_COUNT` allows several CS procedures to be armed at the same time, so
the code re-enables the next procedure of a beacon *before* fetching its previous results.
The startup of the next procedure then runs in the background, during the GATT fetches.
Measured result: **~510 ms per cycle for two beacons** (one fresh distance every ~255 ms),
instead of the initial 1.5 s — about **3× faster**, with stable distances and no aborts.

## Scheduler and UART control

On top of this, a measurement **scheduler** now decides the order of the beacons in each
round. In automatic mode it gives extra measurement slots to the reflectors with the lowest
estimated error (low jitter, no failures), while every connected beacon still keeps at least
one slot per round so it stays observed. The order can also be forced through the **UART**:
the board receives simple text commands on the same UART used for the data output —
`ORDER:0,1,1` imposes a fixed passing order (a relevant beacon may be measured several times
per round), and `AUTO` returns to automatic mode.

| Step | Startup | Fetch | Cycle (2 beacons) |
|---|---|---|---|
| Initial | ~700 ms | 300–460 ms | ~1.5 s |
| Resized reservations, 30 ms interval | ~340 ms | 130–180 ms | ~540 ms |
| + startup hidden behind fetches | ~340 ms (hidden) | 220–255 ms | **~510 ms** |
