# PPA cross-core race reproducer (espressif/esp-idf PR #19047)

ESP32-P4 with PSRAM, no display needed.

    idf.py set-target esp32p4 && idf.py build flash monitor

`sdkconfig.defaults` sets the minimum chip revision to v1.0 so v1.x boards flash.

The SRM client is registered from `app_main` on core 0, so the PPA/2D-DMA
interrupt lives there. A task on core 1 then submits single non-blocking
SRM copies (480×40 RGB565, PSRAM to PSRAM) spaced at the transaction's own
measured duration ±60 µs, so that some submissions land inside the
completion interrupt of the previous one. Each carries its index in
`user_data`; the done callback counts completions arriving out of index
order and their latency. A second phase submits pairs and waits for both
with a timeout: if the second of a pair was stranded, nothing follows to
rescue it and the wait times out — the application hang. Both phases then
run from core 0 as the control.

Measured on ESP32-P4 v1.3, ESP-IDF v5.5.5:

| submitter | driver | phase 1: out of order / 20000 | phase 2: hangs / 2000 pairs | max latency |
|---|---|---|---|---|
| core 1 | stock | **195** (all ~3 durations late) | **18** | 15.7 ms |
| core 0 | stock | 0 | 0 | 0.94 ms |
| core 1 | PR #19047 | 0 | 0 | 0.89 ms |
| core 0 | PR #19047 | 0 | 0 | 0.94 ms |

Mechanism: `ppa_do_operation` inserts the transaction under the engine
spinlock and then tries the engine-idle semaphore with a zero timeout;
the completion ISR decides "queue empty" under that spinlock but gives
the semaphore only after unlocking. A submitter on the other core can
slip in between, see the semaphore taken, assume the ISR will chain it,
and be left at the head of an idle engine's queue.
