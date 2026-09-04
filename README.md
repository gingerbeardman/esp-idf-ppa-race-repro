# PPA cross-core race reproducer (espressif/esp-idf PR #19047)

ESP32-P4 with PSRAM, no display. `idf.py set-target esp32p4 && idf.py build flash monitor`.

A task on core 1 submits single non-blocking PPA SRM copies at the
transaction's own duration ±60 µs so that some submissions land inside the
completion interrupt (which lives on core 0, where the client was
registered). Stock driver: ~130 of 20000 complete out of order and ~3
durations late (stranded until the next submission), and in the pair phase
the second of a pair never completes within 500 ms several times — the
application hang. From core 0 (control): 0 and 0. With the fix: 0 and 0.
