# cmdline_manager host tests

Covers the PURE byte-stream→line assembler (`cmdline_manager_line.c`) —
the piece every transport's chunking runs through: whole lines, CRLF
(no bogus empty line from the `\n` tail), lines split across chunks,
several lines batched in one chunk, bare-newline skipping, and
oversize-line discard with clean recovery.

Run (on the bench Pi or any Linux with the IDF `linux` target):

    idf.py --preview set-target linux && idf.py build
    ./build/cmdline_manager_host_test.elf

Expected passing output ends with:

    6 Tests 0 Failures 0 Ignored
    OK
