# dev_status_manager — on-target test app

Self-contained. Builds against the main partition table (rev 2.1). Run:

```powershell
.\test.ps1 target dev_status_manager
```

## What is covered

Init capturing the running partition/app version, set/clear/query, all-set vs
any-set semantics, the network mask, a **cross-task waiter** (second task sets
ETH after 300 ms and wakes `wait_any`), the wait-timeout path, bit names,
uptime formatting, clear-all.

## Expected result — serial markers, in this order

```
INIT ok=1 partition=ota_0 version_set=1
SET sta=1 time=1 mqtt=0
ALLSET both=1 with_mqtt=0
NETMASK connected=1
CLEAR sta=0 net=0
WAIT eth=1
TIMEOUT mqtt=0
NAME b2=sta_connected b17=eth_connected unknown=unknown
UPTIME ok=1
CLEARALL bits=0x000000
TEST DONE
```

(`partition=` may read `ota_1` after an OTA switch.) Last verified: 2026-07-03
on WiCAN Pro.
