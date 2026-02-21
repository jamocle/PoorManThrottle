
# ✅ Test Battery 

| Test # | Test Name                              | What is tested                     | How to perform                                                   | Expected Serial log patterns                                 |
| ------ | -------------------------------------- | ---------------------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------ |
| T01    | MINSTART set valid                     | `M<n>` parsing in-range            | Send `M15`, `M0`, `M100`                                         | `[CMD] M..`, `[CFG] M=..`                                    |
| T02    | MINSTART clamp edge                    | Clamp above max                    | Send `M101`, `M999`                                              | `[CFG] M=100`                                                |
| T03    | KICK set 2-arg valid                   | `K<t>,<ms>` parsing + defaults     | Send `K60,200`                                                   | `[CFG] KICK=60,200 rd=80 max=15`                             |
| T04    | KICK set 4-arg valid                   | Extended `K<t>,<ms>,<rd>,<max>`    | Send `K60,200,100,20`                                            | `[CFG] KICK=60,200 rd=100 max=20`                            |
| T05    | KICK clamp edges                       | Clamp t/ms/rd/max                  | Send `K999,9999,9999,999`                                        | `[CFG] KICK=100,2000 rd=2000 max=100`                        |
| T06    | KICK invalid formats                   | Invalid token counts               | Send `K60`, `K60,`, `K,200`, `K60,200,10`                        | No `[CFG] KICK=` line; BLE should send `ERR:<original>`      |
| T07    | STOP S basic                           | Quick stop ramp                    | From nonzero throttle, send `S`                                  | `[CMD] S`, `[TGT] ... STOP target=0`, ramp ticks, final STOP |
| T08    | STOP B basic                           | Brake ramp profile                 | From nonzero throttle, send `B`                                  | Same as S but longer ramp                                    |
| T09    | Momentum forward basic                 | `F<n>` ramp up                     | From stop, send `F40`                                            | `[TGT] Momentum`, ramp ticks increasing                      |
| T10    | Momentum reverse basic                 | `R<n>` ramp up                     | From stop, send `R25`                                            | Ramp ticks, final REV                                        |
| T11    | Momentum clamp edges                   | Throttle 0 and 100                 | Send `F0`, `F100`, `R0`, `R100`                                  | `0` → stop ramp; `100` → ramps to full                       |
| T12    | Quick ramp forward                     | `FQ<n>` uses quick-ramp            | From stop, send `FQ60`                                           | `[TGT] QuickRamp`, faster ramp ticks                         |
| T13    | Quick ramp reverse                     | `RQ<n>` quick-ramp                 | From stop, send `RQ60`                                           | Quick ramp to REV                                            |
| T14    | Reverse while moving (momentum)        | Stop-first sequencing              | Start `F40`, then send `R30`                                     | Ramp to 0, delay, ramp up REV; `pending=1` during sequence   |
| T15    | Reverse while moving (quick ramp)      | Stop-first quick-ramp sequencing   | Start `FQ60`, then send `RQ30`                                   | Stop ramp, delay, quick ramp up REV                          |
| T16    | KICK activation path                   | Kick triggers properly             | Configure M + K, then low throttle ≤ maxApply                    | `[THR] ... KICK begin`, hold, continuation                   |
| T17    | KICK does NOT trigger                  | Kick suppression                   | Command target > maxApply or not from stop                       | No `KICK begin` reason in logs                               |
| T18    | Forced stop latch via disconnect       | Grace timeout + latch              | Disconnect BLE, wait for grace expiry                            | `[BLE] Disconnected`, countdown, `Grace expired`, stop ramp  |
| T19    | Latch blocks motion while disconnected | Motion blocked when latched        | After T18, attempt motion while still disconnected (if possible) | Motor remains stopped                                        |
| T20    | Reconnect clears latch                 | Latch clears on reconnect + motion | After latch, reconnect then send `F10`                           | Motion allowed; `[THR] ... latch=0`                          |
| T21    | MTU chunking sanity (optional)         | Notify chunking                    | Negotiate small MTU, send `G`                                    | No Serial change; notify arrives intact                      |


