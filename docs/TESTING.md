# Build and bench verification

Status: procedure only. These checks have not been executed during the documentation update. Record the date, firmware commit, board revision, tool versions, measured results, and pass/fail outcome when running them.

## Without hardware

- Import and rebuild the existing LiveArrow project with the supported WCH toolchain.
- Confirm the linker uses `Ld/Link.ld`, the target is CH32V005D6U6, and the image fits the configured memory regions.
- Inspect compile warnings and confirm a fresh ELF/HEX is produced rather than relying on committed binaries.

No simulator or automated host test harness is provided in the current repository.

## On the board

| Check | Expected behavior from current source |
| --- | --- |
| Boot | One indicator blink before sensor setup |
| Sensor identity | SDI reports WHO_AM_I `0x6B` |
| Sensor ready | Two indicator blinks after configuration |
| Raw samples | Six-axis raw output approximately every 100 ms, plus processing overhead |
| Stationary orientation | Acceleration changes with gravity orientation; gyro is near zero, subject to offsets |
| Wake interrupt | A suitable bench motion can report INT1 source and toggle the LED; repeated events must still be observed after latch clearing |
| Sensor unavailable | Identity check fails and the indicator blinks continuously |
| Probe detached | Observe indicator behavior with no SDI listener |

Power down before changing wiring. Test sensor absence only with an electrically appropriate setup. Threshold-dependent behavior and timing must be measured; the table is not evidence of successful hardware testing.

## Record template

- Date / operator:
- Firmware commit:
- Board revision / sensor:
- IDE / compiler / probe versions:
- Build output and memory usage:
- Checks run and measured results:
- Failures / follow-up issues:
