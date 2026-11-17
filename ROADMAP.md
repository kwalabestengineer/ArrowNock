# Arrow-motion recognition and calibration roadmap

The owner plans to extend the firmware to detect actual arrow behavior and improve calibration. The following task breakdown and estimates are proposed, not measured or owner-confirmed hours. Confirm the remaining effort before claiming the program's 20-hour minimum.

This proposed approach uses deterministic signal processing, thresholds, time windows, and a state machine. It does not require training or evaluating an AI model. If machine learning is intended instead, clarify eligibility with the program before submitting: its instructions exclude AI-model training and evaluation work.

## Current baseline

The current main program reads raw accelerometer and gyroscope values, handles sensor wake interrupts, and drives an LED. It does not implement launch detection. Historical documents may discuss other revisions and are not proof of functionality in this source tree.

## Proposed work

| Task | Estimate | Completion evidence |
| --- | ---: | --- |
| Define arrow-motion events, sensor orientation, and labeled acceptance scenarios | 3 h | Written definitions and expected outcomes for rest, handling, and intended arrow events |
| Implement accelerometer/gyro offset calibration and raw-unit conversion | 5 h | Known-orientation and stationary measurements; documented calibration procedure |
| Implement deterministic motion-pattern recognition using time windows and a state machine | 8 h | Documented rules and replayable synthetic traces showing transitions and rejection cases |
| Improve sampling timing, interrupt handling, and sensor-failure reporting | 4 h | Timing measurements, repeated-event checks, and explicit failure behavior |
| Add host-side tests for calibration and event-recognition logic | 4 h | Tests covering offsets, boundaries, noise, saturation, and false-event cases |
| Validate on the target board and document results and limitations | 4 h | Board revision, commit, measurements, observed errors, and reproducible procedure |
| **Proposed total** | **28 h** | **Owner must confirm actual remaining effort** |

## Decisions before implementation

- Define which arrow behaviors are in scope and the required response time.
- Confirm whether shaft rotation and launch detection are required, and the sensor's usable measurement range.
- Confirm that the intended algorithm is deterministic rather than a trained ML model.
- Confirm hardware availability and rights to use any recorded sensor traces.
- Verify the vendor dependency terms with the submission program.

No completed implementation, calibration accuracy, field testing, or detection performance is claimed by this roadmap.
