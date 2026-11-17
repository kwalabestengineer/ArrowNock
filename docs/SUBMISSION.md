# Submission draft

Review these answers before copying them into Labelbox. This document is preparation only; no submission has been made.

- GitHub profile: https://github.com/kwalabestengineer
- Project name: ArrowNock
- Repository: https://github.com/kwalabestengineer/ArrowNock

## Short description

ArrowNock is embedded C firmware for a motion-sensing arrow nock using a CH32V005D6U6 microcontroller and an ISM330DHCX six-axis inertial sensor. The current baseline reads motion data, handles sensor interrupts, and provides LED and debug diagnostics. I plan to extend it with arrow-motion pattern recognition and calibration.

Rewrite this in your own words, as requested by the program.

## Ownership

The owner has confirmed independent development, personal ownership as a self-employed project, and the right to publish and license the original code. Select sole ownership only if nobody else owns any portion of the original work. Self-employment alone does not answer whether a company or other legal entity owns it; answer that separate field accurately.

## License disclosure

MIT for original contributions owned by kwalabestengineer. Included WCH/vendor-derived portions retain their original notices and terms; see THIRD_PARTY_NOTICES.md. Ask the program how to select the license field for this combination before representing the whole repository as MIT.

## Additional context to review

The IDE project retains the name LiveArrow. The public history currently has two commits; describe earlier development accurately if any exists, and preserve any original history. An engineering disclosure contains a confidentiality marking and references an agreement; the owner has stated they have publication and licensing rights. The proposed roadmap estimates 28 hours, but actual remaining hours must be confirmed by the owner. Specialized hardware is needed for bench validation.

## Remaining questions for the owner

- Does a company or other entity own any code, even if you own that entity?
- Do other people actively work on the repository?
- Are you willing to participate in the optional paired-work format?
- Is the intended recognition algorithm deterministic or based on a trained AI model?
- Does the proposed roadmap accurately represent at least 20 hours of remaining work?

## Question to send to the program yourself

My personally owned embedded firmware project includes WCH MCU support code with vendor-specific notices alongside my MIT-licensed original contributions. Target-board testing requires CH32V005 hardware and an ISM330DHCX sensor. Is this dependency/license combination acceptable, and are there special requirements for hardware-dependent submissions?
