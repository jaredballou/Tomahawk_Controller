# 2. Functional Design

Date: 2021-12-06

## Status

Accepted

## Context

Basic layout of how the system will work, from hardware and software.

## Decision

### Hardware

- 12v solenoids for feed and pusher
- Narfduino for controller
- Add keypad/button panel to allow minor configuration tweaks without serial/USB.
- Investigate mag loading improvements, i.e. rear loading ports and/or folding front drum cover.

### Software

- Should activate feed cycle on unknown status, and immediately after pusher cycle completes.
- Unknown/empty feed status should prevent pusher cycle until feed is ready.
- 

## Consequences

Hopefully, a somewhat functional automatic foam dart blaster.
