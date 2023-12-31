# pico_parrot

Baofeng Parrot Simplex Repeater by Rafał Rozestwiński, https://github.com/rafal-rozestwinski/pico_parrot

- Listens for transmission,
- If audio level is high, it records audio for 10s,
- After 10s, triggers PTT on baofeng,
- plays beep (from sample.h),
- plays recorded audio,
- plays beep, releases PTT.

Based on https://github.com/rgrosset/pico-pwm-audio

Refer to it on how to generate own sample.h header with a custom beep (you may want to put your callsign there).

## Board

![front](/img/board_photo.jpeg)

![front](/img/boards.jpeg)

## Schematic

[schematic.pdf](/schematic.pdf)

![schematic](/img/schematic.png)

## BoM

Footprints are duplicated - either SMT or THT parts can be used.
All part values are written on the PCB itself.

- 1x Raspberry Pi Pico
- 1x BJT transistor 2n3904
- 2x PCB Audio Jack Socket PJ-307
- 2x 100nF capacitor
- 1x bigger capacitor (10uF+)
- Resitors:
  - 2x 1000R pot, tht, RM-065_Vertical
  - 220
  - 100
  - 2x 1.8k
  - 2x 68k (or 56k... or 92k etc.) 

Optional power supply regulator pads are for AMS1117-5.0V.

## Future Plans

- Detect end of transmission and play back sooner.
- Easy way to change beep: littlefs has been added, need to add USB Mass Storage support, so those can be replaced without recompiling the code.
- Remote control over USB?
- 3D printed case