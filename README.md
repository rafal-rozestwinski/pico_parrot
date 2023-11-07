# pico_parrot

Baofeng Parrot Simplex Repeater

- Listens for transmission
- If audio level is high, it records audio for 10s
- After 10s, plays beep
- then plays recorded audio
- Then plays beep.

Based on https://github.com/rgrosset/pico-pwm-audio

Refer to it on how to generate own sample.h header with a custom beep (you may want to put your callsign there).

## Board

![front](/img/board_photo.jpeg)

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