# RoboticsScienceAndSystems

Latest grbl files for the PCB manufacturing are located in /pcb/RSS_StampC3Extension/pcbway_production/

## Development Notes

**13 April 2026:** Had a few hardware issues.  I failed to notice that a connection between the phototransistors and the GPIO labels was not made in the schematic (appearing as a box, not a dot) - so the PCB arrived without these connections.  It also turns out that GPIO 5 cannot be used for ADC when using WiFi on the ESP32-C3, so not having the connections on the PCB turned out to help with debugging that.  I updated the board design for GPIO changes, and also added a decoupling capacitor just in case this turns out to be necessary.  I currently have the IR LEDs drawing from the 3v3 supply, and I am a little nervous about the available current.  The schematic for the StampC3 shows a MUN3CAD01-SC DCDC converter. The datasheet says it can supply 1A current.  Apparently the ESP32-C3 could peak at 200ma consumption on a wireless transmit - so it seems all in range.  I've tested it all on the Pololu 3Pi+ robot and it seemed fine.

**24 March 2026:** The M5_Hardware kicad repository had a footprint for the StampC3 but not a symbol.  I duplicated the symbol for the S3, saved it as C3, and changed the overall symbol layout to reflect the physical layout.  The M5_Hardware repository is included in this repository with the original attributions and license.
