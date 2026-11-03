# Bot wiring guide (build one bot fully, then repeat)

**Rule for today: cells stay OUT of the holder until every wire below is done and double-checked.**
The L298N has no reverse-polarity protection, and you have no multimeter, so we check by eye.

## Pin map (fixed)

| D1 Mini pin | Goes to                          | Notes                                |
|-------------|----------------------------------|--------------------------------------|
| D1          | L298N **IN1**                    | OUT1/2, physical right motor          |
| D2          | L298N **IN2**                    | OUT1/2, physical right motor          |
| D5          | L298N **IN3**                    | OUT3/4, physical left motor           |
| D6          | L298N **IN4**                    | OUT3/4, physical left motor           |
| D7          | 1.5 kOhm resistor -> NPN **base**| marker switch                        |
| 5V          | L298N **+5V** terminal           | L298N powers the D1 Mini             |
| G           | L298N **GND** terminal           | common ground                        |
| D3, D4, D8  | **nothing**                      | boot pins, leave empty               |

L298N header (left to right): ENA, IN1, IN2, IN3, IN4, ENB. Wire only the four middle pins. **Leave the ENA and
ENB jumpers fitted.** Both recorded builds: right motor on OUT1/OUT2, left motor on OUT3/OUT4. `SWAP_SIDES=true`.
Direction is fixed in software,
so don't worry which wire goes where.

## Power

```
cell pack (+, red) ---[slide switch]---> L298N  +12V terminal
cell pack (-, black) ------------------> L298N  GND terminal
L298N +5V terminal ---> D1 Mini 5V
L298N GND terminal ---> D1 Mini G   (and transistor emitter, see below)
```

### While the D1 Mini is on USB (all bench testing)

**Do NOT connect the L298N +5V terminal to the D1 Mini 5V pin while USB is plugged in**: two 5V sources would be
tied together. For bench tests connect only GND (L298N GND to D1 Mini G) and the four signal wires; the D1 Mini is
powered from USB and the L298N (logic and motors) from the cells. Add the +5V link only for the final untethered bot,
with USB unplugged.

## Marker (LED stand-in; the laser module later goes in the same place)

```
L298N +5V ---[220-330 ohm]---> LED anode (long leg)
LED cathode (short leg) -----> transistor COLLECTOR
transistor EMITTER ----------> GND
D7 ---[1.5 kOhm]-------------> transistor BASE
```

Transistor legs: read the marking on its flat face.
- **BC547**, flat face toward you, legs down: left to right = **C, B, E**
- **2N2222 (plastic TO-92)**, same view: left to right = **E, B, C**
(Some 2N2222 variants differ; tell me the exact text printed on yours before wiring.)

When the real laser arrives: laser **+** replaces the LED+resistor branch at 5V, laser **-** goes to the collector.
No series resistor if the module has its own; if it's a bare diode, ask me first.

## Steps

1. Assemble the chassis: motors, wheels, caster. Leave the 4xAA box unused.
2. Fix the L298N, D1 Mini and cell holder with foam tape / zip ties. Keep wires short and away from the wheels.
3. Motor wires into the OUT screw terminals (tighten, then tug-test each wire).
4. Four signal jumpers D1/D2/D5/D6 to IN1-IN4.
5. Transistor + resistor + LED marker circuit on D7 (a small breadboard or twisted/soldered joints; no bare legs touching).
6. Ground and 5V links between L298N and D1 Mini.
7. Power wires: red pack lead through the slide switch to +12V, black to GND. Switch OFF.
8. Photo of the whole bot from above and close-ups of the L298N and D1 Mini wiring: send them to me to check
   **before** any cell goes in.

## Cells (no charger yet)

Treat the two ICR-18650 cells as unprotected: never short, reverse or over-discharge them. Don't put them in
until we flash firmware (needs the USB cable). Don't charge without a proper lithium charger.
