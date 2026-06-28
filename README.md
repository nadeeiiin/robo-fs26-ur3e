# manipulator_lab – Pick and Place mit dem UR3e

Pick-and-Place für den UR3e (mit Robotiq-Greifer) in RViz-Simulation und am echten Roboter. Aus den Pick-/Place-Koordinaten werden über eine eigene Vorwärts- und Rückwärtskinematik alle Wegpunkte berechnet.

Dieses Repo enthält nur die `src`-Dateien des Pakets (der Ordner `~/catkin_ws/src/manipulator_lab/src`), da nur diese geändert wurden.

## Verwendung

Simulation:
```bash
rosrun manipulator_lab pick_place_simulation.py \
  --pick_x 0.32 --pick_y 0.12 --place_x 0.32 --place_y -0.12 \
  --height 0.04 --width 32
```

Echter Roboter:
```bash
rosrun manipulator_lab main.py \
  --pick_x 0.32 --pick_y 0.12 --place_x 0.32 --place_y -0.12 \
  --height 0.04 --width 32
```

Argumente: `pick_x/y`, `place_x/y` in m, `height` in m, `width` Greiferöffnung in mm (0–85).
