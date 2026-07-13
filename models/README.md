# Models — 3D CAD & print files

Physical design files for the Artemis vehicle. **The binaries here are kept
local only** (gitignored) — they are large and their source of truth is the
team's CAD tool, not git. Only this README is tracked so the folder and its
layout are documented.

## Layout

```
models/
├── cad-source/          # Editable CAD: *.step / *.STEP, *.sldprt
├── print/               # Print-ready meshes + slicer output: *.stl, *.3mf
└── reference/
    └── dimensions/      # Measurement screenshots backing the real dimensions
```

- `cad-source/` — the authoritative geometry (SolidWorks part + STEP exports):
  `Improved frame`, `body*`, `legs`, `New assembly test`.
- `print/` — meshes and slicer files sent to the printer, incl. the
  `*_print` variants and `body_lid_print.gcode.3mf`.
- `reference/dimensions/` — screenshots of measured dimensions; see the
  `project_robot_dimensions` note and `docs/` for the written measurements.

To share these with the team, sync through the CAD tool / drive rather than
committing them.
