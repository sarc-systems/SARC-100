# hardware

Panel fabrication files — one subdirectory per module, matching `modules/`.

Not synced to the Bela board (deploy only touches `modules/<name>/`), so this is
the right place for Lightburn projects and any exported cut files (SVG/DXF/PDF)
without bloating the device payload.

Suggested naming: `<module>-panel.lbrn2`, with iteration history left to git rather
than filename suffixes (`-v2`, `-final`, etc.).

Physical panel specs (jack/pot/knob hardware, layout) stay documented in each
module's own `CLAUDE.md` (e.g. `modules/empath/CLAUDE.md`'s "Panel hardware"
section) — these files are the cut geometry that implements that spec.
