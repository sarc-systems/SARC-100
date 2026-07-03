# SARC-100

Patch-programmable analog field computer — firmware and DSP for Bela.

## Layout
- `lib/`      — reusable, Bela-agnostic DSP + helpers (shared core; the library all modules build on)
- `modules/`  — one subdirectory per module (servo, phase, resonator, mixer, tine, ...)
- `hardware/` — panel fabrication files (Lightburn projects, cut exports), one subdirectory per module
- `test/`     — desktop test harness for the portable core
- `docs/`     — design notes and the module index

Dependency rule: modules depend on `lib/`; `lib/` never depends on a module.
