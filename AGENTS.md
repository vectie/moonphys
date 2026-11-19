# MoonPhys agent guide

This is the independent MoonBit module `vectie/moonphys`. Package root files own
the physics API; `review/` owns model-metadata review. `moonnum/geometry` owns the
shared geometry types and their methods. Keep that dependency direction; do not
import MoonMoon, MoonRobo, UI packages, simulator bindings, or training backends.

Use `///|` block boundaries and the current `moon.pkg` / `moon.mod` formats. Public
record field names, enum constructors, JSON encodings, labeled arguments, and
default arguments are compatibility contracts. Re-export canonical public types
instead of copying their definitions or algorithms into another module.

Use the MoonBit agent guide for changes and `.mbtx` for authored automation.
`docs/provenance/` holds immutable, noncompiled source snapshots and SHA-256 records;
do not edit them as implementation code. The original extraction/forwarding tool
is `../moonmoon/scripts/extract_moonphys.mbtx`.

Run the relevant `moon check`, then `moon info` and `moon fmt`, and inspect the
generated interface changes. Tests live in `_test.mbt` files. **The current user
instruction defers executing tests, simulation, training, and hardware operations**;
typecheck those test sources without running `moon test` until that is lifted.
