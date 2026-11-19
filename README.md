# MoonPhys

`vectie/moonphys@0.1.0` is a standalone MoonBit module for rigid-body integration,
collision and contact, articulated kinematics, joint control, heightfield worlds,
support assessments, and typed physical review records. It was extracted from
MoonMoon's `src/moonphys` without changing its physical algorithms or JSON fields.
Source lives at [vectie/moonphys](https://github.com/vectie/moonphys); source
publication is separate from a Mooncakes registry release.

Import the root package as `vectie/moonphys`, and the model metadata review package
as `vectie/moonphys/review`. Existing users of
`vectie/moonmoon/src/moonphys` and its `review` subpackage can continue through
MoonMoon's generated forwarding facades.
The enum's short package-qualified constructor spelling requires the type-qualified
form through the facade; see the compatibility note in `MIGRATION.md`.

```moonbit
import {
  "vectie/moonphys" @physics,
}
```

The only module dependency is `vectie/moonnum@0.1.0`. `Vec3`, `Basis3`, and
`Transform3`, their functions, and their methods belong to the public
`vectie/moonnum/geometry` package and are re-exported here. Physical state, contact,
constraint, world, and assessment types belong to this module. The dependency
direction is `MoonMoon -> MoonPhys -> MoonNum`; neither MoonPhys nor MoonNum
depends on MoonMoon, MoonRobo, a UI, a robot SDK, MuJoCo, Python, or LibTorch.

The numerical models retain their original scope: these are explicit portable
models and review calculations, not replacements for MuJoCo or hardware SDKs.
The extraction does not add training, automatic differentiation, tensor kernels,
or an external simulator interface to MoonPhys.

The included local `moon.work` resolves this module and the sibling `../moonnum`.
MoonMoon's workspace additionally includes MoonMoon itself. With those sibling
checkouts present, `moon check --target all` typechecks production code and the
migrated tests without executing them. `moon info` generates the public interfaces
and `moon fmt` formats the source.

On 19 November 2026, the native workspace reports **127 passed, zero failed**,
including the shared MoonNum suite with an explicitly selected Accelerate BLAS
provider. Existing compiler deprecation warnings remain; this is portable
numerical validation, not hardware or external-simulator qualification.
See [MIGRATION.md](MIGRATION.md) for the source mapping, compatibility
contract, verification record, and immutable original-source snapshots. The
license is [Apache-2.0](LICENSE), inherited from MoonMoon.
