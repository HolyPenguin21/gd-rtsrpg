# AGENTS.md — Fantasy RTS/RPG

Read this file before changing any source or Unity scene. This repository is a Unity RTS/RPG prototype with a **strict stage-by-stage scope** and an explicit architecture contract.

## Source of truth

1. `Docs/Architecture.md` — mandatory architecture contract, **actual modules vs planned modules**.
2. `Docs/Decisions.md` — approved design/technical decisions.
3. `Docs/GDD.md` — game design, gameplay limits.
4. `Docs/ImplementationPlan.md` — current/next stages (not a blanket implementation request).
5. `Docs/WorkingAgreement.md` — branch, PR, QA and approval process.

Before implementing anything, inspect the current repository and the real affected code. Never assume a future module already exists because it appears in the roadmap.

## Current baseline (Stage 01 merged; Stage 02 pending review)

- Unity **6000.5.4f1**, URP **17.5.0**.
- The only scene is `Assets/Scenes/SampleScene.unity`.
- `Assets/Camera/RtsCameraController.cs` owns the camera; `CursorWorldProjection.cs` owns stateless mathematical plane picking. `Assets/TutorialInfo` remains Unity template support.
- Stage 01 was accepted and merged in PR #3. Its camera/Input and plane projection are preserved.
- `Assets/Terrain/` owns Stage 02: TerrainMapData (authored input/validation), TerrainSurface (immutable 4x4 geometry), TerrainBuilder (single mesh and lifetime). SampleScene references DemoTerrain; old Plane and cursor marker are disabled. Stage 02 awaits review; Unity checks are NOT RUN here.

## Mandatory rules

- Implement **only the currently assigned Issue/Stage**. Read dependencies and inspect existing implementation first.
- **Do not** create duplicate navigation, terrain, input, selection, orders, combat or other parallel systems.
- **Do not** create manager/service/framework abstractions or empty module directories for anticipated future stages.
- **Do not** introduce ECS, Jobs/Burst, Unity NavMesh or new packages without an explicit, justified and approved architectural change.
- Navigation will be custom: 4×4 cells per terrain block, 8-neighbor A*, no diagonal corner cutting, size-aware footprints. Unit world movement is continuous.
- Terrain has one surface per XZ coordinate, ramps between discrete elevations, and no runtime deformation.
- Ranged attacks use combat range checks; projectile visuals are cosmetic; **no friendly fire**, allied units do not obstruct friendly shooting.
- Keep scene references and Unity `.meta`/GUIDs intact.
- Update `Docs/Architecture.md` when **real** module responsibilities or dependencies change; do not present intended systems as implemented.
- **No automated tests (Unit/Integration/PlayMode), fixtures, or test infrastructure** at this stage. Tests only after separate owner approval.
- Perform manual Unity scene, compile and Play Mode smoke checks where possible; mark anything unavailable as **NOT RUN**. Do not claim unexecuted checks passed.
- Use a separate branch and PR. Merge to `main` only after project-manager/owner acceptance.

## Expected implementation report

Provide: changed systems and files; why those changes are within scope; dependencies/ownership; actual manual checks; verification limits; known risks; follow-up work explicitly excluded. Review related scene/config/asset references as well as code logic.

## Scope reminder

Stage 02 = explicit terrain data, static mesh, cliffs, variable-width ramps and static geometric boundary information. No procedural generation, terrain picking, unit selection, A*, movement, combat, RPG, save systems or RTS base economy until their own stages.
