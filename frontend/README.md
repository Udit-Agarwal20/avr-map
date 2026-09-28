# AVR Console

A fixture-driven research console for adaptive 2.5D LiDAR mapping. The five primary screens are Mission, Compare, Benchmarks, Diagnostics and Config. System Architecture is a secondary route available through the command palette.

React 18, strict TypeScript, Vite, plain Three.js, CSS Modules, Zustand (UI state only), Radix dialogs and Phosphor icons. IBM Plex Sans, Sans Condensed and Mono are bundled locally from Fontsource; no external font request is required.

## Run and verify

From `frontend/`:

```sh
npm ci
npm run dev
npm test
npm run typecheck
npm run lint
npm run build
npm run test:browser
```

The browser audit serves `dist/`, requires the Playwright Chromium browser (`npx playwright install chromium` if absent), and writes screenshots and its JSON report to the ignored `.screenshots/` directory. Build before running it. It checks the production routes, keyboard controls, dialog focus, comparison interaction, local config review, accessibility, seven viewport sizes and offline typography.

## Presentation

Open `/mission?demo=ppt`. Add `&view=resolution`, `&view=dynamics`, `&view=semantic` or `&view=raw` for repeatable scene captures. `/compare?demo=ppt` opens the wipe comparison. The presentation preset selects Scenario D and hides navigation.

- `P`: presentation mode.
- `Ctrl/Cmd K`: command palette.
- `L`: layer drawer; `C`: point cloud.
- `1` / `2`: perspective / top-down camera; `R`: reset camera.
- `Escape`: close a dialog/drawer or clear selection.

Shortcuts ignore text inputs and active dialogs. Below 1100 px, Mission becomes a camera-locked monitoring view with a scene snapshot, light metrics, navigation and command palette; full engineering controls are hidden.

## Data boundaries

The source is explicitly `SIMULATED DEMO / STATIC FIXTURE`. There is no playback timer, live sensor connection or measured benchmark stream. Benchmarks therefore shows awaiting-data states. Configuration changes affect local renderer visibility only and require review before applying.

`FixturePerceptionSource` feeds `PerceptionFrameRuntime`; React subscribes to metadata and the renderers read frame arrays directly. `PerceptionFrameSource` and `GatewayFrameDecoder` define the future adapter boundary. No backend transport is implemented.

Semantics describe current-frame observations. Clusters are frame-local and have no temporal identity, velocity or trajectory. Dynamic observations can support refinement, but cannot authorize persistent static elevation fusion. The golden pedestrian cell `D_L0_P_127` contains exactly source indices 201–206, all `DYNAMIC_EXCLUDED`, and has no static elevation payload.

Scenario D uses deterministic illustrative active leaves on an explicit fixture origin, preserving the golden cell. It is not the output of a backend policy run. Compare subdivides precisely those occupied leaf footprints into a uniform 5 cm reference. Its counts are demo topology, not measured savings. No memory, latency, accuracy or preservation results are invented.

The [completion report](COMPLETION_REPORT.md) records the implementation, evidence, screenshots and deviations from the requested specification.
