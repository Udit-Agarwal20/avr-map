# AVR Console — Frontend (Milestone F1)

Adaptive Variable-Resolution LiDAR Perception Console frontend foundation.

## Technology Stack

- **Framework**: React 18+ (Vite)
- **Language**: TypeScript (Strict)
- **Styling**: CSS Modules, Design Tokens (OKLCH + Hex Fallbacks), Technical Instrumentation Aesthetic
- **State**: Zustand (UI state only; perception frame data handled via data source boundary)
- **Routing**: React Router DOM (`/mission`, `/compare`, `/architecture`, `/diagnostics`, `/config`)

## Available Scripts

In the `frontend/` directory:

```bash
# Install dependencies
npm install

# Start development server
npm run dev

# Run TypeScript typechecker
npm run typecheck

# Run ESLint
npm run lint

# Build production bundle
npm run build
```

## Presentation & Demo Mode

The console includes a deterministic presentation mode for SIH PPT / demo capture:

- **URL Trigger**: Append `?demo=ppt` to the mission route (e.g. `http://localhost:5173/mission?demo=ppt`)
- **Keyboard Trigger**: Press `P` on any view to toggle Presentation Mode.
- Automatically selects **Scenario D: Complex Mixed Intersection**, sets deterministic scene parameters, expands label contrast, and ensures 1920×1080 composition stability.

## Data Truth & Integrity Rules

- **Status**: The console is currently driven by deterministic fixtures (`FixturePerceptionSource`). Top status displays `SIMULATED DEMO` and `DETERMINISTIC FIXTURE`.
- **Playback Cadence**: Demo playback is labeled `DEMO PLAYBACK: 10 Hz` (UI playback cadence, not real-time backend hardware measurement).
- **Cluster Classification**: Objects are labeled as frame-local candidate clusters (`[C-00..C-xx]`), strictly without temporal tracks, velocity, or Kalman filter predictions.
- **Topology Comparisons**: Cell comparisons are labeled `DEMO TOPOLOGY CELL COUNT`, comparing active adaptive cells against an illustrative uniform 5 cm reference grid over the active bounds.
