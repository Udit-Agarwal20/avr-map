import { create } from 'zustand';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { ScenarioId } from '../types/perception';

export type ViewMode = 'OVERVIEW' | 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS' | 'RAW_LIDAR';
export type CameraMode = 'PERSPECTIVE' | 'TOP_DOWN';

export interface LayerVisibility {
  sceneContext: boolean;
  lidarPoints: boolean;
  adaptiveCells: boolean;
  dynamicPoints: boolean;
  clusterBounds: boolean;
  objectLabels: boolean;
}

interface ConsoleState {
  commandOpen: boolean;
  setCommandOpen: (open: boolean) => void;
  cameraRevision: number;
  resetCamera: () => void;
  currentScenarioId: ScenarioId;
  activeViewMode: ViewMode;
  cameraMode: CameraMode;
  isPresentationMode: boolean;
  isInspectorOpen: boolean;
  isLayerDrawerOpen: boolean;
  selectedCellId: string | null;
  selectedClusterId: number | null;
  layers: LayerVisibility;

  setScenario: (id: ScenarioId) => void;
  togglePresentationMode: () => void;
  setPresentationMode: (active: boolean) => void;
  setViewMode: (mode: ViewMode) => void;
  setCameraMode: (mode: CameraMode) => void;
  toggleInspector: () => void;
  toggleLayerDrawer: () => void;
  setLayerVisible: (layer: keyof LayerVisibility, visible: boolean) => void;
  toggleLayer: (layer: keyof LayerVisibility) => void;
  setSelectedCellId: (id: string | null) => void;
  setSelectedClusterId: (id: number | null) => void;
  clearSelection: () => void;
}

export const useConsoleStore = create<ConsoleState>((set, get) => ({
  commandOpen: false,
  setCommandOpen: (commandOpen) => set({commandOpen}),
  cameraRevision: 0,
  resetCamera: () => set(state => ({cameraRevision: state.cameraRevision + 1})),
  currentScenarioId: 'D',
  activeViewMode: 'OVERVIEW',
  cameraMode: 'PERSPECTIVE',
  isPresentationMode: false,
  isInspectorOpen: false,
  isLayerDrawerOpen: false,
  selectedCellId: null,
  selectedClusterId: null,
  layers: {
    sceneContext: true,
    lidarPoints: true,
    adaptiveCells: true,
    dynamicPoints: true,
    clusterBounds: true,
    objectLabels: true,
  },

  setScenario: (id: ScenarioId) => {
    if (get().currentScenarioId === id) return;
    frameRuntime.setScenario(id);
    set({ currentScenarioId: id, selectedCellId: null, selectedClusterId: null, isInspectorOpen: false });
  },

  togglePresentationMode: () => {
    const next = !get().isPresentationMode;
    document.body.setAttribute('data-presentation', next ? 'true' : 'false');
    set({ isPresentationMode: next });
  },

  setPresentationMode: (active: boolean) => {
    document.body.setAttribute('data-presentation', active ? 'true' : 'false');
    set({ isPresentationMode: active });
  },

  setViewMode: (mode: ViewMode) => {
    set({ activeViewMode: mode });
  },

  setCameraMode: (mode: CameraMode) => {
    set({ cameraMode: mode });
  },

  toggleInspector: () => {
    set((state) => ({ isInspectorOpen: !state.isInspectorOpen }));
  },

  toggleLayerDrawer: () => {
    set((state) => ({ isLayerDrawerOpen: !state.isLayerDrawerOpen }));
  },

  setLayerVisible: (layer: keyof LayerVisibility, visible: boolean) => {
    set((state) => ({
      layers: { ...state.layers, [layer]: visible },
    }));
  },

  toggleLayer: (layer: keyof LayerVisibility) => {
    set((state) => ({
      layers: { ...state.layers, [layer]: !state.layers[layer] },
    }));
  },

  setSelectedCellId: (id: string | null) => {
    if (id !== null) {
      set({ selectedCellId: id, selectedClusterId: null, isInspectorOpen: true });
    } else {
      set((state) => ({
        selectedCellId: null,
        isInspectorOpen: state.selectedClusterId !== null,
      }));
    }
  },

  setSelectedClusterId: (id: number | null) => {
    if (id !== null) {
      set({ selectedClusterId: id, selectedCellId: null, isInspectorOpen: true });
    } else {
      set((state) => ({
        selectedClusterId: null,
        isInspectorOpen: state.selectedCellId !== null,
      }));
    }
  },

  clearSelection: () => {
    set({ selectedCellId: null, selectedClusterId: null, isInspectorOpen: false });
  },
}));
