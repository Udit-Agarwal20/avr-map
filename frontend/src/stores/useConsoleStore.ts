import { create } from 'zustand';
import { frameRuntime } from '../runtime/PerceptionFrameRuntime';
import { ScenarioId } from '../types/perception';

export type ViewMode = 'SEMANTIC' | 'RESOLUTION' | 'DYNAMICS';

interface ConsoleState {
  currentScenarioId: ScenarioId;
  activeViewMode: ViewMode;
  isPresentationMode: boolean;
  isInspectorOpen: boolean;
  selectedCellId: string | null;
  selectedClusterId: number | null;

  setScenario: (id: ScenarioId) => void;
  togglePresentationMode: () => void;
  setPresentationMode: (active: boolean) => void;
  setViewMode: (mode: ViewMode) => void;
  toggleInspector: () => void;
  setSelectedCellId: (id: string | null) => void;
  setSelectedClusterId: (id: number | null) => void;
}

export const useConsoleStore = create<ConsoleState>((set, get) => ({
  currentScenarioId: 'D',
  activeViewMode: 'RESOLUTION',
  isPresentationMode: false,
  isInspectorOpen: true,
  selectedCellId: null,
  selectedClusterId: null,

  setScenario: (id: ScenarioId) => {
    frameRuntime.setScenario(id);
    set({ currentScenarioId: id, selectedCellId: null, selectedClusterId: null });
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

  toggleInspector: () => {
    set((state) => ({ isInspectorOpen: !state.isInspectorOpen }));
  },

  setSelectedCellId: (id: string | null) => {
    set({ selectedCellId: id });
  },

  setSelectedClusterId: (id: number | null) => {
    set({ selectedClusterId: id });
  },
}));
