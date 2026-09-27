import { ResolutionLevel, ResolutionReason } from '../types/perception';

export function resolutionMeters(level: ResolutionLevel): number {
  switch (level) {
    case 0:
      return 0.05;
    case 1:
      return 0.10;
    case 2:
      return 0.20;
    case 3:
      return 0.40;
  }
}

export function resolutionLabel(level: ResolutionLevel): string {
  switch (level) {
    case 0:
      return 'L0 (5 cm)';
    case 1:
      return 'L1 (10 cm)';
    case 2:
      return 'L2 (20 cm)';
    case 3:
      return 'L3 (40 cm)';
  }
}

export function resolutionColor(level: ResolutionLevel): string {
  switch (level) {
    case 0:
      return '#62D3C6'; // Turquoise
    case 1:
      return '#70B8E8'; // Blue
    case 2:
      return '#D6B45F'; // Amber / Sand
    case 3:
      return '#A18AC8'; // Lavender
  }
}

export function resolutionReasonLabel(reason: ResolutionReason): string {
  switch (reason) {
    case 'DISTANCE_BASELINE':
      return 'Distance Baseline';
    case 'SEMANTIC_RELEVANCE':
      return 'Semantic Relevance';
    case 'DYNAMIC_RELEVANCE':
      return 'Dynamic Relevance';
    case 'UNCERTAINTY':
      return 'Uncertainty';
    case 'GEOMETRY_COMPLEXITY':
      return 'Geometry Complexity';
    case 'INSUFFICIENT_EVIDENCE':
      return 'Insufficient Evidence';
    case 'DENSITY_GATE_BLOCKED':
      return 'Density Gate';
  }
}
