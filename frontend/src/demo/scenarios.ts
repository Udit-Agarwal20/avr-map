import { PerceptionFrame, ScenarioId } from '../types/perception';
import { scenarioA } from './scenarioA_corridor';
import { scenarioB } from './scenarioB_pedestrian';
import { scenarioC } from './scenarioC_vehicle';
import { scenarioD } from './scenarioD_intersection';

export const SCENARIOS: Record<ScenarioId, PerceptionFrame> = {
  A: scenarioA,
  B: scenarioB,
  C: scenarioC,
  D: scenarioD,
};

export function getScenario(id: ScenarioId): PerceptionFrame {
  return SCENARIOS[id] || SCENARIOS.D;
}
