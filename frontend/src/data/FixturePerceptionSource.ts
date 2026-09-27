import {
  validateBackendStatusContract,
  validatePerceptionFrame,
} from '../demo/fixtureValidation';
import { getScenario, SCENARIOS } from '../demo/scenarios';
import { PerceptionFrame, ScenarioId } from '../types/perception';
import { DataSourceStatus, FrameCallback, PerceptionFrameSource } from './PerceptionFrameSource';

export class FixturePerceptionSource implements PerceptionFrameSource {
  private activeScenarioId: ScenarioId = 'D';
  private subscribers: Set<FrameCallback> = new Set();
  private status: DataSourceStatus = 'FIXTURE';

  constructor(initialScenario: ScenarioId = 'D') {
    this.activeScenarioId = initialScenario;

    // Run contract and fixture conservation validation upon initialization
    validateBackendStatusContract();
    for (const [id, frame] of Object.entries(SCENARIOS)) {
      const res = validatePerceptionFrame(frame);
      if (!res.valid) {
        throw new Error(
          `Fixture validation failed for scenario ${id}:\n${res.errors.join('\n')}`
        );
      }
    }
  }

  public async connect(): Promise<void> {
    this.status = 'FIXTURE';
    this.notifySubscribers();
  }

  public disconnect(): void {
    this.status = 'DISCONNECTED';
  }

  public subscribe(callback: FrameCallback): () => void {
    this.subscribers.add(callback);
    // Immediately deliver current frame upon subscription
    callback(this.getCurrentFrame());

    return () => {
      this.subscribers.delete(callback);
    };
  }

  public getStatus(): DataSourceStatus {
    return this.status;
  }

  public setScenario(id: ScenarioId): void {
    if (this.activeScenarioId !== id) {
      this.activeScenarioId = id;
      this.notifySubscribers();
    }
  }

  public getActiveScenario(): ScenarioId {
    return this.activeScenarioId;
  }

  public getCurrentFrame(): PerceptionFrame {
    return getScenario(this.activeScenarioId);
  }

  private notifySubscribers(): void {
    const frame = this.getCurrentFrame();
    for (const sub of this.subscribers) {
      try {
        sub(frame);
      } catch (err) {
        console.error('Error in PerceptionFrameSource subscriber:', err);
      }
    }
  }
}

// Global singleton instance for F1 UI
export const defaultFixtureSource = new FixturePerceptionSource('D');
