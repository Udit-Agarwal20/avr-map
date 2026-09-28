import { PerceptionFrame, ScenarioId } from '../types/perception';

export type DataSourceStatus = 'FIXTURE' | 'DISCONNECTED' | 'CONNECTED';

export type FrameCallback = (frame: PerceptionFrame) => void;

/**
 * @brief Abstract data source boundary for perception frames.
 * Allows seamless hot-swapping between offline deterministic fixtures (today)
 * and real backend WebSocket/ROS transports (future).
 */
export interface PerceptionFrameSource {
  connect(): Promise<void>;
  disconnect(): void;
  subscribe(callback: FrameCallback): () => void;
  getStatus(): DataSourceStatus;
  setScenario(id: ScenarioId): void;
  getActiveScenario(): ScenarioId;
  getCurrentFrame(): PerceptionFrame;
}

/** Future gateway boundary. A decoder must validate protocol version, sequence,
 * snapshot/delta revisions and routing before publishing a frame to the runtime.
 * No transport is connected or simulated by this interface. */
export interface GatewayFrameDecoder {
  decode(packet: ArrayBuffer): Promise<PerceptionFrame>;
  reset(): void;
}
