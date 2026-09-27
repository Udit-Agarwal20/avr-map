import { defaultFixtureSource } from '../data/FixturePerceptionSource';
import { DataSourceStatus, PerceptionFrameSource } from '../data/PerceptionFrameSource';
import {
  AdaptivePipelineStatusName,
  DynamicSummary,
  PerceptionFrame,
  PointXYZ,
  ResolutionSummary,
  ScenarioId,
  SemanticClass,
  SemanticSummary,
} from '../types/perception';

export interface DynamicClusterSummary {
  clusterLocalId: number;
  pointCount: number;
  dominantClass: SemanticClass;
  meanSemanticConfidence: number;
  meanMotionConfidence: number;
  aabbMin: PointXYZ;
  aabbMax: PointXYZ;
}

export interface FrameMetadata {
  frameSeq: number;
  pipelineStatus: AdaptivePipelineStatusName;
  scenarioId: ScenarioId;
  scenarioTitle: string;
  scenarioDescription: string;
  sensorOrigin: PointXYZ;
  sourceStatus: DataSourceStatus;
  cellCount: number;
  pointCount: number;
  dynamicPointCount: number;
  clusterCount: number;
  noiseCount: number;
  resolutionSummary: ResolutionSummary;
  semanticSummary: SemanticSummary;
  dynamicSummary: DynamicSummary;
  dynamicClustersSummary: DynamicClusterSummary[];
}

/**
 * @brief Dedicated runtime boundary for high-frequency perception frames.
 *
 * Guarantees:
 * 1. Perception frame payloads (thousands of points, cells, dynamic objects)
 *    are owned exclusively by this runtime layer and NEVER stored in Zustand.
 * 2. React UI shell components subscribe to lightweight derived FrameMetadata.
 * 3. Future Canvas F2 engine directly reads latest frame via getLatestFrame()
 *    without passing arrays through React props or global store.
 */
export class PerceptionFrameRuntime {
  private source: PerceptionFrameSource;
  private currentFrame: PerceptionFrame;
  private currentMetadata: FrameMetadata;
  private metadataListeners: Set<(meta: FrameMetadata) => void> = new Set();
  private frameListeners: Set<(frame: PerceptionFrame) => void> = new Set();

  constructor(source: PerceptionFrameSource) {
    this.source = source;
    this.currentFrame = source.getCurrentFrame();
    this.currentMetadata = this.extractMetadata(this.currentFrame);

    this.source.subscribe((frame) => {
      this.currentFrame = frame;
      this.currentMetadata = this.extractMetadata(frame);
      this.notifyListeners();
    });
  }

  public getLatestFrame(): PerceptionFrame {
    return this.currentFrame;
  }

  public getMetadata(): FrameMetadata {
    return this.currentMetadata;
  }

  public getSourceStatus(): DataSourceStatus {
    return this.source.getStatus();
  }

  public setScenario(id: ScenarioId): void {
    this.source.setScenario(id);
  }

  public subscribeMetadata(callback: (meta: FrameMetadata) => void): () => void {
    this.metadataListeners.add(callback);
    callback(this.currentMetadata);
    return () => {
      this.metadataListeners.delete(callback);
    };
  }

  public subscribeFrame(callback: (frame: PerceptionFrame) => void): () => void {
    this.frameListeners.add(callback);
    callback(this.currentFrame);
    return () => {
      this.frameListeners.delete(callback);
    };
  }

  private extractMetadata(frame: PerceptionFrame): FrameMetadata {
    return {
      frameSeq: frame.frameSeq,
      pipelineStatus: frame.pipelineStatus,
      scenarioId: frame.scenarioId,
      scenarioTitle: frame.scenarioTitle,
      scenarioDescription: frame.scenarioDescription,
      sensorOrigin: frame.sensorOrigin,
      sourceStatus: this.source.getStatus(),
      cellCount: frame.cells.length,
      pointCount: frame.points.length,
      dynamicPointCount: frame.dynamicPoints.length,
      clusterCount: frame.dynamicClusters.length,
      noiseCount: frame.noiseSourceIndices.length,
      resolutionSummary: frame.resolutionSummary,
      semanticSummary: frame.semanticSummary,
      dynamicSummary: frame.dynamicSummary,
      dynamicClustersSummary: frame.dynamicClusters.map((c) => ({
        clusterLocalId: c.clusterLocalId,
        pointCount: c.pointCount,
        dominantClass: c.dominantClass,
        meanSemanticConfidence: c.meanSemanticConfidence,
        meanMotionConfidence: c.meanMotionConfidence,
        aabbMin: c.aabbMin,
        aabbMax: c.aabbMax,
      })),
    };
  }

  private notifyListeners(): void {
    for (const listener of this.metadataListeners) {
      listener(this.currentMetadata);
    }
    for (const listener of this.frameListeners) {
      listener(this.currentFrame);
    }
  }
}

// Global runtime instance
export const frameRuntime = new PerceptionFrameRuntime(defaultFixtureSource);
