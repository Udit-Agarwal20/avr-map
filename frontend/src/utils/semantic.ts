import { PointRouting, SemanticClass } from '../types/perception';

export function semanticClassLabel(cls: SemanticClass): string {
  switch (cls) {
    case 'UNKNOWN':
      return 'Unknown';
    case 'GROUND':
      return 'Ground';
    case 'ROAD':
      return 'Road';
    case 'VEGETATION':
      return 'Vegetation';
    case 'BUILDING':
      return 'Building';
    case 'VEHICLE':
      return 'Vehicle';
    case 'PEDESTRIAN':
      return 'Pedestrian';
    case 'OBSTACLE_OTHER':
      return 'Obstacle (Other)';
  }
}

export function semanticClassColor(cls: SemanticClass): string {
  switch (cls) {
    case 'UNKNOWN':
      return '#A596C8';
    case 'GROUND':
      return '#8DB174';
    case 'ROAD':
      return '#7E8B95';
    case 'VEGETATION':
      return '#4F8A6C';
    case 'BUILDING':
      return '#858DA0';
    case 'VEHICLE':
      return '#6FA9EC';
    case 'PEDESTRIAN':
      return '#ED8177';
    case 'OBSTACLE_OTHER':
      return '#DEB65F';
  }
}

export function routingLabel(routing: PointRouting): string {
  switch (routing) {
    case 'STATIC_FUSABLE':
      return 'Static Map Evidence';
    case 'DYNAMIC_EXCLUDED':
      return 'Dynamic / Excluded from Static Map';
  }
}
