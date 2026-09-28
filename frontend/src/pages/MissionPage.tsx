import React from 'react';
import { PerceptionViewport } from '../visualization/PerceptionViewport';
import styles from './MissionPage.module.css';

/**
 * @file MissionPage.tsx
 * @brief Hero Autonomous Perception Scene Page (Milestone F2A).
 *
 * Implements "The Scene is the Interface":
 * - 88-92% perception scene, 8-12% persistent UI chrome
 * - Plain Three.js digital-twin perception engine
 * - Direct frame read from PerceptionFrameRuntime
 * - Zero high-volume point/cell arrays in React/Zustand state
 */
export const MissionPage: React.FC = () => {
  return (
    <div className={styles.pageContainer}>
      <PerceptionViewport />
    </div>
  );
};
