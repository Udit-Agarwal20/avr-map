import React from 'react';
import styles from './StatusIndicator.module.css';

export type StatusIndicatorType =
  | 'healthy'
  | 'warning'
  | 'critical'
  | 'info'
  | 'muted'
  | 'degraded'
  | 'rejected'
  | 'error'
  | 'terminal';

interface StatusIndicatorProps {
  label: string;
  status: StatusIndicatorType;
  className?: string;
}

export const StatusIndicator: React.FC<StatusIndicatorProps> = ({
  label,
  status,
  className = '',
}) => {
  // Normalize severity names to style classes
  let styleKey: 'healthy' | 'warning' | 'critical' | 'terminal' | 'info' | 'muted' = 'info';
  if (status === 'healthy') styleKey = 'healthy';
  else if (status === 'warning' || status === 'degraded' || status === 'rejected') styleKey = 'warning';
  else if (status === 'critical' || status === 'error') styleKey = 'critical';
  else if (status === 'terminal') styleKey = 'terminal';
  else if (status === 'muted') styleKey = 'muted';

  return (
    <div className={`${styles.indicator} ${className}`}>
      <span className={`${styles.dot} ${styles[styleKey]}`} />
      <span>{label}</span>
    </div>
  );
};
