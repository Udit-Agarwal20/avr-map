import React from 'react';
import styles from './MetricValue.module.css';

interface MetricValueProps {
  label: string;
  value: string | number;
  unit?: string;
  large?: boolean;
  className?: string;
  color?: string;
}

export const MetricValue: React.FC<MetricValueProps> = ({
  label,
  value,
  unit,
  large = false,
  className = '',
  color,
}) => {
  return (
    <div className={`${styles.container} ${className}`}>
      <span className={styles.label}>{label}</span>
      <div className={styles.valueRow}>
        <span
          className={`${styles.value} ${large ? styles.large : ''}`}
          style={color ? { color } : undefined}
        >
          {value}
        </span>
        {unit && <span className={styles.unit}>{unit}</span>}
      </div>
    </div>
  );
};
