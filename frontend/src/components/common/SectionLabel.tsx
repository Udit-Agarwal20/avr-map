import React from 'react';
import styles from './SectionLabel.module.css';

interface SectionLabelProps {
  title: string;
  extra?: string | number;
  className?: string;
}

export const SectionLabel: React.FC<SectionLabelProps> = ({
  title,
  extra,
  className = '',
}) => {
  return (
    <div className={`${styles.header} ${className}`}>
      <span className={styles.title}>{title}</span>
      {extra !== undefined && <span className={styles.extra}>{extra}</span>}
    </div>
  );
};
