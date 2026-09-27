import React from 'react';
import { NavLink } from 'react-router-dom';
import styles from './NavRail.module.css';

interface NavEntry {
  to: string;
  id: string;
  label: string;
  icon: React.ReactNode;
}

export const NavRail: React.FC = () => {
  const topNav: NavEntry[] = [
    {
      to: '/mission',
      id: 'nav-mission',
      label: 'Mission',
      icon: (
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6">
          <circle cx="12" cy="12" r="9" />
          <path d="M12 3v4M12 17v4M3 12h4M17 12h4" />
          <circle cx="12" cy="12" r="2" fill="currentColor" />
        </svg>
      ),
    },
    {
      to: '/compare',
      id: 'nav-compare',
      label: 'Compare',
      icon: (
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6">
          <rect x="3" y="4" width="8" height="16" rx="1" />
          <rect x="13" y="4" width="8" height="16" rx="1" />
          <path d="M8 9h-2M18 9h-2M8 15h-2M18 15h-2" />
        </svg>
      ),
    },
    {
      to: '/architecture',
      id: 'nav-architecture',
      label: 'Pipeline',
      icon: (
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6">
          <rect x="4" y="4" width="6" height="4" rx="1" />
          <rect x="14" y="10" width="6" height="4" rx="1" />
          <rect x="4" y="16" width="6" height="4" rx="1" />
          <path d="M10 6h4v4M14 14H10v4" />
        </svg>
      ),
    },
    {
      to: '/diagnostics',
      id: 'nav-diagnostics',
      label: 'Health',
      icon: (
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6">
          <path d="M3 12h4l3-7 4 14 3-7h4" />
        </svg>
      ),
    },
  ];

  const bottomNav: NavEntry[] = [
    {
      to: '/config',
      id: 'nav-config',
      label: 'Config',
      icon: (
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6">
          <circle cx="12" cy="12" r="3" />
          <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z" />
        </svg>
      ),
    },
  ];

  return (
    <nav className={styles.navRail} aria-label="Main Navigation">
      <div className={styles.topLinks}>
        {topNav.map((item) => (
          <NavLink
            key={item.to}
            id={item.id}
            to={item.to}
            className={({ isActive }) =>
              `${styles.navItem} ${isActive ? styles.navItemActive : ''}`
            }
            title={item.label}
          >
            <div className={styles.icon}>{item.icon}</div>
            <span className={styles.label}>{item.label}</span>
          </NavLink>
        ))}
      </div>

      <div className={styles.bottomLinks}>
        {bottomNav.map((item) => (
          <NavLink
            key={item.to}
            id={item.id}
            to={item.to}
            className={({ isActive }) =>
              `${styles.navItem} ${isActive ? styles.navItemActive : ''}`
            }
            title={item.label}
          >
            <div className={styles.icon}>{item.icon}</div>
            <span className={styles.label}>{item.label}</span>
          </NavLink>
        ))}
      </div>
    </nav>
  );
};
