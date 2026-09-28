import { NavLink } from 'react-router-dom';
import { Crosshair, Columns, ChartLine, Pulse, SlidersHorizontal, Command } from '@phosphor-icons/react';
import { PRIMARY_ROUTES } from '../navigation/routes';
import { useConsoleStore } from '../../stores/useConsoleStore';
import styles from './NavRail.module.css';
const icons = [Crosshair, Columns, ChartLine, Pulse, SlidersHorizontal];
export function NavRail() {
    return <nav className={styles.navRail} aria-label="Main Navigation"><div className={styles.topLinks}>{PRIMARY_ROUTES.map((r, i) => { const Icon = icons[i]; return <NavLink key={r.path} to={r.path} title={r.label} aria-label={r.label} className={({ isActive }) => `${styles.navItem} ${isActive ? styles.navItemActive : ''}`}><Icon size={18}/><span className={styles.label}>{r.label}</span></NavLink>; })}</div><button className={styles.navItem} title="Commands (⌘/Ctrl K)" aria-label="Open command palette" onClick={() => useConsoleStore.getState().setCommandOpen(true)}><Command size={18}/><span className={styles.label}>Commands</span></button></nav>;
}
