import { useEffect, useRef, useState } from 'react';
import * as Dialog from '@radix-ui/react-dialog';
import { useNavigate } from 'react-router-dom';
import { useConsoleStore } from '../../stores/useConsoleStore';
import { PRIMARY_ROUTES } from './routes';
import s from './CommandPalette.module.css';
export function CommandPalette() {
    const open = useConsoleStore(state => state.commandOpen);
    const setOpen = useConsoleStore(state => state.setCommandOpen);
    const previousFocus = useRef<HTMLElement | null>(null);
    const [query, setQuery] = useState('');
    const [index, setIndex] = useState(0);
    const navigate = useNavigate();
    useEffect(() => { if (open) {
        setQuery('');
        setIndex(0);
    } }, [open]);
    const ui = useConsoleStore.getState();
    const commands = [...PRIMARY_ROUTES.map(r => ({ label: `Go to ${r.label}`, action: () => navigate(r.path) })),
        { label: 'System architecture / About', action: () => navigate('/architecture') },
        { label: 'Switch Resolution view', action: () => { navigate('/mission'); ui.setViewMode('RESOLUTION'); } },
        { label: 'Switch Dynamics view', action: () => { navigate('/mission'); ui.setViewMode('DYNAMICS'); } },
        { label: 'Toggle point cloud', action: () => ui.toggleLayer('lidarPoints') },
        { label: 'Reset camera', action: () => ui.resetCamera() },
        { label: 'Open selected cell inspector / pedestrian example', action: () => { navigate('/mission'); ui.setScenario('D'); ui.setViewMode('RESOLUTION'); ui.setSelectedCellId(ui.selectedCellId ?? 'D_L0_P_127'); ui.resetCamera(); } },
        { label: ui.isPresentationMode ? 'Disable presentation mode' : 'Enable presentation mode', action: () => ui.togglePresentationMode() }];
    const filtered = commands.filter(c => c.label.toLowerCase().includes(query.toLowerCase()));
    const run = (n: number) => { filtered[n]?.action(); setOpen(false); };
    return <Dialog.Root open={open} onOpenChange={setOpen}><Dialog.Portal><Dialog.Overlay className={s.overlay}/><Dialog.Content className={s.dialog} onOpenAutoFocus={() => { previousFocus.current = document.activeElement as HTMLElement; }} onCloseAutoFocus={e => { e.preventDefault(); previousFocus.current?.focus(); }}><Dialog.Title className={s.heading}>AVR Console / Commands</Dialog.Title><Dialog.Description className={s.description}>Navigate, inspect evidence, or control the view.</Dialog.Description><input aria-label="Search commands" placeholder="Type a command…" value={query} onChange={e => { setQuery(e.target.value); setIndex(0); }} onKeyDown={e => { if (e.key === 'ArrowDown') {
        e.preventDefault();
        setIndex(i => Math.min(i + 1, filtered.length - 1));
    } if (e.key === 'ArrowUp') {
        e.preventDefault();
        setIndex(i => Math.max(0, i - 1));
    } if (e.key === 'Enter') {
        e.preventDefault();
        run(index);
    } }}/><div className={s.results}>{filtered.map((c, i) => <button data-active={i === index} key={c.label} onClick={() => run(i)}>{c.label}<span>↵</span></button>)}{!filtered.length && <p>No matching commands.</p>}</div><footer>↑ ↓ Select · Enter Run · Esc Close</footer></Dialog.Content></Dialog.Portal></Dialog.Root>;
}
