import React, { lazy, Suspense } from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { AppShell } from './components/shell/AppShell';
const MissionPage = lazy(() => import('./pages/MissionPage').then(m => ({ default: m.MissionPage })));
const ComparePage = lazy(() => import('./pages/ComparePage').then(m => ({ default: m.ComparePage })));
const ArchitecturePage = lazy(() => import('./pages/ArchitecturePage').then(m => ({ default: m.ArchitecturePage })));
const DiagnosticsPage = lazy(() => import('./pages/DiagnosticsPage').then(m => ({ default: m.DiagnosticsPage })));
const BenchmarksPage = lazy(() => import('./pages/BenchmarksPage').then(m => ({ default: m.BenchmarksPage })));
const ConfigPage = lazy(() => import('./pages/ConfigPage').then(m => ({ default: m.ConfigPage })));

export const App: React.FC = () => {
  return (
    <BrowserRouter future={{ v7_startTransition: true, v7_relativeSplatPath: true }}>
      <Suspense fallback={<div role="status" style={{padding:24}}>Loading AVR workspace…</div>}>
      <Routes>
        <Route element={<AppShell />}>
          <Route path="/mission" element={<MissionPage />} />
          <Route path="/compare" element={<ComparePage />} />
          <Route path="/benchmarks" element={<BenchmarksPage />} />
          <Route path="/architecture" element={<ArchitecturePage />} />
          <Route path="/diagnostics" element={<DiagnosticsPage />} />
          <Route path="/config" element={<ConfigPage />} />
          <Route path="/" element={<Navigate to="/mission" replace />} />
          <Route path="*" element={<Navigate to="/mission" replace />} />
        </Route>
      </Routes>
      </Suspense>
    </BrowserRouter>
  );
};

export default App;
