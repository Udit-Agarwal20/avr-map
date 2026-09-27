import React from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { AppShell } from './components/shell/AppShell';
import { MissionPage } from './pages/MissionPage';
import { ComparePage } from './pages/ComparePage';
import { ArchitecturePage } from './pages/ArchitecturePage';
import { DiagnosticsPage } from './pages/DiagnosticsPage';
import { ConfigPage } from './pages/ConfigPage';

export const App: React.FC = () => {
  return (
    <BrowserRouter future={{ v7_startTransition: true, v7_relativeSplatPath: true }}>
      <Routes>
        <Route element={<AppShell />}>
          <Route path="/mission" element={<MissionPage />} />
          <Route path="/compare" element={<ComparePage />} />
          <Route path="/architecture" element={<ArchitecturePage />} />
          <Route path="/diagnostics" element={<DiagnosticsPage />} />
          <Route path="/config" element={<ConfigPage />} />
          <Route path="/" element={<Navigate to="/mission" replace />} />
          <Route path="*" element={<Navigate to="/mission" replace />} />
        </Route>
      </Routes>
    </BrowserRouter>
  );
};

export default App;
