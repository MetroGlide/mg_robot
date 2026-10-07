import { lazy } from "react";
import { Routes, Route, Navigate } from "react-router-dom";
import { useFoxgloveClient } from "./hooks/useFoxgloveClient";
import { useSystemManagerClient } from "./hooks/useSystemManagerClient";
import { SimulationProvider } from "./contexts/SimulationContext";
import { VisualizationProvider } from "./contexts/VisualizationContext";
import { TeleopProvider } from "./contexts/TeleopContext";
import { RosbagReplayProvider } from "./contexts/RosbagReplayContext";
import { RobotProfileProvider } from "./contexts/RobotProfileContext";
import { GnssSourceProvider } from "./contexts/GnssSourceContext";
import LegacyLayout from "./components/layout/LegacyLayout";
import AppShell from "./components/shell/AppShell";
import OperatePage from "./pages/ops/OperatePage";
import { USE_CASES, USE_CASE_LINKS } from "./pages/ops/useCases";

// three.js や地図ライブラリを使うページは、開いたときに読み込む
const SensorsPage = lazy(() => import("./pages/ops/SensorsPage"));
const SystemOpsPage = lazy(() => import("./pages/ops/SystemOpsPage"));
const TopPage = lazy(() => import("./pages/TopPage"));
const WaypointNavPage = lazy(() => import("./pages/WaypointNavPage"));
const SlamPage = lazy(() => import("./pages/SlamPage"));
const SlamGnss2DPage = lazy(() => import("./pages/SlamGnss2DPage"));
const SystemPage = lazy(() => import("./pages/SystemPage"));
const SettingPage = lazy(() => import("./pages/SettingPage"));
const ScenarioTestPage = lazy(() => import("./pages/ScenarioTestPage"));

export default function App() {
  const client = useFoxgloveClient();
  const sysManager = useSystemManagerClient();

  return (
    <RobotProfileProvider>
    <GnssSourceProvider>
    <SimulationProvider>
      <RosbagReplayProvider>
        <VisualizationProvider>
          <TeleopProvider>
            <Routes>
              {/* 新 UI。ユースケースごとの運用ビューを /ops/:useCase に置く */}
              <Route
                path="/ops"
                element={<AppShell client={client} links={USE_CASE_LINKS} />}
              >
                <Route
                  index
                  element={<Navigate to={USE_CASES[0].id} replace />}
                />
                <Route
                  path="sensors"
                  element={<SensorsPage client={client} />}
                />
                <Route
                  path="system"
                  element={
                    <SystemOpsPage client={client} sysManager={sysManager} />
                  }
                />
                <Route
                  path=":useCase"
                  element={
                    <OperatePage client={client} sysManager={sysManager} />
                  }
                />
              </Route>

              {/* 旧 UI。移行が終わるまで残す(doc/ui_migration_todo.md) */}
              <Route element={<LegacyLayout status={client.status} />}>
                <Route path="/" element={<TopPage client={client} />} />
                <Route
                  path="/waypoint"
                  element={
                    <WaypointNavPage client={client} sysManager={sysManager} />
                  }
                />
                <Route
                  path="/slam"
                  element={<SlamPage client={client} sysManager={sysManager} />}
                />
                <Route
                  path="/slam-gnss-2d"
                  element={
                    <SlamGnss2DPage client={client} sysManager={sysManager} />
                  }
                />
                <Route
                  path="/system"
                  element={
                    <SystemPage client={client} sysManager={sysManager} />
                  }
                />
                <Route
                  path="/scenario-test"
                  element={
                    <ScenarioTestPage client={client} sysManager={sysManager} />
                  }
                />
                <Route path="/setting" element={<SettingPage />} />
              </Route>
            </Routes>
          </TeleopProvider>
        </VisualizationProvider>
      </RosbagReplayProvider>
    </SimulationProvider>
    </GnssSourceProvider>
    </RobotProfileProvider>
  );
}
