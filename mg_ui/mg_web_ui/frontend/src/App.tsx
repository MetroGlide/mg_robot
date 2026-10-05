import { lazy, Suspense, useState } from "react";
import { Routes, Route } from "react-router-dom";
import { useFoxgloveClient } from "./hooks/useFoxgloveClient";
import { useSystemManagerClient } from "./hooks/useSystemManagerClient";
import { SimulationProvider } from "./contexts/SimulationContext";
import { VisualizationProvider } from "./contexts/VisualizationContext";
import { TeleopProvider } from "./contexts/TeleopContext";
import { RosbagReplayProvider } from "./contexts/RosbagReplayContext";
import NavBar from "./components/layout/NavBar";
import ConnectionBadge from "./components/ConnectionBadge";
import SettingModal from "./components/SettingModal";

// three.js や地図ライブラリを使うページは、開いたときに読み込む
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
  const [settingOpen, setSettingOpen] = useState(false);

  return (
    <SimulationProvider>
      <RosbagReplayProvider>
        <VisualizationProvider>
          <TeleopProvider>
            <div className="min-h-screen bg-gray-900 text-white">
              <header className="flex items-center justify-between px-4 py-3 bg-gray-800 shadow-md">
                <span className="text-lg font-bold tracking-wide">
                  MG-01 Control UI
                </span>
                <ConnectionBadge status={client.status} />
              </header>
              <NavBar onSettingClick={() => setSettingOpen(true)} />
              <main className="p-4">
                <Suspense
                  fallback={<p className="text-sm text-gray-500">Loading…</p>}
                >
                  <Routes>
                    <Route path="/" element={<TopPage client={client} />} />
                    <Route
                      path="/waypoint"
                      element={
                        <WaypointNavPage
                          client={client}
                          sysManager={sysManager}
                        />
                      }
                    />
                    <Route
                      path="/slam"
                      element={
                        <SlamPage client={client} sysManager={sysManager} />
                      }
                    />
                    <Route
                      path="/slam-gnss-2d"
                      element={
                        <SlamGnss2DPage
                          client={client}
                          sysManager={sysManager}
                        />
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
                        <ScenarioTestPage
                          client={client}
                          sysManager={sysManager}
                        />
                      }
                    />
                    <Route path="/setting" element={<SettingPage />} />
                  </Routes>
                </Suspense>
              </main>
              <SettingModal
                open={settingOpen}
                onClose={() => setSettingOpen(false)}
              />
            </div>
          </TeleopProvider>
        </VisualizationProvider>
      </RosbagReplayProvider>
    </SimulationProvider>
  );
}
