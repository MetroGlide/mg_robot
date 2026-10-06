import { Suspense, useState } from "react";
import { Outlet } from "react-router-dom";
import { ConnectionStatus } from "../../hooks/useFoxgloveClient";
import { useRobotProfile } from "../../contexts/RobotProfileContext";
import ConnectionBadge from "../ConnectionBadge";
import SettingModal from "../SettingModal";
import NavBar from "./NavBar";

/** 旧 UI(ページごとのアコーディオン構成)の枠 */
export default function LegacyLayout({ status }: { status: ConnectionStatus }) {
  const [settingOpen, setSettingOpen] = useState(false);
  const { name: robotName } = useRobotProfile();

  return (
    <div className="dark min-h-screen bg-gray-900 text-white">
      <header className="flex items-center justify-between px-4 py-3 bg-gray-800 shadow-md">
        <span className="text-lg font-bold tracking-wide">{robotName} Control UI</span>
        <ConnectionBadge status={status} />
      </header>
      <NavBar onSettingClick={() => setSettingOpen(true)} />
      <main className="p-4">
        <Suspense fallback={<p className="text-sm text-gray-500">Loading…</p>}>
          <Outlet />
        </Suspense>
      </main>
      <SettingModal open={settingOpen} onClose={() => setSettingOpen(false)} />
    </div>
  );
}
