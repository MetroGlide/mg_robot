import { useState } from "react";
import { Outlet } from "react-router-dom";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import SettingModal from "../SettingModal";
import ConnectionBanner from "./ConnectionBanner";
import TopBar, { UseCaseLink } from "./TopBar";

interface Props {
  client: FoxgloveClientHandle;
  useCases: UseCaseLink[];
}

/** 新 UI (/ops/*) の枠。画面いっぱいに使い、ページ全体はスクロールさせない */
export default function AppShell({ client, useCases }: Props) {
  const [settingOpen, setSettingOpen] = useState(false);

  return (
    <div className="flex h-screen flex-col bg-surface text-content">
      <TopBar
        client={client}
        useCases={useCases}
        onSettingClick={() => setSettingOpen(true)}
      />
      <ConnectionBanner status={client.status} />
      <main className="relative min-h-0 flex-1">
        <Outlet />
      </main>
      <SettingModal open={settingOpen} onClose={() => setSettingOpen(false)} />
    </div>
  );
}
