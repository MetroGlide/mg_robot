import { useEffect, useRef, useState } from "react";
import {
  DEFAULT_PANEL_OPACITY,
  MAX_PANEL_OPACITY,
  MIN_PANEL_OPACITY,
  usePanelOpacity,
} from "../../hooks/usePanelOpacity";
import Card from "../ui/Card";
import IconButton from "../ui/IconButton";
import { OpacityIcon } from "../ui/icons";
import { PANELS, PanelId, usePanelVisible } from "./operatePanels";

function PanelButton({ id }: { id: PanelId }) {
  const [visible, setVisible] = usePanelVisible(id);
  const { title, icon: PanelIcon } = PANELS[id];
  return (
    <IconButton
      icon={<PanelIcon />}
      title={`${title}を${visible ? "隠す" : "表示"}`}
      active={visible}
      onClick={() => setVisible(!visible)}
    />
  );
}

/** パネルの背景の透明度を調整するスライダー。ボタンの左に開く */
function OpacityControl() {
  const [open, setOpen] = useState(false);
  const [opacity, setOpacity] = usePanelOpacity();
  const rootRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!open) return;
    const close = (e: PointerEvent) => {
      if (!rootRef.current?.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener("pointerdown", close);
    return () => document.removeEventListener("pointerdown", close);
  }, [open]);

  return (
    <div ref={rootRef} className="relative">
      <IconButton
        icon={<OpacityIcon />}
        title="パネルの透明度"
        active={open}
        onClick={() => setOpen(!open)}
      />
      {open && (
        <div className="absolute right-full top-0 mr-2">
          <Card solid className="flex w-52 flex-col gap-1 p-3">
            <div className="flex items-baseline justify-between text-xs">
              <span className="font-semibold">パネルの不透明度</span>
              <span className="tabular-nums text-muted">{Math.round(opacity * 100)}%</span>
            </div>
            <input
              type="range"
              aria-label="パネルの不透明度"
              min={MIN_PANEL_OPACITY * 100}
              max={MAX_PANEL_OPACITY * 100}
              step={5}
              value={Math.round(opacity * 100)}
              onChange={(e) => setOpacity(Number(e.target.value) / 100)}
              className="w-full accent-accent"
            />
            <button
              type="button"
              onClick={() => setOpacity(DEFAULT_PANEL_OPACITY)}
              className="self-end text-xs text-accent hover:opacity-80"
            >
              元に戻す
            </button>
          </Card>
        </div>
      )}
    </div>
  );
}

/**
 * 右端の、パネルの表示切り替え。地図ツールバーとは別のカードで、同じ列の上に置く。
 * ids に、そのビューにあるパネルを並べる。末尾に、パネルの透明度の調整を置く。
 */
export default function PanelToggleBar({ ids }: { ids: PanelId[] }) {
  return (
    <Card solid className="flex flex-col gap-1 p-1">
      {ids.map((id) => (
        <PanelButton key={id} id={id} />
      ))}
      <div className="my-0.5 h-px bg-line" />
      <OpacityControl />
    </Card>
  );
}
