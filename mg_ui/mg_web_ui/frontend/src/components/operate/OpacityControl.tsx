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

/** パネルの背景の透明度を調整するボタン。押すと左にスライダーを開く */
export default function OpacityControl() {
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
        <div className="absolute bottom-0 right-full z-10 mr-2">
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
