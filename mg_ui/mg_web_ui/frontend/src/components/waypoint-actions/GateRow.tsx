import ActionButton from "../ui/ActionButton";
import type {
  ActionResult,
  GateLabel,
  GateView,
} from "../../utils/waypointActions";
import ActionResultText from "./ActionResultText";

const LABEL_CLASS: Record<GateLabel, string> = {
  ON: "bg-green-600",
  OFF: "bg-red-600",
  PENDING: "bg-yellow-600",
  UNKNOWN: "bg-muted",
};

interface GateRowProps {
  title: string;
  view: GateView;
  busy: boolean;
  result: ActionResult | null;
  onSet: (enable: boolean) => void;
}

/** 現在の状態と ON/OFF のボタンを並べた 1 行。状態が不明でも操作はできる。 */
export default function GateRow({
  title,
  view,
  busy,
  result,
  onSet,
}: GateRowProps) {
  return (
    <div className="space-y-1">
      <div className="flex flex-wrap items-center gap-3">
        <span className="w-16 text-sm font-medium">{title}</span>
        <span
          className={`${LABEL_CLASS[view.label]} rounded px-2 py-0.5 text-xs font-semibold`}
        >
          {view.label}
        </span>
        <ActionButton
          label="ON"
          size="sm"
          variant="green"
          disabled={busy}
          onClick={() => onSet(true)}
        />
        <ActionButton
          label="OFF"
          size="sm"
          variant="red"
          disabled={busy}
          onClick={() => onSet(false)}
        />
        {view.detail && (
          <span className="text-xs text-muted">{view.detail}</span>
        )}
      </div>
      <ActionResultText result={result} />
    </div>
  );
}
