import type { ActionResult } from "../../utils/waypointActions";

interface ActionResultTextProps {
  result: ActionResult | null;
}

export default function ActionResultText({ result }: ActionResultTextProps) {
  if (result === null) return null;
  return (
    <p
      className={`text-xs break-all whitespace-pre-wrap ${
        result.ok ? "text-green-400" : "text-red-400"
      }`}
    >
      {result.text}
    </p>
  );
}
