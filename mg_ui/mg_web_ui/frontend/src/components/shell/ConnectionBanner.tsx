import { ConnectionStatus } from "../../hooks/useFoxgloveClient";

/**
 * ロボットと接続できていないとき、画面の上に出す帯。
 * Wi-Fi の瞬断などで表示が古くなっていることを、計器を見なくても分かるようにする。
 */
export default function ConnectionBanner({
  status,
}: {
  status: ConnectionStatus;
}) {
  if (status === "connected") return null;
  const text =
    status === "connecting"
      ? "ロボットに接続しています…"
      : "ロボットに接続できません。自動で再接続します。表示は最後に受信した値で、操作は無効です。";
  return (
    <div
      role="alert"
      className="bg-error px-4 py-1.5 text-center text-xs font-semibold text-surface-elevated"
    >
      {text}
    </div>
  );
}
