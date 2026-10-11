import { useNow } from "../../hooks/useNow";
import { elapsedSinceSec, formatElapsed } from "../../utils/scenarioTest";

/** 開始時刻からの経過時間を、1 秒ごとに更新して出す。開始時刻が読めなければ「-」 */
export default function ElapsedText({ startedAt }: { startedAt?: string }) {
  const now = useNow(true);
  const sec = elapsedSinceSec(startedAt, now);
  return <>{sec === null ? "-" : formatElapsed(sec)}</>;
}
