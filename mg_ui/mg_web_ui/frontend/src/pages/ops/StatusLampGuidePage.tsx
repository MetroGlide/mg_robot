import { Link } from "react-router-dom";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { TOPICS } from "../../ros/topics";
import { ActivityLampTable } from "../../types";
import StatusLamp from "../../components/operate/StatusLamp";
import {
  LampMotion,
  LampResponse,
  LampSpec,
  MOTION_LABELS,
  RESPONSE_LABELS,
  STALE_LAMP,
  UNKNOWN_LAMP,
  lampLabel,
  lampOf,
} from "../../components/operate/statusLamp";
import { ACTIVITY_LOG_PATH } from "../../components/operate/RecentActivity";
import WorkLayout from "../../components/work/WorkLayout";

const RESPONSES = Object.keys(RESPONSE_LABELS) as LampResponse[];
const MOTIONS = Object.keys(MOTION_LABELS) as LampMotion[];

function LampWithLabel({ lamp }: { lamp: LampSpec }) {
  return (
    <span className="flex items-center gap-1.5 whitespace-nowrap">
      <StatusLamp lamp={lamp} size={12} />
      {lampLabel(lamp)}
    </span>
  );
}

const cellClass = "border border-line px-2 py-1.5 align-top";

/**
 * 状態ランプの見方。活動ごとの意味はロボット側の表 (~/activity_lamps)、色と点滅は statusLamp.ts から作るので、
 * どちらの表を変えてもこのページが変わる。
 */
export default function StatusLampGuidePage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const table = useTopicSubscriber<ActivityLampTable>(
    client,
    TOPICS.WAYPOINT_ACTIVITY_LAMPS,
    "mg_msgs/msg/ActivityLampTable",
  );
  const lamps = table?.lamps ?? [];
  return (
    <WorkLayout
      title="状態ランプの見方"
      headerRight={
        <Link
          to={ACTIVITY_LOG_PATH}
          className="text-xs text-muted underline hover:text-content"
        >
          状態の履歴へ
        </Link>
      }
    >
      <p className="max-w-[70ch] text-sm text-muted">
        色は「人がどうすればよいか」、点灯・点滅は「動いているか」で決まります。状態の判定はロボット
        (シーケンサの「いまの活動」) が行い、画面はその結果を表示するだけです。
      </p>
      {lamps.length === 0 && (
        <p className="text-sm text-muted">
          ロボットからランプの表 (~/activity_lamps) がまだ届いていません。
        </p>
      )}

      <section className="space-y-2">
        <h2 className="text-sm font-bold">色と点滅の組み合わせ</h2>
        <div className="overflow-x-auto">
          <table className="min-w-[44rem] border-collapse text-xs">
            <thead>
              <tr className="bg-surface-sunken text-muted">
                <th className={`${cellClass} text-left font-medium`}>
                  人がどうすればよいか
                </th>
                {MOTIONS.map((motion) => (
                  <th
                    key={motion}
                    className={`${cellClass} text-left font-medium`}
                  >
                    {MOTION_LABELS[motion]}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {RESPONSES.map((response) => (
                <tr key={response}>
                  <th className={`${cellClass} text-left font-medium`}>
                    {RESPONSE_LABELS[response]}
                  </th>
                  {MOTIONS.map((motion) => {
                    const entries = lamps.filter(
                      (lamp) =>
                        lamp.response === response && lamp.motion === motion,
                    );
                    return (
                      <td key={motion} className={cellClass}>
                        {entries.length === 0 ? (
                          <span className="text-muted">—</span>
                        ) : (
                          <div className="space-y-1">
                            <LampWithLabel lamp={lampOf(response, motion)} />
                            <ul className="space-y-0.5">
                              {entries.map((lamp) => (
                                <li key={lamp.code}>{lamp.description}</li>
                              ))}
                            </ul>
                          </div>
                        )}
                      </td>
                    );
                  })}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </section>

      <section className="space-y-2">
        <h2 className="text-sm font-bold">状態ごとのランプ</h2>
        <div className="overflow-x-auto">
          <table className="w-full min-w-[44rem] border-collapse text-xs">
            <thead>
              <tr className="bg-surface-sunken text-muted">
                <th className={`${cellClass} text-left font-medium`}>ランプ</th>
                <th className={`${cellClass} text-left font-medium`}>状態</th>
                <th className={`${cellClass} text-left font-medium`}>code</th>
                <th className={`${cellClass} text-left font-medium`}>
                  人がどうすればよいか
                </th>
                <th className={`${cellClass} text-left font-medium`}>動き</th>
              </tr>
            </thead>
            <tbody>
              {lamps.map((m) => {
                const code = m.code;
                return (
                  <tr key={code}>
                    <td className={cellClass}>
                      <LampWithLabel lamp={lampOf(m.response, m.motion)} />
                    </td>
                    <td className={cellClass}>{m.description}</td>
                    <td className={`${cellClass} font-mono`}>{code}</td>
                    <td className={cellClass}>
                      {RESPONSE_LABELS[m.response as LampResponse] ??
                        m.response}
                    </td>
                    <td className={cellClass}>
                      {MOTION_LABELS[m.motion as LampMotion] ?? m.motion}
                    </td>
                  </tr>
                );
              })}
              <tr>
                <td className={cellClass}>
                  <LampWithLabel lamp={STALE_LAMP} />
                </td>
                <td className={cellClass}>
                  状態が届かない (通信の途絶・シーケンサの停止)
                </td>
                <td className={`${cellClass} text-muted`}>(画面が決める)</td>
                <td className={cellClass} colSpan={2}>
                  ロボットのエラー (赤) と分けて灰にする。MMI の LED1 は白の点灯
                  (シーケンサごと止まれば LED2 が赤)
                </td>
              </tr>
              <tr>
                <td className={cellClass}>
                  <LampWithLabel lamp={UNKNOWN_LAMP} />
                </td>
                <td className={cellClass}>
                  画面が知らない状態 (ロボット側が新しい)
                </td>
                <td className={`${cellClass} text-muted`}>(画面が決める)</td>
                <td className={cellClass} colSpan={2}>
                  画面を更新する
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </section>

      <section className="space-y-1 text-xs text-muted">
        <h2 className="text-sm font-bold text-content">
          MMI のボタンの LED との関係
        </h2>
        <p className="max-w-[70ch]">
          MMI の LED1 は、画面のランプと同じ意味から色を決めます
          (動いていれば点灯、止まっていれば点滅)。MMI の点滅は 1
          種類だけなので、遅い点滅と速い点滅は同じに見えます。状態が届かないときは白で点灯します。LED2
          は MMI の通信の表示 (正常は暗い白、通信が途絶えると赤) です。SW3
          を押すと走行を止めます。
        </p>
      </section>
    </WorkLayout>
  );
}
