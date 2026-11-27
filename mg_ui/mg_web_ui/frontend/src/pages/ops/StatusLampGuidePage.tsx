import { Link } from "react-router-dom";
import StatusLamp from "../../components/operate/StatusLamp";
import {
  ACTIVITY_MEANINGS,
  ActivityCode,
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
const CODES = Object.keys(ACTIVITY_MEANINGS) as ActivityCode[];

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
 * 状態ランプの見方。statusLamp.ts の表 (色の軸・動きの軸・活動ごとの意味) から作るので、表を変えるとこのページも変わる。
 */
export default function StatusLampGuidePage() {
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
                    const codes = CODES.filter(
                      (code) =>
                        ACTIVITY_MEANINGS[code].response === response &&
                        ACTIVITY_MEANINGS[code].motion === motion,
                    );
                    return (
                      <td key={motion} className={cellClass}>
                        {codes.length === 0 ? (
                          <span className="text-muted">—</span>
                        ) : (
                          <div className="space-y-1">
                            <LampWithLabel
                              lamp={lampOf(ACTIVITY_MEANINGS[codes[0]])}
                            />
                            <ul className="space-y-0.5">
                              {codes.map((code) => (
                                <li key={code}>
                                  {ACTIVITY_MEANINGS[code].description}
                                </li>
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
              {CODES.map((code) => {
                const m = ACTIVITY_MEANINGS[code];
                return (
                  <tr key={code}>
                    <td className={cellClass}>
                      <LampWithLabel lamp={lampOf(m)} />
                    </td>
                    <td className={cellClass}>{m.description}</td>
                    <td className={`${cellClass} font-mono`}>{code}</td>
                    <td className={cellClass}>{RESPONSE_LABELS[m.response]}</td>
                    <td className={cellClass}>{MOTION_LABELS[m.motion]}</td>
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
                  ロボットのエラー (赤) と分けて灰にする。MMI では LED2
                  が赤になる
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
          MMI の LED1
          は「ボタンで走行を始められるか」だけを表し、始められるときは橙の点滅、それ以外は緑の点灯です。画面の橙の点滅は同じ条件で出ます。MMI
          の緑の点灯は正常という意味ではないので、異常は画面のランプで見てください。
        </p>
      </section>
    </WorkLayout>
  );
}
