import { isValidRobotAddress } from "../../utils/scenarioTest";

export interface RunOptionsValue {
  repeat: number;
  gui: boolean;
  attach: boolean;
  robot: string;
}

export const DEFAULT_RUN_OPTIONS: RunOptionsValue = {
  repeat: 1,
  gui: false,
  attach: false,
  robot: "",
};

const MAX_REPEAT = 20;

interface RunOptionsFormProps {
  value: RunOptionsValue;
  onChange: (next: RunOptionsValue) => void;
  disabled?: boolean;
}

export default function RunOptionsForm({
  value,
  onChange,
  disabled = false,
}: RunOptionsFormProps) {
  const set = (patch: Partial<RunOptionsValue>) =>
    onChange({ ...value, ...patch });
  const robotValid = isValidRobotAddress(value.robot);

  return (
    <div className="space-y-3 text-sm">
      <fieldset className="space-y-1" disabled={disabled}>
        <legend className="text-xs text-gray-400 mb-1">実行方法</legend>
        <label className="flex items-start gap-2">
          <input
            type="radio"
            className="mt-1"
            checked={!value.attach}
            onChange={() => set({ attach: false })}
          />
          <span>
            シナリオごとに起動し直す
            <span className="block text-xs text-gray-400">
              シミュレータとナビゲーションを毎回起動する(回帰テストと同じ条件)
            </span>
          </span>
        </label>
        <label className="flex items-start gap-2">
          <input
            type="radio"
            className="mt-1"
            checked={value.attach}
            onChange={() => set({ attach: true, robot: "" })}
          />
          <span>
            起動済みの環境で実行する(attach)
            <span className="block text-xs text-gray-400">
              下の「起動済み環境」を先に起動しておく。シミュレータを閉じずに続けて確認できる
            </span>
          </span>
        </label>
      </fieldset>

      <div className="flex flex-wrap items-center gap-4">
        <label className="flex items-center gap-2 select-none">
          <input
            type="checkbox"
            checked={value.gui && !value.attach}
            disabled={disabled || value.attach}
            onChange={(e) => set({ gui: e.target.checked })}
          />
          シミュレータの GUI を表示
        </label>
        <label className="flex items-center gap-2">
          繰り返し
          <input
            type="number"
            min={1}
            max={MAX_REPEAT}
            value={value.repeat}
            disabled={disabled}
            onChange={(e) =>
              set({
                repeat: Math.min(
                  MAX_REPEAT,
                  Math.max(1, Math.floor(Number(e.target.value) || 1)),
                ),
              })
            }
            className="w-16 bg-gray-700 text-gray-100 rounded px-2 py-0.5 disabled:opacity-50"
          />
          回
        </label>
      </div>

      <label className="block">
        <span className="text-xs text-gray-400">
          実機PCのアドレス(任意。指定するとナビゲーションを実機PCで動かす)
        </span>
        <input
          value={value.robot}
          disabled={disabled || value.attach}
          onChange={(e) => set({ robot: e.target.value.trim() })}
          placeholder={
            value.attach ? "attach では使えません" : "例: 192.168.0.10"
          }
          className={`mt-1 w-full bg-gray-700 text-gray-100 rounded px-2 py-1 disabled:opacity-50 ${
            robotValid ? "" : "ring-1 ring-red-500"
          }`}
        />
        {!robotValid && (
          <span className="text-xs text-red-400">
            IP アドレスまたはホスト名を入力してください
          </span>
        )}
      </label>
    </div>
  );
}
