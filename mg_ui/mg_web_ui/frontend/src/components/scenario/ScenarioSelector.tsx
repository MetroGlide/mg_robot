import { useMemo, useState } from "react";
import type { ScenarioInfo } from "../../types/scenarioTest";
import {
  collectGroups,
  collectTags,
  filterScenarios,
} from "../../utils/scenarioTest";

interface ScenarioSelectorProps {
  scenarios: readonly ScenarioInfo[];
  selected: ReadonlySet<string>;
  onChange: (next: Set<string>) => void;
  onReload: () => void;
  disabled?: boolean;
}

export default function ScenarioSelector({
  scenarios,
  selected,
  onChange,
  onReload,
  disabled = false,
}: ScenarioSelectorProps) {
  const [group, setGroup] = useState("");
  const [tag, setTag] = useState("");
  const [showKnownIssues, setShowKnownIssues] = useState(false);

  const groups = useMemo(() => collectGroups(scenarios), [scenarios]);
  const tags = useMemo(() => collectTags(scenarios), [scenarios]);
  const visible = useMemo(
    () => filterScenarios(scenarios, { group, tag, showKnownIssues }),
    [scenarios, group, tag, showKnownIssues],
  );

  const toggle = (name: string) => {
    const next = new Set(selected);
    if (next.has(name)) next.delete(name);
    else next.add(name);
    onChange(next);
  };

  const selectVisible = () => {
    onChange(new Set([...selected, ...visible.map((s) => s.name)]));
  };

  const selectClass =
    "bg-gray-700 text-gray-100 text-xs rounded px-2 py-1 disabled:opacity-50";

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap items-center gap-2">
        <select
          value={group}
          onChange={(e) => setGroup(e.target.value)}
          className={selectClass}
        >
          <option value="">すべてのグループ</option>
          {groups.map((g) => (
            <option key={g} value={g}>
              {g}
            </option>
          ))}
        </select>
        <select
          value={tag}
          onChange={(e) => setTag(e.target.value)}
          className={selectClass}
        >
          <option value="">すべてのタグ</option>
          {tags.map((t) => (
            <option key={t} value={t}>
              {t}
            </option>
          ))}
        </select>
        <label className="flex items-center gap-1 text-xs text-gray-300 select-none">
          <input
            type="checkbox"
            checked={showKnownIssues}
            onChange={(e) => setShowKnownIssues(e.target.checked)}
          />
          known_issue も表示
        </label>
      </div>

      <div className="flex items-center gap-3 text-xs">
        <button
          type="button"
          onClick={selectVisible}
          disabled={disabled || visible.length === 0}
          className="text-blue-300 hover:text-blue-200 disabled:opacity-50"
        >
          表示中をすべて選択
        </button>
        <button
          type="button"
          onClick={() => onChange(new Set())}
          disabled={disabled || selected.size === 0}
          className="text-gray-300 hover:text-white disabled:opacity-50"
        >
          選択を解除
        </button>
        <button
          type="button"
          onClick={onReload}
          className="text-gray-300 hover:text-white"
        >
          一覧を再読み込み
        </button>
        <span className="ml-auto text-gray-400">{selected.size} 件選択</span>
      </div>

      <ul className="max-h-80 overflow-y-auto divide-y divide-gray-700 rounded bg-gray-900">
        {visible.length === 0 && (
          <li className="px-2 py-3 text-xs text-gray-500">
            シナリオがありません(system_manager
            に接続できているか確認してください)
          </li>
        )}
        {visible.map((s) => (
          <li key={`${s.group}/${s.name}`}>
            <label
              className="flex items-start gap-2 px-2 py-1.5 hover:bg-gray-800 cursor-pointer"
              title={s.description || s.error}
            >
              <input
                type="checkbox"
                className="mt-0.5"
                checked={selected.has(s.name)}
                onChange={() => toggle(s.name)}
                disabled={disabled}
              />
              <span className="min-w-0 flex-1">
                <span className="block text-sm text-gray-100 break-all">
                  {s.name}
                  <span className="ml-2 text-[10px] text-gray-500">
                    {s.group}
                  </span>
                </span>
                {s.error ? (
                  <span className="block text-[10px] text-red-400">
                    YAML を読めません: {s.error}
                  </span>
                ) : (
                  <span className="flex flex-wrap gap-1 mt-0.5">
                    {s.tags.map((t) => (
                      <span
                        key={t}
                        className="rounded bg-gray-700 px-1 text-[10px] text-gray-300"
                      >
                        {t}
                      </span>
                    ))}
                  </span>
                )}
              </span>
            </label>
          </li>
        ))}
      </ul>
    </div>
  );
}
