import EntryStatusBadge from "./EntryStatusBadge";

interface PlannedRunListProps {
  names: readonly string[];
  repeat: number;
  /** シナリオ名 → 説明 */
  descriptions: ReadonlyMap<string, string>;
}

/** テストの起動中(進捗ファイルがまだない間)に、これから実行するシナリオを順に出す */
export default function PlannedRunList({
  names,
  repeat,
  descriptions,
}: PlannedRunListProps) {
  return (
    <div className="space-y-2">
      <p className="rounded-lg border-l-4 border-accent bg-surface-sunken px-3 py-2 text-sm">
        テストを起動しています… 実行予定 {names.length} 本
        {repeat > 1 && `(各 ${repeat} 回)`}
      </p>
      <ol className="space-y-1 text-sm">
        {names.map((name, i) => (
          <li
            key={name}
            title={descriptions.get(name)}
            className="flex items-center gap-2"
          >
            <span className="w-6 text-right text-xs tabular-nums text-muted">
              {i + 1}
            </span>
            <EntryStatusBadge status="PENDING" />
            <span className="break-all">{name}</span>
          </li>
        ))}
      </ol>
    </div>
  );
}
