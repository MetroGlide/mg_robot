// system_manager の /scenario/ API と、sim_scenario_test が書く progress.json・result.json の型

export type EntryStatus = "PENDING" | "RUNNING" | "PASSED" | "FAILED" | "ERROR";

export type RunState = "running" | "finished" | "aborted";

export interface ScenarioInfo {
  name: string;
  /** シナリオを置いたディレクトリ名(regression / examples など) */
  group: string;
  tags: string[];
  description: string;
  /** YAML を読めなかった場合のエラー */
  error?: string;
}

export interface RunOptions {
  attach?: boolean;
  gui?: boolean;
  remote_stack?: string;
  repeat?: number;
}

export interface ProgressEntry {
  name: string;
  file: string;
  /** run のディレクトリからの相対パス。結果・ログの取得に使う */
  result_dir: string;
  status: EntryStatus;
  message: string;
  elapsed_sec: number;
  attempt: number;
  started_at?: string;
}

export interface RunProgress {
  run_id: string;
  state: RunState;
  started_at: string;
  options: RunOptions;
  entries: ProgressEntry[];
  finished: boolean;
  exit_code: number | null;
}

export interface RunSummary {
  run_id: string;
  started_at: string;
  state: RunState;
  exit_code: number | null;
  total: number;
  counts: Partial<Record<EntryStatus, number>>;
  options: RunOptions;
}

export interface ScenarioStatus {
  /** scenario-test / scenario-env / gazebo-simulation のコンテナの状態 */
  services: Record<string, string | null>;
  run: RunProgress | null;
}

export interface ScenarioCheck {
  name: string;
  status: string;
  message: string;
}

export interface ScenarioResultData {
  scenario_name: string;
  status: string;
  elapsed_wall_sec: number;
  elapsed_sim_sec: number;
  goals: {
    reached: number;
    total: number;
    failed_index: number;
    failure: string | null;
  };
  checks: ScenarioCheck[];
  events: Record<string, unknown>[];
}

export interface RunRequest {
  scenarios: string[];
  repeat: number;
  gui: boolean;
  /** 起動済みの環境(scenario-env など)に接続して実行する */
  attach: boolean;
  /** 実機PCのアドレス。空なら開発PCだけで実行する */
  robot: string;
}

export type ScenarioLogName = "launch" | "stack";
