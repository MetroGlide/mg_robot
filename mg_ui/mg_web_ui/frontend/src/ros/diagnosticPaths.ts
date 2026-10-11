/**
 * /diagnostics_agg(diagnostic_aggregator が集約した診断)のパスのうち、UI が個別に参照するもの。
 * 規約は doc/monitoring.md「集約のパス」。ここと文書の 2 か所にだけ書く。
 */
export const DIAG_ROOT = "/Robot";

/** 最上位の区分(DIAG_ROOT の直下) */
export const DIAG_REGIONS = [
  { id: "Drivers", label: "ドライバ" },
  { id: "Localization", label: "自己位置" },
  { id: "Navigation", label: "ナビ" },
  { id: "Mapping", label: "地図作成" },
  { id: "System", label: "システム" },
  { id: "Other", label: "その他" },
] as const;

export const DIAG_PATHS = {
  /** GNSS の受信と測位の品質(実機のみ) */
  GNSS: `${DIAG_ROOT}/Drivers/GNSS`,
  /** 自己位置推定の監視(watchdog か supervisor のどちらか) */
  LOCALIZATION: `${DIAG_ROOT}/Localization/Monitor`,
  /** Nav2 の lifecycle(Nav2 Health) */
  NAV2: `${DIAG_ROOT}/Navigation/Nav2`,
  /** CPU とメモリ(pc_resource_publisher_node の診断 cpu / memory) */
  CPU: `${DIAG_ROOT}/System/Resources/pc_resource_publisher_node: cpu`,
  MEMORY: `${DIAG_ROOT}/System/Resources/pc_resource_publisher_node: memory`,
} as const;
