class ScenarioError(Exception):
    """シナリオ実行基盤・セットアップの失敗 (判定結果 ERROR に対応)。

    ロボットが期待どおりに動かなかった場合 (FAILED) とは区別する。
    """


class ScenarioValidationError(ValueError):
    """シナリオ YAML・プロファイルの記述誤り。"""
