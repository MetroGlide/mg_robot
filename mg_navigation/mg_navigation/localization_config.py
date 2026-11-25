"""自己位置推定の構成 (監視・監督の並走・門・プリセット・プロファイルの組) を、構成の名前 1 つから launch で決める。

個別の引数 (環境変数) が空なら構成の値を使い、書けばそちらが優先する。プリセットとプロファイルを
構成に関わらず外すときは none と書く (空は「構成に従う」)。
"""
from typing import Dict, Mapping

# 構成の名前 → launch の引数の値
#   legacy      実機で最後に走った構成 (タグ v2.1)。watchdog だけ
#   observe     legacy で走り、監督と門を判定と照合だけで並走させる (実機で誤った判定の率を測る)
#   supervised  最新の構成。監督 (門を入れる)、プリセット robust、監督のプロファイル robust_recovery
CONFIGS: Dict[str, Dict[str, str]] = {
    'legacy': {
        'localization_monitor': 'watchdog', 'localization_supervisor': 'false', 'localization_gate': 'false',
        'localization_preset': '', 'localization_profile': '',
    },
    'observe': {
        'localization_monitor': 'watchdog', 'localization_supervisor': 'true', 'localization_gate': 'observe',
        'localization_preset': '', 'localization_profile': '',
    },
    'supervised': {
        'localization_monitor': 'supervisor', 'localization_supervisor': 'false', 'localization_gate': 'true',
        'localization_preset': 'robust', 'localization_profile': 'robust_recovery',
    },
}
DEFAULT_CONFIG = 'supervised'
# プリセット・プロファイルを外す指定
NONE = 'none'
CLEARABLE = ('localization_preset', 'localization_profile')


def resolve(config: str, overrides: Mapping[str, str]) -> Dict[str, str]:
    """構成の名前と個別の指定から、launch の引数の値を決める。"""
    name = config or DEFAULT_CONFIG
    if name not in CONFIGS:
        raise RuntimeError(f'自己位置推定の構成は {" / ".join(CONFIGS)} のどれかです: {name}')
    values = dict(CONFIGS[name])
    for key, value in overrides.items():
        if key not in values:
            raise RuntimeError(f'構成で決めない引数です: {key}')
        if value:
            values[key] = '' if key in CLEARABLE and value == NONE else value
    return values
