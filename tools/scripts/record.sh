#!/bin/bash
set -e

# スクリプトの配置ディレクトリを基準にパスを解決
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# デフォルト設定
mode="slam"
output_arg=""
topics_file=""
use_sim_time=false

show_help() {
    cat << EOF
使用方法: $(basename "$0") [オプション]

実機・シミュレータ環境で rosbag (MCAP形式) を記録します。

オプション:
  -m <mode>     記録モードを選択します (デフォルト: slam)
                slam | a : 軽量版 (SLAMGNSS2D & TF のみ、約0.28GB/30分)
                all  | c : 全センサ版 (SLAMGNSS2D & TF & RealSense RGB/Depth/IMU、約39GB/30分)
  -o <output>   出力ファイル/ディレクトリ名
                相対パス指定時は ROSBAG_PATH 配下に保存されます。
                未指定時は ROSBAG_PATH 配下に record_<mode>_YYYYMMDD_HHMMSS が生成されます。
  -f <file>     カスタムのトピックリストファイルを指定
  -s            シミュレーション時間 (--use-sim-time) を使用
  -h            このヘルプを表示

環境変数:
  ROSBAG_PATH   保存先ベースディレクトリ (例: /root/ros2_data/rosbag)
                未設定時はカレントディレクトリ (.) が使用されます。
EOF
}

# コマンドライン引数の解析
while getopts ":m:o:f:sh" opt; do
  case $opt in
    m)
      mode="$OPTARG"
      ;;
    o)
      output_arg="$OPTARG"
      ;;
    f)
      topics_file="$OPTARG"
      ;;
    s)
      use_sim_time=true
      ;;
    h)
      show_help
      exit 0
      ;;
    \?)
      echo "無効なオプション: -$OPTARG" >&2
      show_help >&2
      exit 1
      ;;
    :)
      echo "オプション -$OPTARG には引数が必要です" >&2
      exit 1
      ;;
  esac
done

# トピックリストファイルの決定
if [ -n "$topics_file" ]; then
    if [ ! -f "$topics_file" ]; then
        echo "エラー: 指定されたトピックファイルが見つかりません: $topics_file" >&2
        exit 1
    fi
else
    case "$mode" in
        slam|a|A)
            mode="slam"
            topics_file="${SCRIPT_DIR}/record_topics_slam.txt"
            ;;
        all|c|C|camera)
            mode="all"
            topics_file="${SCRIPT_DIR}/record_topics_all.txt"
            ;;
        *)
            echo "エラー: 未知のモードです: $mode (利用可能: slam, all)" >&2
            exit 1
            ;;
    esac

    if [ ! -f "$topics_file" ]; then
        # フォールバックとして record_topics.txt を確認
        if [ -f "${SCRIPT_DIR}/record_topics.txt" ]; then
            topics_file="${SCRIPT_DIR}/record_topics.txt"
        else
            echo "エラー: トピックファイルが見つかりません: $topics_file" >&2
            exit 1
        fi
    fi
fi

# 出力先パスの決定 (ROSBAG_PATH 対応)
base_dir="${ROSBAG_PATH:-.}"
mkdir -p "$base_dir"

timestamp=$(date +"%Y%m%d_%H%M%S")
if [ -n "$output_arg" ]; then
    if [[ "$output_arg" = /* ]]; then
        output_path="$output_arg"
    else
        output_path="${base_dir}/${output_arg}"
    fi
else
    output_path="${base_dir}/record_${mode}_${timestamp}"
fi

# トピックリストの読み込み (空行・コメント行を除外)
topics=()
while IFS= read -r line || [ -n "$line" ]; do
    # 行前後の空白を除去
    trimmed=$(echo "$line" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')
    # 空行または # から始まる行をスキップ
    if [ -n "$trimmed" ] && [[ ! "$trimmed" =~ ^# ]]; then
        topics+=("$trimmed")
    fi
done < "$topics_file"

if [ ${#topics[@]} -eq 0 ]; then
    echo "エラー: 記録対象のトピックが存在しません: $topics_file" >&2
    exit 1
fi

# ros2 bag record コマンドの構築
record_cmd=("ros2" "bag" "record" "-s" "mcap" "-o" "$output_path")
if [ "$use_sim_time" = true ]; then
    record_cmd+=("--use-sim-time")
fi
record_cmd+=("${topics[@]}")

# 実行情報表示
echo "=================================================="
echo " [mg_robot] rosbag 記録開始"
echo " モード        : $mode"
echo " トピックファイル: $topics_file"
echo " 保存先        : $output_path"
echo " トピック数    : ${#topics[@]}"
echo " 記録トピック  :"
for topic in "${topics[@]}"; do
    echo "   - $topic"
done
echo "=================================================="

# プロセスを ros2 bag record に置換して実行 (Ctrl+C 正常終了を担保)
exec "${record_cmd[@]}"
