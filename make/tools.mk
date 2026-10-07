##@ ツール (rosbag の解析・可視化・評価。詳細: tools/README.md)
# 共通の引数:
#   BAG=<bag>        解析対象の rosbag (BAG_PATH でも可)。省略時は .env の ROSBAG_FILE
#   OPTS="..."       各スクリプトへの追加オプション
# 出力先の切り替え (既定は rosbag と同じディレクトリ):
#   TO_TOOLS=1       tools/data/ に出力する (OUT_DIR=tools でも可。Git 追跡対象外)
#   OUT_DIR=<dir>    指定ディレクトリに出力する
#   OUT=<file>       指定ファイルパスに出力する

.PHONY: bag-summary bag-plot-gnss bag-plot-gnss-map bag-plot-scans bag-eval-slam bag-eval-localization

define _resolve_bag_output_opts
$(strip \
  $(if $(OUT),-o "$(OUT)", \
    $(if $(filter 1 true,$(TO_TOOLS)$(TOOLS)),--output-dir /app/tools/data, \
      $(if $(filter tools,$(OUT_DIR)),--output-dir /app/tools/data, \
        $(if $(OUT_DIR),--output-dir "$(OUT_DIR)",--output-to-bag-dir) \
      ) \
    ) \
  ) \
)
endef

# develop コンテナで、対象 bag (BAG > BAG_PATH > ROSBAG_FILE) を解決してスクリプトを実行する。
# ROS 2 の環境は、コンテナの entrypoint (docker/ros_entrypoint.sh) が読み込む。
#   $(1): bag の解決後・スクリプト実行前に挟むシェルの断片 (空でよい。末尾は && で終える)
#   $(2): 実行するスクリプトと引数 (対象 bag はシェル変数 TARGET_BAG)
define _bag_run
$(COMPOSE) run --rm --no-deps $(if $(BAG),-e BAG="$(BAG)" )$(if $(BAG_PATH),-e BAG_PATH="$(BAG_PATH)" )develop bash -c \
  "TARGET_BAG=\"\$${BAG:-\$${BAG_PATH:-\$$ROSBAG_FILE}}\" && \
   if [ -z \"\$$TARGET_BAG\" ]; then \
     echo 'エラー: 解析対象の rosbag が指定されていません。.env に ROSBAG_FILE を設定するか、BAG=/path/to/bag を指定してください。' >&2; \
     exit 1; \
   fi && \
   $(1)$(2)"
endef

# rosbag の統計サマリー (通信の健全性・GNSS Fix 率と精度・オドメトリ積算距離) を summary.md に出力する。
#   BAG=<bag> / TO_TOOLS=1 / OPTS="--info-only"
bag-summary: ## rosbag の統計サマリーを出力 [BAG TO_TOOLS OPTS]
	$(call _bag_run,,python3 /app/tools/scripts/rosbag_summary.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) --all $(OPTS))

# GNSS 軌跡・Fix 状態・精度の可視化。
#   CIRCLES=1        精度円を描く (ACC_CIRCLES=1 でも可)
#   SCALE=<倍率>     精度円の倍率 (CIRCLE_SCALE=<倍率> でも可)
#   例: make bag-plot-gnss CIRCLES=1 SCALE=5 / make bag-plot-gnss BAG=/path OPTS="--circle-step 1"
bag-plot-gnss: ## GNSS 軌跡・Fix 状態・精度を画像化 [BAG CIRCLES SCALE TO_TOOLS OPTS]
	$(call _bag_run,,python3 /app/tools/scripts/plot_gnss_trajectory.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) $(if $(filter 1 true,$(CIRCLES)$(ACC_CIRCLES)),--accuracy-circles )$(if $(SCALE),--circle-scale $(SCALE) )$(if $(CIRCLE_SCALE),--circle-scale $(CIRCLE_SCALE) )$(OPTS))

# MAP_PATH/map_list.txt の地図群と ROSBAG_FILE の GNSS 位置・精度を、
# MAP_PATH/gnss_transform.yaml で map 座標に直して重ね描きする。
#   MAP_LIST=<file>        地図一覧 (既定: $MAP_PATH/map_list.txt)
#   GNSS_TRANSFORM=<file>  GNSS の変換 (既定: $MAP_PATH/gnss_transform.yaml)
#   CIRCLES=1 / SCALE=<倍率> / TO_TOOLS=1
bag-plot-gnss-map: ## 地図群に変換後の GNSS を重ね描き [MAP_LIST GNSS_TRANSFORM CIRCLES SCALE TO_TOOLS]
	$(call _bag_run,MAP_LIST_FILE=\"$(if $(MAP_LIST),$(MAP_LIST),\$$MAP_PATH/map_list.txt)\" && \
	   GNSS_TRANSFORM_FILE=\"$(if $(GNSS_TRANSFORM),$(GNSS_TRANSFORM),\$$MAP_PATH/gnss_transform.yaml)\" && \
	   ,python3 /app/tools/scripts/plot_gnss_trajectory.py \"\$$TARGET_BAG\" --mode map --map-list \"\$$MAP_LIST_FILE\" --gnss-transform \"\$$GNSS_TRANSFORM_FILE\" -o gnss_on_map.png $(_resolve_bag_output_opts) $(if $(filter 1 true,$(CIRCLES)$(ACC_CIRCLES)),--accuracy-circles )$(if $(SCALE),--circle-scale $(SCALE) )$(if $(CIRCLE_SCALE),--circle-scale $(CIRCLE_SCALE) )$(OPTS))

# LiDAR スキャン点群の 2D 画像化。
#   NODES=<範囲>  対象ノード (例: NODES=1:20)
bag-plot-scans: ## LiDAR スキャン点群を 2D 画像化 [BAG NODES TO_TOOLS]
	$(call _bag_run,,python3 /app/tools/scripts/plot_lidar_scans.py \"\$$TARGET_BAG\" $(_resolve_bag_output_opts) $(if $(NODES),--nodes $(NODES) )$(OPTS))

# SLAM 出力 (pose_graph.json / gnss_transform.yaml / map.yaml) を NavPVT と比較して評価する。
#   SLAM_DIR=<dir>  必須。SLAM の出力ディレクトリ
#   例: make bag-eval-slam SLAM_DIR=<dir> TO_TOOLS=1 OPTS="--time-offset 0.2"
bag-eval-slam: ## SLAM 出力を RTK(GNSS) と比較評価 [SLAM_DIR=<dir> BAG TO_TOOLS OPTS]
	$(call _bag_run,if [ -z \"$(SLAM_DIR)\" ]; then \
	     echo 'エラー: SLAM_DIR=<pose_graph.json と gnss_transform.yaml を含むディレクトリ> を指定してください。' >&2; \
	     exit 1; \
	   fi && \
	   ,python3 /app/tools/scripts/eval_slam.py \"\$$TARGET_BAG\" --slam-dir \"$(SLAM_DIR)\" $(_resolve_bag_output_opts) $(OPTS))

# 自己位置推定 (EKF 融合) の評価。実機のナビ走行 bag、または run_localization_variant.sh の再生出力 bag が対象。
#   GT_DIR=<dir>      真値 (pose_graph.json と gnss_transform.yaml を含む SLAM 出力)。省略可
#   MAP_GT_DIR=<dir>  別走行を評価するとき、地図を作った走行の SLAM 出力
#   FAULTS=<file>     注入した故障の定義 (faults.yaml)
#   例: make bag-eval-localization BAG=<bag> GT_DIR=<slam_dir> MAP_GT_DIR=<map_slam_dir> FAULTS=<faults.yaml>
bag-eval-localization: ## 自己位置推定 (EKF 融合) を評価 [BAG GT_DIR MAP_GT_DIR FAULTS TO_TOOLS]
	$(call _bag_run,,python3 /app/tools/scripts/eval_localization.py \"\$$TARGET_BAG\" $(if $(GT_DIR),--gt-dir \"$(GT_DIR)\" )$(if $(MAP_GT_DIR),--map-gt-dir \"$(MAP_GT_DIR)\" )$(if $(FAULTS),--faults \"$(FAULTS)\" )$(_resolve_bag_output_opts) $(OPTS))
