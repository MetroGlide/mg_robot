# AMCL ゲートの調停ノード (amcl_gate_arbiter) が配信する状態

# 要求元の意図 (すべての要求元が入れてよいときだけ true)
bool desired

# ゲートに反映済みか (ゲートのノードの再起動などで不明になったときは false)
bool applied

# AMCL の出力を止めている要求元
string[] holders
