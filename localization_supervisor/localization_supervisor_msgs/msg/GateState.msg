# ゲートの調停ノード (gate_arbiter) が配信する状態

# 要求元の意図 (すべての要求元が開けてよいときだけ true)
bool desired

# ゲートに反映済みか (ゲートのノードの再起動などで不明になったときは false)
bool applied

# ゲートを閉じている要求元
string[] holders
