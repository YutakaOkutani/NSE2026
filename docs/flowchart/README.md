# フローチャート

このディレクトリでは、CanSat の制御アルゴリズムを D2 記法の `.d2` ファイルで管理する。

D2 はテキストから図を生成するための記法・CLI ツールで、ノードと矢印をコードとして書ける。図の意味をレビューしやすく、変更履歴も Git の差分として追いやすい。

## 管理方針

- 編集対象は `.d2` ファイルとする。
- `.svg` や `.pdf` は `.d2` から再生成できる生成物として扱う。
- 画像や PDF が提出物として必要な場合だけ、必要な形式を生成して共有する。

## セットアップ

公式の推奨手順では、インストールスクリプトを使う。実行内容を先に確認したい場合は `--dry-run` を付ける。

```bash
curl -fsSL https://d2lang.com/install.sh | sh -s -- --dry-run
curl -fsSL https://d2lang.com/install.sh | sh -s --
```

インストール後、次のコマンドで確認する。

```bash
d2 version
```

macOS で Homebrew を使う場合は次でもよい。

```bash
brew install d2
```

Windows では WSL 上で Linux と同じ手順を使うのが扱いやすい。ネイティブ環境では Scoop や Chocolatey でもインストールできる。

詳しい手順は D2 公式ドキュメントを参照する。

- <https://d2lang.com/tour/install/>
- <https://github.com/terrastruct/d2/blob/master/docs/INSTALL.md>

## D2 記法の基本

ノードは `id: "表示名"`、接続は `a -> b` のように書く。

```d2
direction: down

start: "START"
check: "条件判定 ?" { shape: diamond }
action: "処理"
end: "END" { shape: oval }

start -> check
check -> action: "Yes"
check -> end: "No"
action -> end
```

このリポジトリのフローチャートでは、次のような書き方を基本にしている。

- `title` で図のタイトルを付ける。
- `direction: down` で上から下へ流す。
- `classes` でフェーズ、判定、処理、開始・終了の見た目をそろえる。
- 条件分岐は `shape: diamond` を使う。
- 矢印ラベルに `Yes` / `No` や保持時間などを書く。

## 生成方法

`.d2` から SVG を生成する。

```bash
d2 docs/flowchart/cansat_control_algorithm_01a_release_detection.d2
```

出力先を明示する場合:

```bash
d2 docs/flowchart/cansat_control_algorithm_01a_release_detection.d2 \
  docs/flowchart/cansat_control_algorithm_01a_release_detection.svg
```

PDF を生成する場合:

```bash
d2 docs/flowchart/cansat_control_algorithm_01a_release_detection.d2 \
  docs/flowchart/cansat_control_algorithm_01a_release_detection.pdf
```

編集中にブラウザで確認したい場合:

```bash
d2 --watch docs/flowchart/cansat_control_algorithm_01a_release_detection.d2
```

`docs/flowchart` 配下の `.d2` をまとめて SVG にする場合:

```bash
for file in docs/flowchart/*.d2; do
  d2 "$file" "${file%.d2}.svg"
done
```

## 追加・更新の流れ

1. 既存の `.d2` をコピーするか、新しい `.d2` を作る。
2. フェーズ名、判定条件、タイムアウト、遷移先をコードと対応する名前で書く。
3. `d2 --watch` または SVG 生成で見た目を確認する。
4. Git には `.d2` と、この README などの説明文書をコミットする。
5. 生成した `.svg` / `.pdf` は、提出や共有に必要な場合だけ扱う。

