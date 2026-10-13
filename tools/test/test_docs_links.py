"""ドキュメントのリンクと、コードや設定からの文書への参照が実在することを検査する。

文書を移動・改名したときの参照の切れを見つけるためのもの (規約: doc/documentation_guide.md)。
"""
import os
from pathlib import Path
import re
from urllib.parse import unquote

import pytest

ROOT = Path(__file__).resolve().parents[2]

# 上流の取り込み (nav2_pkg、geometry2_pkg) は、ルートの README だけを検査する
EXCLUDED_DIRS = {
    '.git', 'node_modules', 'nav2_pkg', 'geometry2_pkg', '__pycache__', '.pytest_cache',
    '.claude', 'build', 'install', 'log', 'data', 'dist'}
VENDORED_ROOT_READMES = [ROOT / 'nav2_pkg' / 'README.md', ROOT / 'geometry2_pkg' / 'README.md']
MAX_BYTES = 1024 * 1024

LINK = re.compile(r'(?<!\!)\[[^\]]*\]\(([^)\s]+)(?:\s+"[^"]*")?\)')
DOC_REFERENCE = re.compile(r'[\w./-]*\bdoc/[\w-]+\.md\b')
HEADING = re.compile(r'^\s{0,3}#{1,6}\s+(.*?)\s*#*\s*$')
FENCE = re.compile(r'^\s*(```|~~~)')


def _walk_files():
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
        for name in filenames:
            path = Path(dirpath) / name
            if path.is_symlink() or path.stat().st_size > MAX_BYTES:
                continue
            yield path


def _read_text(path):
    try:
        return path.read_text(encoding='utf-8')
    except UnicodeDecodeError:
        return None


def _all_text_files():
    files = []
    for path in list(_walk_files()) + VENDORED_ROOT_READMES:
        text = _read_text(path)
        if text is not None:
            files.append((path, text))
    return files


def _markdown_files():
    return [(p, t) for p, t in _all_text_files() if p.suffix == '.md']


def _lines_outside_code_fences(text):
    in_fence = False
    for line in text.splitlines():
        if FENCE.match(line):
            in_fence = not in_fence
            continue
        if not in_fence:
            yield line


def github_slug(heading):
    """GitHub が見出しから作るアンカー名。"""
    text = re.sub(r'\[([^\]]*)\]\([^)]*\)', r'\1', heading)
    text = re.sub(r'[`*]', '', text).strip().lower()
    text = re.sub(r'[^\w\- ]', '', text)
    return text.replace(' ', '-')


def anchors_of(text):
    anchors = set()
    counts = {}
    for line in _lines_outside_code_fences(text):
        match = HEADING.match(line)
        if not match:
            continue
        slug = github_slug(match.group(1))
        n = counts.get(slug, 0)
        counts[slug] = n + 1
        anchors.add(slug if n == 0 else f'{slug}-{n}')
    return anchors


def _rel(path):
    return path.relative_to(ROOT).as_posix()


def test_github_slug_matches_existing_links():
    assert github_slug('衝突の防止 (`collision_monitor`)') == '衝突の防止-collision_monitor'
    assert github_slug('3. 値の関係') == '3-値の関係'
    assert anchors_of('# A\n\n```\n# not heading\n```\n## A\n') == {'a', 'a-1'}


def test_markdown_links_resolve():
    markdown = _markdown_files()
    anchor_cache = {path: anchors_of(text) for path, text in markdown}
    broken = []
    for path, text in markdown:
        for line in _lines_outside_code_fences(text):
            line = re.sub(r'`[^`]*`', '', line)
            for target in LINK.findall(line):
                if re.match(r'^[a-z][a-z0-9+.-]*:', target):
                    continue
                file_part, _, fragment = target.partition('#')
                dest = path if not file_part else (path.parent / unquote(file_part)).resolve()
                if not dest.exists():
                    broken.append(f'{_rel(path)}: {target} (ファイルがない)')
                elif fragment and dest.suffix == '.md':
                    if dest not in anchor_cache:
                        anchor_cache[dest] = anchors_of(dest.read_text(encoding='utf-8'))
                    if unquote(fragment).lower() not in anchor_cache[dest]:
                        broken.append(f'{_rel(path)}: {target} (アンカーがない)')
    assert not broken, '\n'.join(broken)


def _reference_exists(source, reference):
    # リポジトリのルート、または参照元のファイルを含むいずれかのディレクトリからの相対パス
    base = source.parent
    while True:
        if (base / reference).resolve().is_file():
            return True
        if base == ROOT or ROOT not in base.parents:
            return False
        base = base.parent


def test_references_to_docs_from_any_file_resolve():
    broken = []
    for path, text in _all_text_files():
        if path.suffix == '.md':
            continue
        for reference in DOC_REFERENCE.findall(text):
            if not _reference_exists(path, reference):
                broken.append(f'{_rel(path)}: {reference}')
    assert not broken, '\n'.join(broken)
