#!/usr/bin/env python3
"""Reproduce RobotBridge4's style-stage equivalence and scope checks."""

from __future__ import annotations

import argparse
import ast
import difflib
import hashlib
import json
import re
import subprocess
import sys
from html.parser import HTMLParser
from pathlib import Path

SNAPSHOT = "e85fcae"
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
BASELINE_ROOT = Path("/home/yhl/Desktop/BAAI-Humanoid/RobotBridge")
AUDIT_PATH = REPOSITORY_ROOT / "docs/refactor_20260909/style_audit.json"
COMPARISON_PATH = REPOSITORY_ROOT / "docs/refactor_20260909/baseline_comparison.json"

CPP_PUNCTUATORS = sorted(
    [
        "%:%:",
        ">>=",
        "<<=",
        "<=>",
        "->*",
        "...",
        "##",
        "::",
        ".*",
        "->",
        "++",
        "--",
        "<<",
        ">>",
        "<=",
        ">=",
        "==",
        "!=",
        "&&",
        "||",
        "*=",
        "/=",
        "%=",
        "+=",
        "-=",
        "&=",
        "^=",
        "|=",
        "<:",
        ">:",
        "<%",
        "%>",
        "%:",
    ],
    key=len,
    reverse=True,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--revision",
        help="Read post-style files from this Git revision instead of the working tree.",
    )
    return parser.parse_args()


def git_bytes(revision: str, path: str) -> bytes:
    return subprocess.run(
        ["git", "show", f"{revision}:{path}"],
        cwd=REPOSITORY_ROOT,
        check=True,
        stdout=subprocess.PIPE,
    ).stdout


def post_style_bytes(path: str, revision: str | None) -> bytes:
    if revision is not None:
        return git_bytes(revision, path)
    return (REPOSITORY_ROOT / path).read_bytes()


def sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def cpp_tokens(source: str) -> list[tuple[str, str]]:
    """Lex non-comment C++ tokens, retaining macros and literal spelling."""
    source = source.replace("\\\r\n", "").replace("\\\n", "")
    tokens: list[tuple[str, str]] = []
    index = 0
    while index < len(source):
        if source[index].isspace():
            index += 1
            continue
        if source.startswith("//", index):
            end = source.find("\n", index + 2)
            index = len(source) if end < 0 else end + 1
            continue
        if source.startswith("/*", index):
            end = source.find("*/", index + 2)
            if end < 0:
                raise ValueError("unterminated C++ block comment")
            index = end + 2
            continue

        match = re.match(r'(?:u8|u|U|L)?R"([^ ()\\\t\r\n]{0,16})\(', source[index:])
        if match:
            terminator = ")" + match.group(1) + '"'
            end = source.find(terminator, index + match.end())
            if end < 0:
                raise ValueError("unterminated C++ raw string")
            end += len(terminator)
            tokens.append(("literal", source[index:end]))
            index = end
            continue

        match = re.match(r'(?:u8|u|U|L)?(["\'])', source[index:])
        if match:
            quote = match.group(1)
            end = index + match.end()
            escaped = False
            while end < len(source):
                character = source[end]
                if escaped:
                    escaped = False
                elif character == "\\":
                    escaped = True
                elif character == quote:
                    end += 1
                    break
                end += 1
            else:
                raise ValueError("unterminated C++ literal")
            tokens.append(("literal", source[index:end]))
            index = end
            continue

        match = re.match(r"[A-Za-z_$][A-Za-z0-9_$]*", source[index:])
        if match:
            value = match.group(0)
            tokens.append(("identifier", value))
            index += len(value)
            continue

        if source[index].isdigit() or (
            source[index] == "." and index + 1 < len(source) and source[index + 1].isdigit()
        ):
            end = index + 1
            while end < len(source):
                character = source[end]
                if character.isalnum() or character in "_'.":
                    end += 1
                    continue
                if character in "+-" and source[end - 1] in "eEpP":
                    end += 1
                    continue
                break
            tokens.append(("number", source[index:end]))
            index = end
            continue

        punctuator = next((item for item in CPP_PUNCTUATORS if source.startswith(item, index)), None)
        if punctuator is not None:
            tokens.append(("punctuator", punctuator))
            index += len(punctuator)
            continue
        tokens.append(("punctuator", source[index]))
        index += 1
    return tokens


def check_protected_regions(path: str, after: bytes) -> tuple[int, int, int]:
    baseline = (BASELINE_ROOT / path).read_text().splitlines(keepends=True)
    before = git_bytes(SNAPSHOT, path).decode().splitlines(keepends=True)
    post_style = after.decode().splitlines(keepends=True)
    initial_ops = difflib.SequenceMatcher(None, baseline, before, autojunk=False).get_opcodes()
    mutable: set[int] = set()
    protected: list[tuple[int, int]] = []
    for tag, _i1, _i2, j1, j2 in initial_ops:
        if tag == "equal":
            protected.append((j1, j2))
        else:
            mutable.update(range(j1, j2))

    out_of_scope = 0
    for tag, i1, i2, _j1, _j2 in difflib.SequenceMatcher(None, before, post_style, autojunk=False).get_opcodes():
        if tag == "equal":
            continue
        if i1 < i2 and any(index not in mutable for index in range(i1, i2)):
            out_of_scope += 1
        if i1 == i2:
            left_mutable = i1 > 0 and i1 - 1 in mutable
            right_mutable = i1 < len(before) and i1 in mutable
            if not (left_mutable or right_mutable):
                out_of_scope += 1

    result = "".join(post_style)
    cursor = 0
    missing = 0
    for j1, j2 in protected:
        block = "".join(before[j1:j2])
        position = result.find(block, cursor)
        if position < 0:
            missing += 1
        else:
            cursor = position + len(block)
    return len(protected), out_of_scope, missing


def main() -> int:
    args = parse_args()
    audit = json.loads(AUDIT_PATH.read_text())
    comparison = json.loads(COMPARISON_PATH.read_text())
    failures: list[str] = []

    # Resolve the immutable snapshot without coupling the audit to current HEAD.
    resolved_snapshot = subprocess.run(
        ["git", "rev-parse", SNAPSHOT],
        cwd=REPOSITORY_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout.strip()
    if audit["pre_style_snapshot_commit"] != resolved_snapshot:
        failures.append("audit snapshot commit does not resolve to e85fcae")

    records = audit["files"]
    if len(records) != 73 or len({record["path"] for record in records}) != 73:
        failures.append("audit does not contain 73 unique review records")

    python_count = 0
    cpp_count = 0
    for record in records:
        path = record["path"]
        before = git_bytes(SNAPSHOT, path)
        after = post_style_bytes(path, args.revision)
        if sha256(before) != record["before_sha256"]:
            failures.append(f"{path}: pre-style SHA256 differs from audit")
        if sha256(after) != record["after_sha256"]:
            failures.append(f"{path}: post-style SHA256 differs from audit")
        if record["language"] == "python":
            python_count += 1
            compile(after, path, "exec", dont_inherit=True)
            before_ast = ast.dump(
                ast.parse(before.decode(), filename=path, type_comments=True, feature_version=(3, 10)),
                include_attributes=False,
            )
            after_ast = ast.dump(
                ast.parse(after.decode(), filename=path, type_comments=True, feature_version=(3, 10)),
                include_attributes=False,
            )
            if before_ast != after_ast:
                failures.append(f"{path}: Python AST differs")
        elif record["language"] == "cpp":
            cpp_count += 1
            if cpp_tokens(before.decode()) != cpp_tokens(after.decode()):
                failures.append(f"{path}: C++ token sequence differs")
        elif before != after:
            failures.append(f"{path}: reviewed-unchanged bytes differ")

    for path in ("deploy/simulator/mujoco.py", "deploy/simulator/real_world.py"):
        equal_blocks, out_of_scope, missing = check_protected_regions(path, post_style_bytes(path, args.revision))
        if out_of_scope or missing:
            failures.append(
                f"{path}: protected-region failure: equal_blocks={equal_blocks}, "
                f"out_of_scope={out_of_scope}, missing={missing}"
            )

    # Baseline-identical paths and non-simulator baseline differences are outside style ownership.
    protected_outside_scope = []
    for key in ("same", "changed"):
        for item in comparison[key]:
            path = item if isinstance(item, str) else item["path"]
            if path in {"deploy/simulator/mujoco.py", "deploy/simulator/real_world.py"}:
                continue
            protected_outside_scope.append(path)
    for path in protected_outside_scope:
        before = git_bytes(SNAPSHOT, path)
        after = post_style_bytes(path, args.revision)
        if before != after:
            failures.append(f"{path}: out-of-scope baseline path changed")

    shell_paths = [record["path"] for record in records if record["language"] == "shell"]
    for path in shell_paths:
        if args.revision is None:
            subprocess.run(["bash", "-n", path], cwd=REPOSITORY_ROOT, check=True)
        else:
            subprocess.run(["bash", "-n"], input=post_style_bytes(path, args.revision), check=True)

    html_path = next(record["path"] for record in records if record["language"] == "html")
    parser = HTMLParser(convert_charrefs=True)
    parser.feed(post_style_bytes(html_path, args.revision).decode())
    parser.close()

    result = {
        "revision": args.revision or "working-tree",
        "snapshot": SNAPSHOT,
        "review_records": len(records),
        "python_ast_files": python_count,
        "cpp_token_files": cpp_count,
        "shell_syntax_files": len(shell_paths),
        "html_parse_files": 1,
        "protected_outside_scope_files": len(protected_outside_scope),
        "failures": failures,
    }
    print(json.dumps(result, indent=2))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
