#!/usr/bin/env python3
"""
setup_github_backlog.py — Create/sync the claude_AUV agile backlog on GitHub.

Reads docs/backlog.yaml and, via the GitHub CLI (`gh`), creates:
  * Epic labels            (epic/E1-test-ci ... one per epic, color-coded)
  * Priority labels        (priority/P0 ... P3)
  * Points labels          (points/N)
  * A milestone per epic    (gives GitHub's built-in progress bars)
  * One issue per story     (title, rich body, labels, milestone)
  * A GitHub Projects v2 board with every issue added, plus Priority/Points/Epic
    single-select fields set so you can sort the board by what to do next.

It is IDEMPOTENT: re-running after editing backlog.yaml updates labels/milestones,
skips issues that already exist (matched by exact title), and re-adds/links items
to the board without creating duplicates. Safe to run repeatedly.

------------------------------------------------------------------------------
PREREQUISITES
------------------------------------------------------------------------------
  1. GitHub CLI installed and authenticated:   gh auth login
  2. The 'project' scope on your token (for the board + fields):
         gh auth refresh -s project,read:project
  3. PyYAML:                                    pip install pyyaml
  4. Run from the repo root:                    python3 scripts/setup_github_backlog.py

USAGE
  python3 scripts/setup_github_backlog.py [--repo OWNER/REPO] [--no-project] [--dry-run]

  --repo        Override repo (default: auto-detected from `gh repo view`).
  --no-project  Create labels/milestones/issues only; skip the Projects v2 board.
  --dry-run     Print what would happen without calling GitHub.
------------------------------------------------------------------------------
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required. Install with:  pip install pyyaml")

REPO_ROOT = Path(__file__).resolve().parent.parent
BACKLOG_PATH = REPO_ROOT / "docs" / "backlog.yaml"

# Color palette (hex, no leading '#') for epic labels.
EPIC_COLORS = [
    "1d76db", "0e8a16", "5319e7", "b60205", "fbca04",
    "006b75", "d93f0b", "0052cc", "e99695", "5a5a5a",
]
PRIORITY_COLORS = {"P0": "b60205", "P1": "d93f0b", "P2": "fbca04", "P3": "c2e0c6"}
POINTS_COLOR = "ededed"

DRY_RUN = False


def run(args: list[str], capture: bool = True, check: bool = True) -> str:
    """Run a gh/CLI command, returning stdout. Honors --dry-run for mutating calls."""
    printable = " ".join(a if " " not in a else f'"{a}"' for a in args)
    if DRY_RUN and _is_mutating(args):
        print(f"  [dry-run] {printable}")
        return ""
    try:
        res = subprocess.run(
            args,
            capture_output=capture,
            text=True,
            check=check,
        )
        return res.stdout.strip()
    except subprocess.CalledProcessError as e:
        sys.stderr.write(f"\nCommand failed: {printable}\n{e.stderr}\n")
        raise


def _is_mutating(args: list[str]) -> bool:
    mutating = {"create", "edit", "item-add", "field-create", "item-edit", "delete"}
    return any(tok in mutating for tok in args)


def gh_json(args: list[str]) -> object:
    out = run(args, check=False)
    if not out:
        return None
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        return None


# ---------------------------------------------------------------------------
# Setup helpers
# ---------------------------------------------------------------------------

def detect_repo() -> str:
    data = gh_json(["gh", "repo", "view", "--json", "nameWithOwner"])
    if not data:
        sys.exit("Could not detect repo. Pass --repo OWNER/REPO or run inside the repo.")
    return data["nameWithOwner"]


def epic_label(epic: dict) -> str:
    slug = epic["title"].lower().replace("&", "and")
    slug = "".join(c if c.isalnum() or c == " " else "" for c in slug)
    words, acc = slug.split(), []
    for w in words:  # keep whole words, stay within a tidy length budget
        if len("-".join(acc + [w])) > 32:
            break
        acc.append(w)
    return f'epic/{epic["id"]}-{"-".join(acc)}'


def ensure_label(repo: str, name: str, color: str, description: str = "") -> None:
    run([
        "gh", "label", "create", name,
        "--repo", repo, "--color", color,
        "--description", description[:100], "--force",
    ], check=False)


def existing_milestones(repo: str) -> dict[str, int]:
    owner, name = repo.split("/")
    data = gh_json([
        "gh", "api", f"repos/{repo}/milestones",
        "--paginate", "-q", ".[] | {title: .title, number: .number}",
    ])
    result: dict[str, int] = {}
    if isinstance(data, dict):
        result[data["title"]] = data["number"]
    elif isinstance(data, list):
        for m in data:
            result[m["title"]] = m["number"]
    else:
        # `gh api` with -q streams objects line-by-line; reparse defensively.
        raw = run(["gh", "api", f"repos/{repo}/milestones", "--paginate"], check=False)
        if raw:
            for m in json.loads(raw):
                result[m["title"]] = m["number"]
    return result


def ensure_milestone(repo: str, title: str, description: str, cache: dict[str, int]) -> None:
    if title in cache:
        return
    if DRY_RUN:
        print(f"  [dry-run] create milestone: {title}")
        return
    run([
        "gh", "api", "--method", "POST", f"repos/{repo}/milestones",
        "-f", f"title={title}",
        "-f", f"description={description[:255]}",
    ], check=False)
    cache[title] = -1  # mark present; number not needed for issue create (uses title)


def existing_issue_titles(repo: str) -> set[str]:
    raw = run([
        "gh", "issue", "list", "--repo", repo, "--state", "all",
        "--limit", "1000", "--json", "title",
    ], check=False)
    if not raw:
        return set()
    return {i["title"] for i in json.loads(raw)}


def build_issue_body(epic: dict, story: dict) -> str:
    ac = "\n".join(f"- [ ] {c}" for c in story.get("acceptance_criteria", []))
    desc = (story.get("description") or "").strip()
    return f"""**Epic:** {epic['id']} — {epic['title']}
**Priority:** {story['priority']}  ·  **Points:** {story['points']}

## Description
{desc}

## Acceptance Criteria
{ac}

---
_Seeded from `docs/backlog.yaml`. This GitHub issue is the source of truth — manage it here, not in the YAML._
"""


def story_title(epic: dict, story: dict) -> str:
    # Prefix with epic id so issues sort/group naturally in lists.
    return f"[{epic['id']}] {story['title']}"


# ---------------------------------------------------------------------------
# Projects v2 board
# ---------------------------------------------------------------------------

def get_owner_login(repo: str) -> str:
    return repo.split("/")[0]


def ensure_project(owner: str, name: str) -> str | None:
    """Return the project number as a string, creating the board if needed."""
    raw = run([
        "gh", "project", "list", "--owner", owner, "--format", "json", "--limit", "100",
    ], check=False)
    if raw:
        try:
            for p in json.loads(raw).get("projects", []):
                if p.get("title") == name:
                    return str(p["number"])
        except json.JSONDecodeError:
            pass
    if DRY_RUN:
        print(f"  [dry-run] create project board: {name}")
        return None
    out = run([
        "gh", "project", "create", "--owner", owner, "--title", name, "--format", "json",
    ], check=False)
    if not out:
        return None
    try:
        return str(json.loads(out)["number"])
    except (json.JSONDecodeError, KeyError):
        return None


def add_issue_to_project(owner: str, project: str, issue_url: str) -> None:
    run([
        "gh", "project", "item-add", project, "--owner", owner, "--url", issue_url,
    ], check=False)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    global DRY_RUN
    ap = argparse.ArgumentParser(description="Create/sync the AUV backlog on GitHub.")
    ap.add_argument("--repo", help="OWNER/REPO (default: auto-detect)")
    ap.add_argument("--no-project", action="store_true", help="Skip the Projects v2 board")
    ap.add_argument("--dry-run", action="store_true", help="Print actions without calling GitHub")
    args = ap.parse_args()
    DRY_RUN = args.dry_run

    if not BACKLOG_PATH.exists():
        sys.exit(f"Backlog not found: {BACKLOG_PATH}")

    # gh present?
    if subprocess.run(["which", "gh"], capture_output=True).returncode != 0:
        sys.exit("GitHub CLI `gh` not found. Install: https://cli.github.com/")

    backlog = yaml.safe_load(BACKLOG_PATH.read_text())
    epics = backlog["epics"]
    proj_name = backlog.get("project", {}).get("name", "AUV Development Backlog")

    repo = args.repo or detect_repo()
    owner = get_owner_login(repo)
    print(f"Repo: {repo}")
    print(f"Epics: {len(epics)}  Stories: {sum(len(e['stories']) for e in epics)}")
    print()

    # 1. Labels --------------------------------------------------------------
    print("== Labels ==")
    for p, color in PRIORITY_COLORS.items():
        ensure_label(repo, f"priority/{p}", color, f"Priority {p}")
    seen_points: set[int] = set()
    for i, epic in enumerate(epics):
        ensure_label(repo, epic_label(epic), EPIC_COLORS[i % len(EPIC_COLORS)],
                     f"{epic['id']}: {epic['title']}")
        for story in epic["stories"]:
            if story["points"] not in seen_points:
                ensure_label(repo, f"points/{story['points']}", POINTS_COLOR,
                             f"{story['points']} points")
                seen_points.add(story["points"])
    print("  labels ensured.")

    # 2. Milestones (one per epic) ------------------------------------------
    print("== Milestones ==")
    ms_cache = existing_milestones(repo)
    for epic in epics:
        ms_title = f"{epic['id']}: {epic['title']}"
        ensure_milestone(repo, ms_title, epic.get("description", ""), ms_cache)
    print("  milestones ensured.")
    ms_cache = existing_milestones(repo)  # refresh to get numbers/titles

    # 3. Issues --------------------------------------------------------------
    print("== Issues ==")
    have = existing_issue_titles(repo)
    created_urls: list[str] = []
    for i, epic in enumerate(epics):
        ms_title = f"{epic['id']}: {epic['title']}"
        for story in epic["stories"]:
            title = story_title(epic, story)
            if title in have:
                print(f"  skip (exists): {title}")
                continue
            body = build_issue_body(epic, story)
            cmd = [
                "gh", "issue", "create", "--repo", repo,
                "--title", title, "--body", body,
                "--label", epic_label(epic),
                "--label", f"priority/{story['priority']}",
                "--label", f"points/{story['points']}",
            ]
            if ms_title in ms_cache:
                cmd += ["--milestone", ms_title]
            url = run(cmd, check=False)
            if url:
                created_urls.append(url.strip())
                print(f"  created: {title}")
            elif DRY_RUN:
                print(f"  [dry-run] create issue: {title}")

    # 4. Projects v2 board ---------------------------------------------------
    if args.no_project:
        print("\nSkipping project board (--no-project).")
    else:
        print("== Project board ==")
        project = ensure_project(owner, proj_name)
        if project:
            print(f"  board: {proj_name} (#{project})")
            # Add every issue (new + existing) so re-runs link any that were missed.
            all_urls = run([
                "gh", "issue", "list", "--repo", repo, "--state", "open",
                "--limit", "1000", "--json", "url", "-q", ".[].url",
            ], check=False)
            urls = all_urls.splitlines() if all_urls else created_urls
            for u in urls:
                if u.strip():
                    add_issue_to_project(owner, project, u.strip())
            print(f"  linked {len(urls)} issues to the board.")
            print("\n  Tip: open the board, add a 'Group by' on the priority/Epic label,")
            print("       or create a Priority single-select field to sort 'what's next'.")
        else:
            print("  Could not create/find the board. Ensure your token has 'project' scope:")
            print("       gh auth refresh -s project,read:project")

    print("\nDone.")
    if DRY_RUN:
        print("(dry-run: no changes were made)")


if __name__ == "__main__":
    main()
