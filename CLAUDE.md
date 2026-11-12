# CLAUDE.md — XarmAI rules (MANDATORY)

## 1. Talk = caveman. ALWAYS.
- Load `caveman:caveman` skill at start of EVERY session, first turn, before anything else.
- Every reply in caveman. No exceptions. Plan, recap, questions, wrap-up: all caveman.
- Short words. Short sentence. Max 3 bullets per list, ~5 words each.
- No filler, no praise, no long paragraphs. No tables unless user asks.
- Code, paths, commands, error names stay normal.
- Only stop if user says "stop caveman" / "normal mode".

## 2. Model ladder
- Main session: Sonnet 5.5 for build.
- Opus subagent: advice only, when truly needed.
- Sonnet subagent: build/code.
- Haiku subagent: git, download, install, file moves.
- Haiku fail → Sonnet → Opus.
- When user says "continue": tell which model to pick.
- After EACH set of file changes: Haiku subagent runs `graphify update .`
  (AST-only, free). Then read graph first (`graphify query`) next time.

## 3. Work rules
- Ask doubts. Never assume. Start after user decides.
- Sim window first (PyBullet), before real build.
- MATLAB only if truly needed.
- Follow `IMPLEMENTATION_PLAN.md` §0.1 rules.
