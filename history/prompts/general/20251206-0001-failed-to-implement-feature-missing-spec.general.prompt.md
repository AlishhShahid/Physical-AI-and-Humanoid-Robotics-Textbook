---
id: 20251206-0001
title: Failed to implement feature - missing spec
stage: general
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-humanoid-robotics-book
user: CLI_User
command: /sp.implement
labels: ["implementation","failure","spec-missing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- No files created or modified.
tests:
- No tests run or added.
---

## Prompt

$ARGUMENTS

## Response snapshot

The feature directory is missing. Please run `/sp.specify` first to create the feature structure.

## Outcome

- ✅ Impact: Low
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: Run `/sp.specify`
- 🧠 Reflection: User tried to run /sp.implement without a created feature specification, leading to an error about missing feature directory.

## Evaluation notes (flywheel)

- Failure modes observed: Missing prerequisite feature directory.
- Graders run and results (PASS/FAIL): FAIL
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure a clear message for missing feature spec.
