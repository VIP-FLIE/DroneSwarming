---
name: docs-writing
description: "Apply a factual, declarative writing style to all prose. Use when writing or editing any document, README, doc, comment, or other written content. Enforces 11 rules: state facts, no narrative transitions, no foreshadowing, no editorializing, no hedging, no em dashes, no summaries."
---

# Docs Writing

Apply these rules to all writing you produce or edit.

1. State facts. Do not narrate process, motivation, or consequence unless the consequence is a requirement the reader must act on (e.g., a specific error, a specific failure mode with a specific cause).
2. No conjunctive transitions used for narrative effect (however, moreover, that said, at the end of the day, ultimately, in short).
3. No foreshadowing or warning language framed as story ("this is where it will bite you," "watch out," "the tricky part is"). State the constraint directly: "X requires Y" or "X fails if Y."
4. No "not X, but Y" or "it's not just X, it's Y" constructions. State the fact once.
5. No em dashes. Use periods or commas.
6. No adjectives or adverbs that editorialize about difficulty, importance, or elegance (crucial, powerful, simply, just, seamlessly, robust, elegant).
7. No hedging filler (essentially, basically, generally speaking) unless it changes meaning.
8. Every sentence must contain a verifiable factual claim: a name, a value, a behavior, a constraint, or an instruction. If a sentence can be deleted without losing information, delete it.
9. Prefer flat lists and short declarative sentences over paragraphs that connect ideas with narrative logic.
10. Explanations of "why" are allowed only when the reason changes what the reader must do (e.g., "The first three arguments are fixed by the simulator. Only the fourth is user code." is fine because it tells the reader where their responsibility starts and stops).
11. No summary or recap sentences at the end of a section restating what was just said.