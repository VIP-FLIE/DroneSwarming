---
name: coding-standards
description: Coding standards to apply to all code written or modified. Prioritizes correctness, safety, efficiency, and minimality, with rules for type annotations, self-documenting code, and following existing codebase conventions.
---

Coding standards for this task. Apply them to all code you write or modify.

Priorities, in order:
1. Correctness. The code does what the task requires, including edge cases and error paths.
2. Safety. No memory-unsafe constructs, no undefined behavior, no unchecked input at trust boundaries, no hardcoded secrets, no injection vectors. Release all resources (files, sockets, locks, allocations).
3. Efficiency. Use appropriate algorithms and data structures. Avoid needless allocations, copies, and repeated work. Do not micro-optimize without a measured reason.
4. Minimality. Write the least code that fully solves the problem.

Minimality rules:
- Do not add features, options, configuration, or generality the task did not request.
- Do not add wrapper functions, classes, or layers that only forward calls.
- Add an abstraction only when it makes the code clearer or removes real duplication.
- Reuse existing functions, libraries, and project conventions before writing new code.
- Remove dead code, unused imports, and unused variables.
- Minimal code does not permit shortcuts. Do not skip error handling, validation, or edge cases to save lines. Do not stub, hardcode, or special-case results to make tests pass.

Type annotations:
- Annotate every function parameter and return type.
- Annotate variables, fields, and constants whose type is not obvious from the assignment.
- Use the most specific type the language supports. Avoid escape hatches (any, Object, void*, unchecked casts) unless unavoidable, and comment each one.
- The code must pass the project's type checker or compiler under strict settings.

Self-documenting code:
- Names state intent. Variables, functions, classes, methods, and modules use full words that describe what the thing is or does.
- Each function does one thing and is named for it. Booleans read as predicates (is_, has_, should_).
- Avoid abbreviations, single-letter names (except conventional loop indices), and vague names such as data, temp, info, manager, helper.
- Replace magic numbers and strings with named constants.
- Write a comment only where the code cannot explain itself: a non-obvious reason, constraint, invariant, or workaround. Comments state why. They do not restate what the code does. Before writing a comment, try renaming or restructuring the code.

Follow the existing style and conventions of the codebase where they exist.

When finished, list: assumptions made, requirements not fully met, and any place where these standards conflicted with each other or with the task.