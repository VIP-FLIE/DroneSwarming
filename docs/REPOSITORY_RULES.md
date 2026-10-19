# Repository Rules

## Branches

- `main` contains the latest fully validated release and serves as the demo-ready version of the project.
- `dev` contains changes merged from feature and fix pull requests for the next release.
- Create each feature or fix branch from the current `dev` branch.
- Push work to its feature or fix branch and open a pull request targeting `dev`.
- Do not push directly to `main`.
- Promote `dev` to `main` through a release pull request.

## Release requirements for `main`

Merge a release pull request into `main` only when all of these requirements are met:

- At least two team members approve the release pull request.
- The full workspace builds in the project container with `colcon build --symlink-install`.
- The test suite passes with `colcon test`, and `colcon test-result --verbose` reports no failures.
- The documented SITL mission completes, and the vehicle lands without a crash.
- Reviewers confirm that the changes follow the requirements in [Contributing](../CONTRIBUTING.md).

`main` must compile, run, and remain suitable for a project demonstration. Record any check that was not run. Do not merge a release when a required check is incomplete or fails.

## Code and agent requirements

- Changes that do not follow the repository requirements are not accepted or merged.
- Apply the same requirements to human-written and LLM-generated changes.
- Read the [coding-standards skill](../skills/coding-standards/SKILL.md) before changing code.
- Read the [docs-writing skill](../skills/docs-writing/SKILL.md) before writing or editing documentation.
- Read [Contributing](../CONTRIBUTING.md) for the complete coding, safety, testing, and verification requirements.
