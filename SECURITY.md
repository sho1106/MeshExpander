# Security Policy

## Supported versions

MeshExpander is pre-1.0; security fixes are applied to the latest released
version (currently the `0.1.x` line) and `main`.

| Version | Supported |
|---------|-----------|
| latest `0.1.x` / `main` | ✅ |
| older | ❌ |

## Reporting a vulnerability

MeshExpander is an offline geometry library with no network surface, so the
realistic risk is **malformed input** (crafted STL / CAD meshes triggering
crashes, out-of-memory, or excessive compute in the clipping/partition stage).

If you find such an issue:

- Prefer GitHub's **private vulnerability reporting** ("Report a vulnerability"
  under the repository's *Security* tab), or
- Open a regular issue if the problem is not sensitive.

Please include a minimal reproducing input file and the observed behavior
(crash, hang, memory blow-up). We aim to acknowledge reports within a week.

Do not include any confidential CAD data in a public issue — strip the model
down to the smallest shape that reproduces the problem.
