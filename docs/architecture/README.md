# Architecture Planning and Diagrams

Dependencies: `graphviz`
```bash
sudo apt install graphviz
```

General compile command:
```bash
dot -Tpng -Gdpi=96 raw/<name>.dot -o renders/<name>.png
```

Example — waypoint architecture diagram:
```bash
dot -Tpng -Gdpi=96 raw/arch-waypoint.dot -o renders/arch-waypoint.png
```