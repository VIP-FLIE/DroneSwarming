# Add a ROS 2 package

Create the package under `src/` and declare its ROS dependencies in `package.xml` and `CMakeLists.txt` or `setup.py`.

Build it in the project image:

```bash
docker compose up --build -d
docker compose exec simulation bash
```

The Dockerfile runs `colcon build --symlink-install` for packages under `src/`. Rebuild the image after adding a package or changing build metadata.
