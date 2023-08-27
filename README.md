# Leeds-SoftRasizater

It is a old-school cpu rasizater, it was a coursework showcase need a lot of improvement in the future.

--------------------------------------

## Instructions
To compile on University Linux:
```bash
module load legacy-eng
module add gcc
module add qt/5.13.0
ln -s /usr/lib64/libGL.so.1 lib/libGL.so
qmake
make
```
To compile on OSX:
Use Homebrew to install qt
```bash
qmake -project QT+=opengl
qmake
make
```


To run on feng-linux / feng-gps:
```bash
./LeedsGLRenderWindow ../path_to/model.obj ../path_to/texture.ppm
```

To run on OSX:
```bash
./LeedsGLRenderWindow.app/Contents/MacOS/RaytraceRenderWindowRelease  ../path_to/model.obj ../path_to/texture.ppm
```

To run on Windows
```bash
./LeedsGLRenderWindow.exe ../path_to/model.obj ../path_to/texture.ppm
```
or
Click projects
Select `Run` on the left side menu under the active configuration
Add `../path_to/model.obj ../path_to/texture.ppm` to command line arguments
Click Run/Debug on the bottom left side.