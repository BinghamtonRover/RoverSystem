This repository uses JInput, which requires a native binary. The latest version can be downloaded here:
    http://ci.newdawnsoftware.com/job/JInput/lastBuild/artifact/dist/libjinput-linux64.so

Place the `libjinput-linux64.so` library file in your current working directory. If Java doesn't find it (a runtime
exception will be thrown if that is the case), the library path can be changed with the Java command-line option
`-Djava.library.path=<path-to-directory-of-library-file>`.