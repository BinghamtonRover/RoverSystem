This repository uses JInput, which requires a native binary. The latest version can be downloaded here:
    http://ci.newdawnsoftware.com/job/JInput/lastBuild/artifact/dist/libjinput-linux64.so

Place the `libjinput-linux64.so` library file in your current working directory. If Java doesn't find it (a runtime
exception will be thrown if that is the case), the library path can be changed with the Java command-line option
`-Djava.library.path=<path-to-directory-of-library-file>`.

For whatever reason, when running this on Zach's laptop, jinput was looking for libjinput-linux.so (the 32-bit library),
even though java was 64 bit and linux was 64 bit. If that occurs, place 'libjinput-linux.so' in the same directory
as above. It can be downloaded here:
    http://ci.newdawnsoftware.com/job/JInput/lastBuild/artifact/dist/libjinput-linux.so