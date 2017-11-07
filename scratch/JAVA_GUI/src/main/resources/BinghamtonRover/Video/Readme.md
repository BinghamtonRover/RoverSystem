# Branch Note
I have the camera feed working locally on one computer with openCV and javaFX.

get the openCV 3.3.1 library from: https://opencv.org/releases.html

import the openCV library to Intellij through the following steps(For Window):

- Go to project structure->Modules->Dependencies tab
- Click on the green plus sign on the right of the panel
- Choose import JARs or directories
- Navigate to the directory where you save your openCV Library
- navigate to the subdirectory: build->java->opencv-331.JAR and add it
- after you added the JAR, double click it
- in the Configure Module Library window, add the x64 folder in the same directory as the JAR

 Make sure the resource file have the same hierarchy as the main package<br>
   - ie. if your main.java is in Video.img.sample folder then the fxml resource 
in the resources folder should also be under the Video.img.sample folder

## You can find the JAR and dylib in src/main/files/opencv/


