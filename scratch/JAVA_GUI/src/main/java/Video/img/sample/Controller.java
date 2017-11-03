package Video.img.sample;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.io.File;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static Video.img.sample.Utils.mat2Image;

public class Controller {
    @FXML
    private Button startCameraBtn;
    @FXML
    private ImageView cameraView;

    private ScheduledExecutorService timer;
    //OpenCV object that allows video capture
    private VideoCapture capture = new VideoCapture();
    private boolean cameraActive = false;
    private static int cameraID = 0;
    private File coDefaultImgFile = new File("C:\\Users\\HaoSen\\IdeaProjects\\JavaFX\\src\\img\\weedimage.png");

    @FXML
    protected void startCamera(ActionEvent event)
    {
        //If the camera if inactive
        if(!this.cameraActive)
        {
            //open the camera
            this.capture.open(cameraID);

            //If the camera is now opened successfully
            if (this.capture.isOpened())
            {
                this.cameraActive = true;

                Runnable frameGrabber = () -> {

                    //OpenCV VideoCapture Grabs a frame
                    //Then convert it to FX Image
                    //Then update the image onto the imageView of the GUI
                    Mat frame = grabFrame();
                    Image imageToShow = mat2Image(frame);   //This method comes from the Util Class and
                    updateImageView(cameraView, imageToShow);
                };

                //Spawn OR run the thread frameGrabber every 33 ms, 30 times a second
                this.timer = Executors.newSingleThreadScheduledExecutor();
                this.timer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);

                this.startCameraBtn.setText("Stop Camera");
            }
            else
            {
                //If we cannot open the camera, then log the error
                System.out.println("ERROR, Cannot open the camera connection");
            }

        }
        else
        {
            //if the camera is inactive, set status and text
            this.cameraActive = false;
            this.startCameraBtn.setText("Start Camera");

            //Stop the timer for the frameGrabber
            this.stopAcquisition();
        }
    }

    private Mat grabFrame()
    {
        Mat frame = new Mat();

        if(this.capture.isOpened())
        {
            try
            {
                //If our VideoCapture is opened, read the current frame of the camera
                this.capture.read(frame);
                //If what we read is not empty, then convert the color to Gray Scale
                if(!frame.empty())
                {
                    Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2GRAY);

                }
            }
            catch(Exception e)
            {
                System.out.println("Exception thrown during the image elaboration: " + e);
            }
        }
        return frame;
    }

    private void stopAcquisition()
    {
        //If the timer is not gone and still running
        if(this.timer!=null && !this.timer.isShutdown())
        {
            try
            {
                //Shutdown the timer
                this.timer.shutdown();
                this.timer.awaitTermination(33, TimeUnit.MILLISECONDS);

                //If the frame capture is stopped, then set the image to a default image
                if(coDefaultImgFile.exists())
                {
                    Image coDefaultImage = new Image(coDefaultImgFile.toURI().toString());
                    updateImageView(cameraView, coDefaultImage);
                }
                else
                {
                    System.out.println("Default image does not exist");
                }
            }
            catch (InterruptedException e)
            {
                System.out.println("Exception thrown while trying to close the timer for frame capture");
            }
        }

        if(this.capture.isOpened())
        {
            //Release the camera
            this.capture.release();
        }
    }

    private void updateImageView(ImageView view, Image image)
    {

        view.setImage(image);
        //onFXThread(view.imageProperty(), image);
    }

    //When the application is closed stop grabbing frames from the camera
    protected void setClosed()
    {
        this.stopAcquisition();
    }
}
