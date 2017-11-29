package BinghamtonRover.GuiMain;

import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static BinghamtonRover.Video.Utils.mat2Image;

public class GuiController
{
    private static int gnCameraID = 0;

    @FXML private Button coStartCameraBtn;
    @FXML private ImageView coCameraView;

    @FXML private Label coTimeLabel;
    @FXML private Label coStatusLabel;
    @FXML private Label coLatitudeLabel;
    @FXML private Label coPressureLabel;
    @FXML private Label coTemperatureLabel;

    private ScheduledExecutorService coTimer;

    private VideoCapture coVideoCapture = new VideoCapture();
    private boolean cbCameraActive = false;

    @FXML
    protected void startCamera(ActionEvent aoActionEvent)
    {
        if(! cbCameraActive)
        {
            //open the camera
            coVideoCapture.open(gnCameraID);

            //If the camera is now opened successfully
            if (coVideoCapture.isOpened())
            {
                cbCameraActive = true;

                Runnable loRunnableFrameGrabber = () -> {
                    // OpenCV VideoCapture Grabs a frame
                    // Then convert it to FX Image
                    // Then update the image onto the imageView of the GUI
                    Mat loMatFrame = grabFrame();

                    // This method comes from the Util Class and
                    Image lOImageToShow = mat2Image(loMatFrame);
                    updateImageView(coCameraView, lOImageToShow);
                };

                //Spawn OR run the thread frameGrabber every 33 ms, 30 times a second
                coTimer = Executors.newSingleThreadScheduledExecutor();
                coTimer.scheduleAtFixedRate(loRunnableFrameGrabber, 0, 33, TimeUnit.MILLISECONDS);

                coStartCameraBtn.setText("Stop Camera");
            }
            else
            {
                //If we cannot open the camera, then log the error
                System.out.println("ERROR, Cannot open the camera connection");
                System.exit(1);
            }
        }
        else
        {
            //if the camera is inactive, set status and text
            cbCameraActive = false;
            coStartCameraBtn.setText("Start Camera");

            //Stop the coTimer for the frameGrabber
            stopAcquisition();
        }
    }

    private Mat grabFrame()
    {
        Mat loMatFrame = new Mat();

        if (coVideoCapture.isOpened())
        {
            try
            {
                //If our VideoCapture is opened, read the current frame of the camera
                coVideoCapture.read(loMatFrame);

                //If what we read is not empty, then convert the color to Gray Scale
                if(! loMatFrame.empty())
                {
                    Imgproc.cvtColor(loMatFrame, loMatFrame, Imgproc.COLOR_BGR2GRAY);
                }
            }
            catch (NullPointerException aoException)
            {
                System.out.println("Exception thrown during the image elaboration: " + aoException);
                System.exit(1);
            }
        }
        return loMatFrame;
    }

    private void stopAcquisition()
    {

        //If the coTimer is not gone and still running
        if (coTimer != null && ! coTimer.isShutdown())
        {
            try
            {
                //Shutdown the coTimer
                coTimer.shutdown();
                coTimer.awaitTermination(33, TimeUnit.MILLISECONDS);

                //If the frame coVideoCapture is stopped, then set the image to a default image

                String s = GuiController.class.getResource("./weedimage.png").toString();
                Image loDefaultImage = new Image(s);
                updateImageView(coCameraView, loDefaultImage);

            }
            catch (InterruptedException e)
            {
                System.out.println("Exception thrown while trying to close the coTimer for frame coVideoCapture");
            }
        }

        if (coVideoCapture.isOpened())
        {
            //Release the camera
            //coVideoCapture.release();
            //System.out.println("RELEASE!");
        }
    }

    private void updateImageView(ImageView aoImageView, Image aoImage)
    {
        aoImageView.setImage(aoImage);
    }

    public void updatePressure(String lsText)
    {
        System.out.println(lsText);

        Platform.runLater(new Runnable(){
            @Override
            public void run() {
                coPressureLabel.setText(lsText);
            }
        });
    }

    public void updateTime(String lsText)
    {
        System.out.println(lsText);

        Platform.runLater(new Runnable(){
            @Override
            public void run() {
                coTimeLabel.setText(lsText);
            }
        });
    }

    public void updateStatus(String lsText)
    {
        System.out.println(lsText);

        Platform.runLater(new Runnable(){
            @Override
            public void run() {
                coStatusLabel.setText(lsText);
            }
        });
    }

    public void updateLatitude(String lsText)
    {
        System.out.println(lsText);

        Platform.runLater(new Runnable(){
            @Override
            public void run() {
                coLatitudeLabel.setText(lsText);
            }
        });
    }

    public void updateTemperature(String lsText)
    {
        System.out.println(lsText);

        Platform.runLater(new Runnable(){
            @Override
            public void run() {
                coTemperatureLabel.setText(lsText);
            }
        });
    }

    //When the application is closed stop grabbing frames from the camera
    protected void setClosed()
    {
        stopAcquisition();
        // DO NOT CALL, CAUSES KNOWN ISSUE WITH OPENCV3.2
    }
}