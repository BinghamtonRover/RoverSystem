package video;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.image.ImageView;
import javafx.event.ActionEvent;

import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Mat;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public class FXController {

    @FXML
    private Button button;
    @FXML
    private ImageView currentFrame;

    private VideoCapture capture = new VideoCapture();
    private ScheduledExecutorService timer;

    /*
    Work in progress
    General Idea: gets a frame from the camera and we update that frame to the ImageView every 30 milisecond
     */
    public void startCamera(ActionEvent actionEvent) {
        Runnable frameGrabber = new Runnable() {
            @Override
            public void run() {

            }
        };
        this.timer = Executors.newSingleThreadScheduledExecutor();
        this.timer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);
    }

    protected Mat getFrame() {
        Mat frame = new Mat();
        if (this.capture.isOpened()){
            try {
                this.capture.read(frame);
                if(!frame.empty()){
                    Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BayerRG2GRAY);
                }
            } catch (Exception e){
                System.out.println(e);
            }
        }
        return frame;
    }
}