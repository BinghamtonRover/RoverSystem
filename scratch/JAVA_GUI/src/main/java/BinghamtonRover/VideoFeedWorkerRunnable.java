package BinghamtonRover;


import org.bytedeco.javacv.OpenCVFrameGrabber;

public class VideoFeedWorkerRunnable implements Runnable {

    private OpenCVFrameGrabber feed;

    VideoFeedWorkerRunnable(OpenCVFrameGrabber feed){
        this.feed = feed;
    }

    @Override
    public void run() {
        WebCamServer.startVideoFeed(feed);
    }
}
