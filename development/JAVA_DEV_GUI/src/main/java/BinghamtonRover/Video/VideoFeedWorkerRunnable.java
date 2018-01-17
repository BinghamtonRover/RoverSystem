package BinghamtonRover.Video;


import org.bytedeco.javacv.OpenCVFrameGrabber;

public class VideoFeedWorkerRunnable implements Runnable {

    private OpenCVFrameGrabber feed;

    VideoFeedWorkerRunnable(OpenCVFrameGrabber feed){
        this.feed = feed;
    }

    /**
     * This thread calls the startVideoFeed() method from WebCamServer.
     * We will probably want to take the method body and just place it in here,
     * and eliminate the method from the WebCamServer.
     */
    @Override
    public void run() {
        WebCamServer.startVideoFeed(feed);
    }
}
