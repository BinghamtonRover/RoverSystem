package com.github.zeldazach.binghamtonrover.networking;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

public class PacketCamera extends Packet {
    private char frameSize;
    private BufferedImage bufferedImage;

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        // what should we do? currently noop. Exception?
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        frameSize = buff.getChar();
        byte[] imageData = new byte[frameSize];
        System.arraycopy(buff.array(), 6, imageData, 0, frameSize);
        ByteArrayInputStream imageStream = new ByteArrayInputStream(imageData);
        try {
            bufferedImage = ImageIO.read(imageStream);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Image toJavaFXImage() throws IllegalStateException {
        if (bufferedImage == null) throw new IllegalStateException("BufferedImage not available");
        return SwingFXUtils.toFXImage(bufferedImage, null);
    }
}
