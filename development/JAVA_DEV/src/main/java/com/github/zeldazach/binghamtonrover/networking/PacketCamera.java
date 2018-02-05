package com.github.zeldazach.binghamtonrover.networking;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.nio.ByteBuffer;

public class PacketCamera extends Packet {
    private int frameSize;
    private Image image;

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        // what should we do? currently noop. Exception?
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        try {
            buff.getChar();

            byte[] buffarr = new byte[buff.limit() - buff.position()];

            buff.get(buffarr);

            ByteArrayInputStream imageStream = new ByteArrayInputStream(buffarr);


            image = new Image(imageStream);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Image toJavaFXImage() throws IllegalStateException {
        return image;
    }
}
