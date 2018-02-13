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
    private Image image;

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        throw new IllegalStateException("BASE STATION MUST NOT SEND CAMERA PACKETS!");
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        // Skip the length.
        buff.getChar();

        byte[] buffarr = new byte[buff.limit() - buff.position()];
        buff.get(buffarr);
        ByteArrayInputStream imageStream = new ByteArrayInputStream(buffarr);

        image = new Image(imageStream);
    }

    public Image toJavaFXImage() throws IllegalStateException {
        return image;
    }
}
