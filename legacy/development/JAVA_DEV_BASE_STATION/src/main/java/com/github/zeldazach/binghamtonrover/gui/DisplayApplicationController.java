package com.github.zeldazach.binghamtonrover.gui;

import com.github.zeldazach.binghamtonrover.controller.ControllerHandler;
import com.github.zeldazach.binghamtonrover.controller.ControllerState;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;

import java.util.HashMap;
import java.util.Map;

public class DisplayApplicationController {
    @FXML
    private StackPane cameraView;

    @FXML
    private ImageView cameraImageView;

    @FXML
    private Canvas xboxView;

    private GraphicsContext xboxViewGraphicsContext;

    /**
     * Yes, this is hardcoded. It's used in the calculation of the offset of the joystick "pressed" image.
     * TODO: Does this need to be hardcoded in the future?
     */
    private static final double JOYSTICK_OFFSET_RATIO = 0.02125;


    public ImageView getCameraImageView() {
        return cameraImageView;
    }

    public void initialize() {
        // we can reuse our canvas context rather than continue to call upon this method each time
        xboxViewGraphicsContext = xboxView.getGraphicsContext2D();
        buildXboxView();
    }

    private void buildXboxView()
    {
        xboxViewGraphicsContext.setFill(Color.BLUE);
        xboxViewGraphicsContext.fillRect(0, 0, xboxView.getWidth(), xboxView.getHeight());

        ControllerHandler.getInstance().getControllerState().addObserver(
                (obs, o) -> Platform.runLater(this::renderXboxState));

        // Render this now to show the initial state.
        renderXboxState();
    }

    private void renderXboxState() {
        double joystickOffset = JOYSTICK_OFFSET_RATIO * xboxView.getWidth();
        ControllerState controllerState = ControllerHandler.getInstance().getControllerState();

        xboxViewGraphicsContext.setFill(Color.BLACK);
        xboxViewGraphicsContext.fillRect(0, 0, xboxView.getWidth(), xboxView.getHeight());

        drawImage(controllerState.buttonA ? "a_pressed" : "a_unpressed");
        drawImage(controllerState.buttonB ? "b_pressed" : "b_unpressed");
        drawImage(controllerState.buttonX ? "x_pressed" : "x_unpressed");
        drawImage(controllerState.buttonY ? "y_pressed" : "y_unpressed");
        drawImage(controllerState.buttonSelect ? "view_pressed" : "view_unpressed");
        drawImage(controllerState.buttonMode ? "xbox_pressed" : "xbox_unpressed");
        drawImage(controllerState.buttonStart ? "menu_pressed" : "menu_unpressed");
        drawImage(controllerState.buttonLBumper ? "lb_pressed" : "lb_unpressed");
        drawImage(controllerState.buttonRBumper ? "rb_pressed" : "rb_unpressed");

        drawImage("js_left_background");
        if (controllerState.lStickX != 0.0 || controllerState.lStickY != 0.0)
        {
            drawImage("js_left_pressed", Math.floor(controllerState.lStickX * joystickOffset),
                    Math.floor(controllerState.lStickY * joystickOffset));
        }
        else
        {
            drawImage("js_left_unpressed");
        }

        drawImage("js_right_background");
        if (controllerState.rStickX != 0.0 || controllerState.rStickY != 0.0)
        {
            drawImage("js_right_pressed", Math.floor(controllerState.rStickX * joystickOffset),
                    Math.floor(controllerState.rStickY * joystickOffset));
        }
        else
        {
            drawImage("js_right_unpressed");
        }

        drawImage("dpad_unpressed");
        if (controllerState.dpad == 0.25) drawImage("dpad_pressed_up");
        if (controllerState.dpad == 0.50) drawImage("dpad_pressed_right");
        if (controllerState.dpad == 0.75) drawImage("dpad_pressed_down");
        if (controllerState.dpad == 1.00) drawImage("dpad_pressed_left");

        drawImage("lt_unpressed");
        xboxViewGraphicsContext.setGlobalAlpha((controllerState.lTrigger + 1.0) / 2.0);
        drawImage("lt_pressed");
        xboxViewGraphicsContext.setGlobalAlpha(1.0);

        drawImage("rt_unpressed");
        xboxViewGraphicsContext.setGlobalAlpha((controllerState.rTrigger + 1.0) / 2.0);
        drawImage("rt_pressed");
        xboxViewGraphicsContext.setGlobalAlpha(1.0);
    }

    private Map<String, Image> controllerImageMap = new HashMap<>();

    private void drawImage(String imageName)
    {
        drawImage(imageName, 0, 0);
    }

    private void drawImage(String imageName, double x, double y)
    {
        Image image;

        if (!controllerImageMap.containsKey(imageName))
        {
            image = new Image("xbox/" + imageName + ".png");
            controllerImageMap.put(imageName, image);
        }
        else
        {
            image = controllerImageMap.get(imageName);
        }

        xboxViewGraphicsContext.drawImage(image, x, y, xboxView.getWidth(), xboxView.getHeight());
    }

}
