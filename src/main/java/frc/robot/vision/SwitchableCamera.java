package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A `VideoSource` that toggles between a front and back camera.
 */
public class SwitchableCamera {
    private final UsbCamera top;
    private final UsbCamera bottom;

    private static final int TOP_EXPOSURE = 39;
    private static final int TOP_BRIGHTNESS = 5;

    private static final int BOTTOM_EXPOSURE = 39;
    private static final int BOTTOM_BRIGHTNESS = 5;

    private final VideoSink server;
    private final ComplexWidget widget;

    private boolean topActive = true;

    public SwitchableCamera(ShuffleboardTab shuffleboardTab) {
        top = CameraServer.startAutomaticCapture(1);
        top.setResolution(140, 120);
        top.setExposureManual(TOP_EXPOSURE);
        top.setBrightness(TOP_BRIGHTNESS);
        top.setFPS(30);

        bottom = CameraServer.startAutomaticCapture(0);
        bottom.setResolution(140, 120);
        bottom.setExposureManual(BOTTOM_EXPOSURE);
        bottom.setBrightness(BOTTOM_BRIGHTNESS);
        bottom.setFPS(30);

        // https://github.com/wpilibsuite/allwpilib/blob/main/cscore/src/main/native/linux/UsbCameraImpl.cpp#L108-L124
        // https://www.chiefdelphi.com/t/usb-camera-exposure-too-high-with-setexposuremanual/353630/8
        // top.getProperty("raw_exposure_absolute").set(156);
        // back.getProperty("raw_exposure_absolute").set(156);

        // top.getProperty("raw_brightness").set(40);
        // back.getProperty("raw_brightness").set(40);

        server = CameraServer.getServer();
        server.setSource(bottom);

        widget = shuffleboardTab.add("Intake Camera", getSource())
            .withPosition(9, 0)
            .withSize(3, 3);
    }

    /**
     * Switches the source connected to this camera (toggles between
     * top and bottom).
     */
    public void switchCamera() {
        setCamera(!this.topActive);
    }

    /**
     * Sets the source connected to this camera.
     * @param useTop Whether to set it to the top camera. If false, sets it to the bottom instead.
     */
    public void setCamera(boolean useTop) {
        this.topActive = useTop;
        // server.setSource(this.topActive ? top : bottom);
        // widget.withProperties(Map.of("Rotation", this.topActive ? "NONE" : "QUARTER_CW"));
    }

    /**
     * Gets the `VideoSource` of the camera.
     * @return The connected `VideoSource`.
     */
    public VideoSource getSource() {
        return server.getSource();
    }
}
