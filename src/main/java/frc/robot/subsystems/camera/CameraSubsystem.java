// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import java.util.EnumSet;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private UsbCamera camera1;
  private UsbCamera camera2;
  private MjpegServer mjpegServer1;
  private double currentCameraID;
  private NetworkTableInstance cameraNetworkTableInstance;
  private NetworkTable cameraNetworkTable;
  public CameraSubsystem() {
    camera1 = new UsbCamera("camera1", 0);
    camera1.setFPS(60);
    camera1.setResolution(Constants.CameraConstants.resolutionX, Constants.CameraConstants.resolutionY);
    camera1.setBrightness(45);

    camera2 = new UsbCamera("camera2", 1);
    camera2.setFPS(60);
    camera2.setResolution(Constants.CameraConstants.resolutionX, Constants.CameraConstants.resolutionY);
    camera2.setBrightness(45);

    mjpegServer1 = new MjpegServer("m1", 1181);
    mjpegServer1.setSource(camera1);
    currentCameraID = 1.;
    cameraNetworkTableInstance = NetworkTableInstance.getDefault();
        cameraNetworkTable = cameraNetworkTableInstance.getTable("camera");
        cameraNetworkTable.addListener("id",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (NetworkTable table, String key, NetworkTableEvent event) -> {
            double message = event.valueData.value.getDouble();
            System.out.println(message);
            if(message == currentCameraID){
              // System.out.print("Still the same"); 
            }
            else if(message == 1.0){ 
              // System.out.println("Set to cam1");
              switchTo(1);
            }
            else{
              // System.out.println("Set to cam2");
              switchTo(2);
            }
        });
  }
  public void switchTo(int cameraID){
    if(cameraID == 1){
      currentCameraID = 1;
      mjpegServer1.setSource(camera1);
      currentCameraID = 1;
    }
    else if(cameraID == 2){
      mjpegServer1.setSource(camera2);
      currentCameraID = 2;
    }
    return;
  }
}
