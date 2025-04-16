package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SimpleCamera extends SubsystemBase {
  private final CameraServer cameraServer = CameraServer.getInstance();
  private final UsbCamera camera;

  private final int width = 160;
  private final int height = 120;
  private final int fps = 5;

  public SimpleCamera() {
    camera = cameraServer.startAutomaticCapture();
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, fps);
  }

  @Override
  public void periodic() {}
}
