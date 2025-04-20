package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;


/** 外部プロセスに処理を丸投げしたカメラ */
public class SimpleCamera extends SubsystemExBase {
  private final CameraServer cameraServer = CameraServer.getInstance();
  private final UsbCamera camera;

  private final int width = 160;
  private final int height = 120;
  private final int fps = 5;

  /** カメラ映像を処理してほしいかどうか */
  private boolean enableCameraProcessing;
  /** Boolean Publisher */
  private final NetworkTableEntry enableCameraProcessingEntry;

  /** String Subscriber */
  private final NetworkTableEntry detectedColor;
  /** Double Subscriber */
  private final NetworkTableEntry detectedAreaX;
  /** Double Subscriber */
  private final NetworkTableEntry detectedAreaY;

  public SimpleCamera() {
    camera = cameraServer.startAutomaticCapture();
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, fps);

    final var nt = NetworkTableInstance.getDefault();

    enableCameraProcessing = false;
    enableCameraProcessingEntry = nt.getEntry("/ColorSquares/processCamera");

    detectedColor = nt.getEntry("/ColorSquares/DetectedColor");
    detectedAreaX = nt.getEntry("/ColorSquares/DetectedAreaX");
    detectedAreaY = nt.getEntry("/ColorSquares/DetectedAreaY");
  }

  /** カメラの外部処理を開始する */
  public void enableProcess() {
    // 前に検出した色をクリアする
    detectedColor.setString("NONE");

    enableCameraProcessing = true;
    enableCameraProcessingEntry.setBoolean(enableCameraProcessing);

  }

  /** カメラの外部処理を停止する */
  public void disableProcess() {
    enableCameraProcessing = false;
    enableCameraProcessingEntry.setBoolean(enableCameraProcessing);
  }

  /** カメラ映像の外部処理をしているか */
  public boolean isEnabled() {
    return enableCameraProcessing;
  }

  /** 検出した色を取得する */
  public ColorType getDetectedColor() {
    return ColorType.valueOf(detectedColor.getString("NONE"));
  }

  /** 検出した色の中心座標のX軸(左右)を返す */
  public double getDetectedCentorAreaX() {
    return detectedAreaX.getDouble(0.0);
  }

  /** 検出した色の中心座標のY軸(上下)を返す */
  public double getDetectedCentorAreaY() {
    return detectedAreaY.getDouble(0.0);
  }

  /** カメラ処理を開始して、色を検出するまで待つコマンド */
  public Command DetectColorCommand() {
    return functional(this::enableProcess, null, (interrupted) -> this.disableProcess(),
        () -> !getDetectedColor().equals(ColorType.NONE));
  }

  /** 検出した色 */
  public static enum ColorType {
    NONE, RED, BLUE, YELLOW;
  };
}
