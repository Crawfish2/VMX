package frc.robot.subsystems;

import java.util.function.DoubleBinaryOperator;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;



public class SimpleCamera extends SubsystemBase {
    private final CameraServer cameraServer = CameraServer.getInstance();
    private final UsbCamera camera;

    private final int width = 160;
    private final int height = 120;
    private final int fps = 5;

    // シミュレーション用
    private final CvSource testStream;
    private final Mat mat;
    private final Timer timer;

    public SimpleCamera() {
        if (Robot.isReal()) {
            camera = cameraServer.startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, fps);

            testStream = null;
            mat = null;
            timer = null;
        } else {
            camera = null;

            testStream = cameraServer.putVideo("Server Test", width, height);
            mat = new Mat(new int[] {height, width}, CvType.CV_8U);
            timer = new Timer();
            timer.start();
        }
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
        } else {
            if (timer.hasPeriodPassed(1.0 / fps)) {
                DoubleBinaryOperator randDouble = (min, max) -> (Math.random() * (max - min)) + min;
                Imgproc.rectangle(mat,
                        new Point(randDouble.applyAsDouble(0, width / 2),
                                randDouble.applyAsDouble(0, height / 2)),
                        new Point(randDouble.applyAsDouble(width / 2, width),
                                randDouble.applyAsDouble(height / 2, height)),
                        new Scalar(randDouble.applyAsDouble(0, 255),
                                randDouble.applyAsDouble(0, 255),
                                randDouble.applyAsDouble(0, 255)),
                        10);
                testStream.putFrame(mat);
            }
        }
    }
}

