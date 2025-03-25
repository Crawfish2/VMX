package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;
import java.util.function.Supplier;

import frc.robot.commands.LambdaCommand;
import frc.robot.commands.auto.DriveMotor;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.task.DriveCorners;
import frc.robot.commands.test1.DriveTri;
import frc.robot.util.OmniDrive;

public class ExampleSubsystem extends SubsystemBase {

  public OmniDrive omniDrive;

  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  public SendableChooser<Supplier<Command>> chooser;

  private NetworkTableEntry globalSpeed;
  private NetworkTableEntry globalAngle;

  public ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public double angle = 0.0;

  public ExampleSubsystem() {
    omniDrive = new OmniDrive();
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    chooser = new SendableChooser<>();
    // 角度はchooser.getSelected().get()が呼ばれた時点の値を使う
    chooser.setDefaultOption("DriveMotor", () -> new DriveMotor(this.angle).andThen(new Rotate(0).withTimeout(1)));
    chooser.addOption("DriveMotor", () -> new DriveMotor(this.angle).andThen(new Rotate(0).withTimeout(1)));
    chooser.addOption("Rotate", () -> new Rotate(this.angle).withTimeout(5).andThen(new Rotate(0).withTimeout(1)));
    chooser.addOption("DriveCorners", () -> new DriveCorners());
    chooser.addOption("DriveTri", () -> new DriveTri());
    {
      Ultrasonic[] sonar = new Ultrasonic[1];
      NetworkTableEntry sonarValue = tab.add("LambdaTest", 0.0).getEntry();

      chooser.addOption("LambdaTest", () -> new LambdaCommand(() -> {
        sonar[0] = new Ultrasonic(8, 9);
      }, () -> sonarValue.setDouble(sonar[0].getRangeMM()), () -> sonar[0].close(), this));
    }
    tab.add(chooser);

    globalSpeed = tab.add("Global Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    globalAngle = tab.add("Global Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -360.0, "max", 360.0))
        .getEntry();

    for (int i = 0; i < OmniDrive.MOTOR_NUM; i++) {
      MotorsEncoderValue[i] = tab.add("M" + i, 0).getEntry();
      MotorsLimH_Value[i] = tab.add("M" + i + " nLim.H", 0).getEntry();
      MotorsLimL_Value[i] = tab.add("M" + i + " nLim.L", 0).getEntry();
    }
  }

  @Override
  public void periodic() {
    double speed = globalSpeed.getDouble(0.0);
    double angle = globalAngle.getDouble(0.0);

    this.angle = angle;

    omniDrive.move(speed, angle);

    for (int i = 0; i < OmniDrive.MOTOR_NUM; i++) {
      MotorsEncoderValue[i].setDouble(omniDrive.getEncoderDistance(i));
      MotorsLimH_Value[i].setDouble(omniDrive.getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(omniDrive.getLimitSwitch(i, true));
    }
  }
}
