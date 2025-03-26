package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;
import java.util.function.Supplier;

import frc.robot.commands.auto.DriveMotor;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.driveCommands.Stop;
import frc.robot.commands.sensors.SonicSensor;
import frc.robot.commands.task.DriveCorners;
import frc.robot.commands.test.DriveTri;
import frc.robot.util.OmniDrive;

public class ExampleSubsystem extends SubsystemBase {

  public OmniDrive omniDrive;

  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  public SendableChooser<Supplier<Command>> chooser;

  private NetworkTableEntry globalSpeedValue;
  private NetworkTableEntry globalAngleValue;

  private NetworkTableEntry distanceValue;

  public ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public double speed = 0.0;
  public double angle = 0.0;

  public ExampleSubsystem() {
    omniDrive = new OmniDrive();
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    distanceValue = tab.add("Distance", 0.0).getEntry();

    chooser = new SendableChooser<>();
    // 速度や角度はchooser.getSelected().get()が呼ばれた時点の値を使う
    chooser.setDefaultOption("DriveMotor", () -> new DriveMotor(this.angle).andThen(new Stop(1)));
    chooser.addOption("Rotate", () -> new Rotate(this.speed).withTimeout(5).andThen(new Stop(1)));
    chooser.addOption("DriveCorners", () -> new DriveCorners());
    chooser.addOption("DriveTri", () -> new DriveTri());
    chooser.addOption("SonicSensor",
        () -> new ParallelRaceGroup(new SonicSensor(this.angle), new Drive(this.angle)));
    tab.add(chooser);

    globalSpeedValue = tab.add("Global Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    globalAngleValue = tab.add("Global Angle", 0.0)
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
    double speed = globalSpeedValue.getDouble(0.0);
    double angle = globalAngleValue.getDouble(0.0);

    this.speed = speed;
    this.angle = angle;

    distanceValue.setDouble(omniDrive.getDistance(angle));

    for (int i = 0; i < OmniDrive.MOTOR_NUM; i++) {
      MotorsEncoderValue[i].setDouble(omniDrive.getEncoderDistance(i));
      MotorsLimH_Value[i].setDouble(omniDrive.getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(omniDrive.getLimitSwitch(i, true));
    }
  }
}
