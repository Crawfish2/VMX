package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;

import frc.robot.util.OmniDrive;

public class ExampleSubsystem extends SubsystemBase {

  public OmniDrive omniDrive;

  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  private NetworkTableEntry globalSpeed;
  private NetworkTableEntry globalAngle;

  public ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public double angle = 0.0;

  public ExampleSubsystem() {
    omniDrive = new OmniDrive();
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    globalSpeed = tab.add("Global Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    globalAngle = tab.add("Global Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -360.0, "max", 360.0))
        .getEntry();

    for (int i = 0; i < 4; i++) {
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

    for (int i = 0; i < 4; i++) {
      MotorsEncoderValue[i].setDouble(omniDrive.getEncoderDistance(i));
      MotorsLimH_Value[i].setDouble(omniDrive.getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(omniDrive.getLimitSwitch(i, true));
    }
  }
}
