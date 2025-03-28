package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.OmniDrive;

public class ExampleSubsystem extends SubsystemBase {

  public OmniDrive omniDrive;

  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  public ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public ExampleSubsystem() {
    omniDrive = new OmniDrive();
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    for (int i = 0; i < OmniDrive.MOTOR_NUM; i++) {
      MotorsEncoderValue[i] = tab.add("M" + i, 0).getEntry();
      MotorsLimH_Value[i] = tab.add("M" + i + " nLim.H", 0).getEntry();
      MotorsLimL_Value[i] = tab.add("M" + i + " nLim.L", 0).getEntry();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < OmniDrive.MOTOR_NUM; i++) {
      MotorsEncoderValue[i].setDouble(omniDrive.getEncoderDistance(i));
      MotorsLimH_Value[i].setDouble(omniDrive.getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(omniDrive.getLimitSwitch(i, true));
    }
  }
}
