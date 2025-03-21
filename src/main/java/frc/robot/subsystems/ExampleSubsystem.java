package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;

import frc.robot.Constants;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

public class ExampleSubsystem extends SubsystemBase {

  private TitanQuad[] motors;
  private TitanQuadEncoder[] motors_enc;
  private NetworkTableEntry[] motorsPos;
  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  private ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public ExampleSubsystem() {
    motors = new TitanQuad[4];
    motors_enc = new TitanQuadEncoder[4];
    motorsPos = new NetworkTableEntry[4];
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    

    for (int i = 0; i < 4; i++) {
      motors[i] = new TitanQuad(Constants.TITAN_ID, i);
      motors_enc[i] = new TitanQuadEncoder(motors[i], i, Constants.WHEEL_DIST_PER_TICK);

      motorsPos[i] = tab.add("Motor" + i, 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry();

      MotorsEncoderValue[i] = tab.add("M" + i, 0).getEntry();
      MotorsLimH_Value[i] = tab.add("M" + i + " nLim.H", 0).getEntry();
      MotorsLimL_Value[i] = tab.add("M" + i + " nLim.L", 0).getEntry();

      motorsPos[i].setDouble(0);
      motors_enc[i].reset();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      motors[i].set(motorsPos[i].getDouble(0.0));
      MotorsEncoderValue[i].setDouble(motors_enc[i].getEncoderDistance());
      MotorsLimH_Value[i].setDouble(motors[i].getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(motors[i].getLimitSwitch(i, true));
    }
  }
}
