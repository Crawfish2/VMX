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
  
  private TitanQuad motor0;
  private TitanQuadEncoder motor0_enc;
  
  private ShuffleboardTab tab = Shuffleboard.getTab("VMX");
  
  private NetworkTableEntry motorPos0 = tab.add("Motor0", 0)
                                          .withWidget(BuiltInWidgets.kNumberSlider)
                                          .withProperties(Map.of("min", -1, "max", 1))
                                          .getEntry(); 

  private NetworkTableEntry M0EncoderValue = tab.add("M0", 0) .getEntry();                                        
  private NetworkTableEntry M0LimH_Value = tab.add("M0 nLim.H", 0) .getEntry();              
  private NetworkTableEntry M0LimL_Value = tab.add("M0 nLim.L", 0) .getEntry();              
  public ExampleSubsystem()
    {
    motor0 = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR0);
    motor0_enc = new TitanQuadEncoder(motor0, Constants.MOTOR0, Constants.WHEEL_DIST_PER_TICK);
    motorPos0.setDouble(0);
   
    motor0_enc.reset();
    }
  
  @Override
  public void periodic()
    {
     motor0.set(motorPos0.getDouble(0.0));
     M0EncoderValue.setDouble( motor0_enc.getEncoderDistance());
     M0LimH_Value.setDouble( motor0.getLimitSwitch(0,false)); //direction nLim.H = false
     M0LimL_Value.setDouble( motor0.getLimitSwitch(0,true)); //direction nLim.L = true
    }
}
