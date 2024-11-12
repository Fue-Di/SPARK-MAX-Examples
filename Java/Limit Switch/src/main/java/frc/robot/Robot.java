/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Joystick joystick;
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkLimitSwitch forwardLimit;
  private SparkLimitSwitch reverseLimit;

  private boolean isForwardLimitEnabled;
  private boolean isReverseLimitEnabled;
  private Type forwardLimitNormalState;
  private Type reverseLimitNormalState;

  @Override
  public void robotInit() {
    // Setup starting values for limit switches
    isForwardLimitEnabled = false;
    isReverseLimitEnabled = false;
    forwardLimitNormalState = Type.kNormallyOpen;
    reverseLimitNormalState = Type.kNormallyOpen;

    // Initialize SPARK MAX with CAN ID
    motor = new SparkMax(deviceID, MotorType.kBrushless);

    // Initialize SPARK MAX Configuration object to hold the changes we want applied
    motorConfig = new SparkMaxConfig();

    /**
     * A SparkLimitSwitch object is constructed using the getForwardLimitSwitch() or
     * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
     * on which direction you would like to limit
     */
    forwardLimit = motor.getForwardLimitSwitch();
    reverseLimit = motor.getReverseLimitSwitch();

    /**
     * Limit switches are enabled by default when they are initialized. Using the SPARK MAX 
     * configuration object the enable state of the forward and reverse limits can stored.
     * 
     * Limit switches can be also configured to one of two polarities:
     *  com.revrobotics.spark.config.LimitSwitchConfig.TypekNormallyOpen
     *  com.revrobotics.spark.config.LimitSwitchConfig.TypekNormallyClosed
     * 
     * Changes can then be applied using the configure() method of an existing SPARK MAX
     * object.  
     */
    motorConfig.limitSwitch
      .forwardLimitSwitchEnabled(isForwardLimitEnabled)
      .forwardLimitSwitchType(forwardLimitNormalState)
      .reverseLimitSwitchEnabled(isReverseLimitEnabled)
      .reverseLimitSwitchType(reverseLimitNormalState);

    // Restore defaults and apply changes stored in the configuration object
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    joystick = new Joystick(0);

    // Display the starting values on Smart Dashboard
    SmartDashboard.putBoolean("Forward Limit Enabled", isForwardLimitEnabled);
    SmartDashboard.putBoolean("Reverse Limit Enabled", isReverseLimitEnabled);

    /**
     * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the switch.
     * 
     * In this example, the polarity of the switches are set to normally open. In this case,
     * isPressed() will return true if the switch is pressed. It will also return true if you do not 
     * have a switch connected. isPressed() will return false when the switch is released.
     */
    SmartDashboard.putBoolean("Forward Limit Activated", forwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Activated", reverseLimit.isPressed());
  }

  @Override
  public void teleopPeriodic() {
    motor.set(joystick.getY());
    // Get the current values of the limit switches from Smart Dashboard
    boolean fwLimEnabled = SmartDashboard.getBoolean("Forward Limit Enabled", false);
    boolean revLimEnabled = SmartDashboard.getBoolean("Reverse Limit Enabled", false); 

    // Update SPARK MAX configuration if there are any changes 
    if(isForwardLimitEnabled != fwLimEnabled) {
      isForwardLimitEnabled = fwLimEnabled;
      motorConfig.limitSwitch.forwardLimitSwitchEnabled(isForwardLimitEnabled);
      SmartDashboard.putBoolean("Forward Limit Enabled", isForwardLimitEnabled);
    } 

    if(isReverseLimitEnabled != revLimEnabled) {
      isReverseLimitEnabled = revLimEnabled;
      motorConfig.limitSwitch.reverseLimitSwitchEnabled(isReverseLimitEnabled);
      SmartDashboard.putBoolean("Reverse Limit Enabled", isReverseLimitEnabled);
    }

    // Apply changes (if any) to the SPARK MAX
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putBoolean("Forward Limit Activated", forwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Activated", reverseLimit.isPressed());
  }
}
