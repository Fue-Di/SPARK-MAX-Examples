/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Robot extends TimedRobot {
  private Joystick joystick;
  private static final int deviceID = 1;
  private SparkMax motor;
  private RelativeEncoder encoder;

  @Override
  public void robotInit() {
    // Initialize SPARK MAX with a brushless and a CAN ID of 1
    motor = new SparkMax(deviceID, MotorType.kBrushless);

    // Create empty config to be able to restore defaults on the SPARK MAX
    SparkMaxConfig emptyConfig = new SparkMaxConfig();
    /**
     * Call the configure function to:
     *    - Apply any settings you set
     *    - Restore the SPARK MAX factory default values
     *    - Save parameters across power cycles  
     */
    motor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing SparkMax object
    */
    encoder = motor.getEncoder();

    /**
     * Setup Smartdashboard to show default values on init.
     */
    SmartDashboard.putNumber("Encoder Position", 0);
    SmartDashboard.putNumber("Encoder Velocity", 0);

    joystick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    // Set the motor output based on jostick position
    motor.set(joystick.getY());

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", encoder.getVelocity());
  }
}
