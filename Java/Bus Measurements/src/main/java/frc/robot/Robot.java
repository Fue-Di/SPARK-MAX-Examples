/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This sample program shows how to read basic bus measurements from the SparkMax
 */
public class Robot extends TimedRobot {
  /**
   * Parameters for the SparkMax are defined below. Be sure to change the deviceID
   * and motor type to match your setup.
   */
  private static final int deviceID = 1;
  private static final MotorType motorType = MotorType.kBrushless;
  private SparkMax motor;

  /**
   * A single joystick will be used to control motor outputs when the robot is 
   * enabled.
   */
  private static final int kJoystickPort = 0;
  private Joystick joystick;

  @Override
  public void robotInit() {
    motor = new SparkMax(deviceID, motorType);

    joystick = new Joystick(kJoystickPort);
  }

  @Override
  public void robotPeriodic() {
    /**
     * There are several useful bus measurements you can get from the SparkMax.
     * This includes bus voltage (V), output current (A), Applied Output 
     * (duty cycle), and motor temperature (C)
     */
    double busVoltage = motor.getBusVoltage();
    double current = motor.getOutputCurrent();
    double appliedOut = motor.getAppliedOutput();
    double temperature = motor.getMotorTemperature();

    // Open SmartDashboard when your program is running to see the values
    SmartDashboard.putNumber("Bus Voltage", busVoltage);
    SmartDashboard.putNumber("Current", current);
    SmartDashboard.putNumber("Applied Output", appliedOut);
    SmartDashboard.putNumber("Motor Temperature", temperature);
  }

  @Override
  public void teleopPeriodic() {
    motor.set(joystick.getY());
  }
}
