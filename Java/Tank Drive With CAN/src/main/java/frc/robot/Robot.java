/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private Joystick leftStick;
  private Joystick rightStick;
  private PS4Controller controller;
  private static final int leftDeviceID = 1; 
  private static final int rightDeviceID = 2;
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a SparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
   *  com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    leftMotor = new SparkMax(leftDeviceID, MotorType.kBrushless);
    rightMotor = new SparkMax(rightDeviceID, MotorType.kBrushless);


    /*
     * Create config with no changes
     */
    SparkMaxConfig emptyConfig = new SparkMaxConfig();

    /*
     * Reset both the left motor and right motor to factory defaults that will not persist over power cycles.  
     */
    leftMotor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    myRobot = new DifferentialDrive(leftMotor, rightMotor);

    controller = new PS4Controller(0);

    // leftStick = new Joystick(0);
    // rightStick = new Joystick(1);

    /*
     * Display Default values to setup SmartDashboard
     */

     SmartDashboard.putNumber("Left Motor Applied Output", 0.0);
     SmartDashboard.putNumber("Right Motor Applied Output", 0.0);
  }

  @Override
  public void teleopPeriodic() {
    myRobot.tankDrive(controller.getLeftY(), controller.getRightY());

    /*
     * Refresh the applied output data on SmartDashboard
     */
    SmartDashboard.putNumber("Left Motor Applied Output", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Motor Applied Output", rightMotor.getAppliedOutput());
  }
}
