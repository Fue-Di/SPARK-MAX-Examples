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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick stick;
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  @Override
  public void robotInit() {
    // Starting PID coefficients
    kP = 0.00007; 
    kI = 0.000001;
    kD = 0.000001; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5500;

    stick = new Joystick(0);

    // Initialize motor
    motor = new SparkMax(deviceID, MotorType.kBrushless);

    // Initialize Spark Max configuration object with PID values
    motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    closedLoopController = motor.getClosedLoopController();

    // Encoder object created to display position values
    encoder = motor.getEncoder();

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void teleopPeriodic() {
    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // If PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { motorConfig.closedLoop.p(p); kP = p; }
    if((i != kI)) { motorConfig.closedLoop.i(i); kI = i; }
    if((d != kD)) { motorConfig.closedLoop.d(d); kD = d; }
    if((iz != kIz)) { motorConfig.closedLoop.iZone(iz); kIz = iz; }
    if((ff != kFF)) { motorConfig.closedLoop.velocityFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      motorConfig.closedLoop.outputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // Apply new changes if any
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * Closed Loop Controller objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.spark.SparkBase.ControlType.kDutyCycle
     *  com.revrobotics.spark.SparkBase.ControlType.kPosition
     *  com.revrobotics.spark.SparkBase.ControlType.kVelocity
     *  com.revrobotics.spark.SparkBase.ControlType.kVoltage
     */
    double setPoint = stick.getY()*maxRPM;
    closedLoopController.setReference(setPoint, SparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
  }
}
