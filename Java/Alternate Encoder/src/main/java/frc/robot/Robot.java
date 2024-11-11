/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  /**
   * Change these parameters to match your setup
   */
  private static final int kCanID = 1;
  private static final MotorType kMotorType = MotorType.kBrushless;
  private static final int kCPR = 8192;

  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing SparkMax object. If using a REV Through Bore 
   * Encoder, the counts per revolution should be set to 8192.
   */
  private RelativeEncoder alternateEncoder;

  @Override
  public void robotInit() {
    /*
     * Define starting PID coefficients
     */
    kP = 0.009;
    kI = 0.0000002;
    kD = 0.00001;
    kIz = 0;
    kFF = 0.00001;
    kMaxOutput = 0.2;
    kMinOutput = -0.2;

    /*  
     * Initialize SPARK MAX with CAN ID
     */
    motor = new SparkMax(kCanID, kMotorType);

    /*
     * Initialize the motor config to set settings we want
     */
    motorConfig = new SparkMaxConfig();


    /*
     *  Adjust the alternate encoder config with the encoder's counts per revolution
     */
    motorConfig.alternateEncoder.countsPerRevolution(kCPR);

    /**
     * By default, the closed loop controller will use the primary encoder sensor for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder. We also setup the PID values we want to use in the closed
     */
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    /*
     * Reset to defaults and apply the changes to the motor 
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Get Alternate Encoder object with getAlternateEncoder() to be able to read the position and velocity values
     */
    alternateEncoder = motor.getAlternateEncoder();
    
    /**
     * In order to use closed loop functionality for a controller, a SparkClosedLoopController object
     * is constructed by calling the getClosedLoopController() method on an existing
     * SparkMax object
     */
    closedLoopController = motor.getClosedLoopController();

    /*
     * Display PID coefficients on SmartDashboard
     */
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Applied Output", 0.0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Read PID coefficients from SmartDashboard
     */
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    /*
     * If the PID coefficients on SmartDashboard have changed, write the new values to the controller
     */
    if((p != kP)) { motorConfig.closedLoop.p(p); kP = p; }
    if((i != kI)) { motorConfig.closedLoop.i(i); kI = i; }
    if((d != kD)) { motorConfig.closedLoop.d(d); kD = d; }
    if((iz != kIz)) { motorConfig.closedLoop.iZone(iz); kIz = iz; }
    if((ff != kFF)) { motorConfig.closedLoop.velocityFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      motorConfig.closedLoop.outputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /*
     * Reconfigure the motor PID values if they have been adjusted. In the case that no new changes are made,
     * nothing is sent on the CAN bus.
     */
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * ClosedLoopController objects are commanded to a set point using the 
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
    closedLoopController.setReference(rotations, SparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", alternateEncoder.getPosition());
  }
}