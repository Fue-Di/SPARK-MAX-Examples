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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Robot extends TimedRobot {
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * A RelativeEncoder object is constructed using the GetEncoder() method on an 
   * existing SparkMax object. The assumed encoder type is the hall effect,
   * or a sensor type and counts per revolution can be passed in to specify
   * a different kind of sensor. Here, it's a quadrature encoder with 8192 CPR.
   */
  private RelativeEncoder encoder;

  @Override
  public void robotInit() {
    // Initialize SPARK MAX with CAN ID 1
    motor = new SparkMax(deviceID, MotorType.kBrushless);

    // Initialize SPARK MAX Configuration with PID changes
    motorConfig = new SparkMaxConfig();

    /**
     * The PID Controller can be configured to use the primary encoder sensor as its feedback
     * device with the method feedbackDevice() 
     */

    // Define PID coefficients
    kP = 0.008;
    kI = 0.0000001;
    kD = 0.00001;
    kIz = 0;
    kFF = 0.00001;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // Set PID coefficients in configuration
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    // Restore defaults and apply configuration changes to the motor
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    // Get Encoder object
    encoder = motor.getEncoder();

    /**
     * In order to use Closed Loop functionality for a controller, a SparkClosedLoopController object
     * is constructed by calling the getClosedLoopController() method on an existing
     * SparkMax object
     */
    closedLoopController = motor.getClosedLoopController();

    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
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
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

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

    // Update PID value changes if any to the motor
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
    SmartDashboard.putNumber("ProcessVariable", encoder.getPosition());
  }
}
