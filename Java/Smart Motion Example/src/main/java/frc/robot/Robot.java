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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV MAXMotion Guide
 * 
 * The SPARK MAX includes a new control mode, REV MAXMotion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the MAXMotion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Robot extends TimedRobot {
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  @Override
  public void robotInit() {
    // Starting PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Starting MAXMotion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // Initialize motor
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    // Set PID coefficients and feedback sensor to Primary Encoder
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);
    /**
     * MaxMotion coefficients are set on a SPARK MAX Max Motion Configuration object
     * 
     * - maxVelocity() will limit the velocity in RPM of
     * the pid controller in MaxMotion mode
     * - MaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in MaxMotion mode
     * - MaxMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in MaxMotion mode
     */
    motorConfig.closedLoop.maxMotion
      .allowedClosedLoopError(0.1)
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    // Restore defaults and apply the configuration values from the SPARK MAX configuration object
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Create PID controller and encoder objects
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // Display Max Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // Button to toggle between velocity and position Maxmotion modes
    SmartDashboard.putBoolean("Mode", true);
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
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    double setPoint, processVariable;

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
    if((maxV != maxVel)) { motorConfig.closedLoop.maxMotion.maxVelocity(maxV); maxVel = maxV; }
    if((maxA != maxAcc)) { motorConfig.closedLoop.maxMotion.maxAcceleration(maxA); maxAcc = maxA; }
    if((allE != allowedErr)) { motorConfig.closedLoop.maxMotion.allowedClosedLoopError(allE); allowedErr = allE; }

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * As with other PID modes, MAXMotion is set by calling the
     * setReference() method on an existing pid object and setting
     * the control type to: 
     *    - kMAXMotionVelocityControl
     *    - kMAXMotionPositionControl 
     */
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      closedLoopController.setReference(setPoint, SparkMax.ControlType.kMAXMotionVelocityControl);
      processVariable = encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      closedLoopController.setReference(setPoint, SparkMax.ControlType.kMAXMotionPositionControl);
      processVariable = encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", motor.getAppliedOutput());
  }
}
