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

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


/**
 * This is a demo program showing the use of the CANSparkMax class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick stick;
  private static final int deviceID = 1;
  private static final double positionConvFactorDefault = 1.0f;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;

  @Override
  public void robotInit() {
    /**
     * deviceID is the CAN ID of the SPARK MAX you are using.
     * Change to match your setup
     */
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    
    // Initialize SPARK MAX Configurator to hold our motor settings
    motorConfig = new SparkMaxConfig();

    // Modify generic motor parameters
    motorConfig
      .idleMode(IdleMode.kCoast)
      .inverted(false);

    // Modify Encoder Port specific parameter
    motorConfig.encoder
      .positionConversionFactor(positionConvFactorDefault);

    // Restore defaults before applying our changes and saving parameters to SPARK MAX EEPROM.
    // Store the status to check whether the operation succeeded.
    REVLibError status = motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    if(status != REVLibError.kOk) {
      // On Error, display error messages on Smart Dashboard.
      SmartDashboard.putString("Idle Mode", "Error");
      SmartDashboard.putString("Inverted", "Error");
      SmartDashboard.putString("Position Conversion Factor", "Error");
    } else {
      // On success, read back all the data from the SPARK MAX to display on the Smart Dashboard.
      if(motor.configAccessor.getIdleMode() == IdleMode.kCoast) {
        SmartDashboard.putString("Idle Mode", "Coast");
      } else {
        SmartDashboard.putString("Idle Mode", "Brake");
      }
      SmartDashboard.putBoolean("Inverted", motor.configAccessor.getInverted());
      SmartDashboard.putNumber("Position Conversion Factor", motor.configAccessor.encoder.getPositionConversionFactor());
    }

    stick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    // Set motor output to joystick value
    motor.set(stick.getY());
    
    // Periodically read voltage, temperature, and applied output and publish to SmartDashboard
    SmartDashboard.putNumber("Voltage", motor.getBusVoltage());
    SmartDashboard.putNumber("Temperature", motor.getMotorTemperature());
    SmartDashboard.putNumber("Output", motor.getAppliedOutput());
  }
}
