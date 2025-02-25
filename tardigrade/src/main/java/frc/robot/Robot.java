// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftDrive_leader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax m_leftDrive_follower = new SparkMax(2,MotorType.kBrushed);
  private final SparkMax m_rightDrive_leader = new SparkMax(3,MotorType.kBrushed);
  private final SparkMax m_rightDrive_follower = new SparkMax(4,MotorType.kBrushed);
  private final SparkMax m_scorer = new SparkMax(5,MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final SparkMaxConfig RightleaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig RightfollowerConfig = new SparkMaxConfig();
  private final SparkMaxConfig LeftleaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig LeftfollowerConfig = new SparkMaxConfig();  
  private final SparkMaxConfig commonConfig = new SparkMaxConfig();
  /** Called once at the beginning of the robot program. */
  public Robot() {
    RightleaderConfig.apply(commonConfig)
    .idleMode(IdleMode.kBrake)
    .inverted(true);
    RightfollowerConfig.apply(commonConfig)
      .follow(3);
    LeftleaderConfig.apply(commonConfig)
      .idleMode(IdleMode.kBrake);
    LeftfollowerConfig.apply(commonConfig) 
    .follow(1);

    m_rightDrive_leader.configure(RightleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightDrive_follower.configure(RightfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);// We need to invert one side of the drivetrain so that positive voltages
      
    m_leftDrive_leader.configure(LeftleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftDrive_follower.configure(LeftfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_scorer.configure(commonConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    
    m_robotDrive = new DifferentialDrive(m_leftDrive_leader::set, m_rightDrive_leader::set);

    SendableRegistry.addChild(m_robotDrive, m_leftDrive_leader);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive_leader);
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getRawAxis(1), -m_controller.getRawAxis(4));
    if (m_controller.getRawButton(1)) {
      m_scorer.set(1.0);
  } else {
      m_scorer.set(0);
  }
  };

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
