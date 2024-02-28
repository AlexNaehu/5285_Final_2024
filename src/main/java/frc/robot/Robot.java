// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final String Left2Low1High = "Left2Low1High";
  private static final String Left3HighPrep = "Left3HighPrep";
  private static final String Mid3HighPrep = "Mid3HighPrep";
  private static final String Right2Low1High = "Right2Low1High";
  private static final String Right3HighPrep = "Right3HighPrep";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    /*--------------------------------------------------------------------------
    *  Initialize Auton
    *-------------------------------------------------------------------------*/
    m_chooser.setDefaultOption("Left2Low1High", Left2Low1High);
    m_chooser.addOption("Left3HighPrep", Left3HighPrep);
    m_chooser.addOption("Mid3HighPrep", Mid3HighPrep);
    m_chooser.addOption("Right2Low1High", Right2Low1High);
    m_chooser.addOption("Right3HighPrep", Right3HighPrep);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    
    SwerveSubsystem.gyro.reset();

    RobotContainer.arm.armPivotEnc.reset();
    RobotContainer.intake.wristEnc.reset();
    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());

    RobotContainer.arm.pivotPID();

    RobotContainer.intake.pivotPID(); //Wrist PID
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());

    switch(m_autoSelected){
      case Left2Low1High:
        m_autonomousCommand = m_robotContainer.getLeft2Low1HighAutonomousCommand();
        // Put custom auto code here
        break;
      case Left3HighPrep:
      
        // Put custom auto code here
        break;
      case Mid3HighPrep:
        
        // Put default auto code here
        break;
      case Right2Low1High:
        
        // Put default auto code here
        break;
      case Right3HighPrep:
        
        // Put default auto code here
        break;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
