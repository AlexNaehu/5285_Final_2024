// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SensorObject;
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

  private static final String BlueLeft3High = "Blue Left 3 High";
  private static final String BlueMid3High = "Blue Mid 3 High";
  private static final String BlueRight3High = "Blue Right 3 High";
  private static final String RedRight3High = "Red Right 3 High";
  private static final String RedMid3High = "Red Mid 3 High";
  private static final String RedLeft3High = "Red Left 3 High";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static boolean inAuton = false;

  public static SensorObject sensor;
  public UsbCamera cam;

  public static Timer robotClock = new Timer();
  private Timer autonClock = new Timer();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    /*--------------------------------------------------------------------------
    *  Initialize Limelight
    *-------------------------------------------------------------------------*/

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    robotClock.start();
    /*--------------------------------------------------------------------------
    *  Initialize Auton
    *-------------------------------------------------------------------------*/
    m_chooser.setDefaultOption(" Blue Left 3 High", BlueLeft3High);
    m_chooser.addOption("Blue Mid 3 High", BlueMid3High);
    m_chooser.addOption("Blue Right 3 High", BlueRight3High);
    m_chooser.addOption("Red Right 3 High", RedRight3High);
    m_chooser.addOption("Red Mide 3 High", RedMid3High);
    m_chooser.addOption("Red Left 3 High", RedLeft3High);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    
    sensor = new SensorObject();

    SwerveSubsystem.gyro.reset();

    

    RobotContainer.arm.armPivotEnc.reset();
    RobotContainer.intake.wristEnc.reset();
    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.intake.getPivotAngle());

    RobotContainer.arm.pivotPID();    //Shoulder PID

    RobotContainer.intake.wristPivotPID(); //Wrist PID

    RobotContainer.swerveSubsystem.aimBotPID(); //Tracks AprilTags using LimeLight

    //RobotContainer.flyWheel.flyWheelPID();

    //autonClock = new Timer(); //starts in autonInit()
        cam = CameraServer.startAutomaticCapture();
        cam.setFPS(15);
        cam.setResolution(320, 240);
        cam.setPixelFormat(PixelFormat.kMJPEG);
        

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

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area);

    //cam.setFPS(15);
    //cam.setResolution(320, 240);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    inAuton = true;

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.intake.getPivotAngle());

    switch(m_autoSelected){
      case BlueLeft3High:
        m_autonomousCommand = m_robotContainer.getBlueLeft3HighAutonomousCommand();
        // Put default auto code here
        break;
      case BlueMid3High:
        m_autonomousCommand = m_robotContainer.getBlueMid3HighAutonomousCommand();
        // Put custom auto code here
        break;
      case BlueRight3High:
        m_autonomousCommand = m_robotContainer.getBlueRight3HighAutonomousCommand();
        // Put custom auto code here
        break;
      case RedRight3High:
        m_autonomousCommand = m_robotContainer.getRedRight3HighAutonomousCommand();
        // Put custom auto code here
        break;
      case RedMid3High:
        m_autonomousCommand = m_robotContainer.getRedMid3HighAutonomousCommand();
        // Put custom auto code here
        break;
      case RedLeft3High:
        m_autonomousCommand = m_robotContainer.getRedLeft3HighAutonomousCommand();
        // Put custom auto code here
        break;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("I Am running the Command scheduler");
      m_autonomousCommand.schedule();
    }

    //autonClock.reset();
    //autonClock.start();
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
    //autonClock.stop();

    inAuton = false;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.arm.setPivotTargetAngle(RobotContainer.arm.getPivotAngle());
    RobotContainer.intake.setPivotTargetAngle(RobotContainer.intake.getPivotAngle());
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
