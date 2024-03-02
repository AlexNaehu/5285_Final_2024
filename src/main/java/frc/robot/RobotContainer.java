package frc.robot;

import java.io.IOException;
import java.nio.file.Path; //?????????????????????????????????????????????????????
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.hal.ThreadsJNI;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WristConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SensorObject;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    
    public final static XboxController controller1 = new XboxController(0);
    public final static XboxController controller2 = new XboxController(1);

    public final static FlyWheel flyWheel = new FlyWheel();

    public final static Arm arm = new Arm();

    public final static Intake intake = new Intake();

    public static SensorObject sensor = new SensorObject();
    private UsbCamera cam;



    private double xPosition;
    private double yPosition;
    private double thetaPosition;
    


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -controller1.getLeftY(),                  //Translate Left(+)/ Right(-)
                () -> -controller1.getLeftX(),                  //Translate Forward(+)/ Backward(-)
                () -> -controller1.getRightX(),                 //Turn Right(+), Turn Left(-)
                () -> !controller2.getBackButton(),             //Field Oriented Function Unless Pressed
                () ->  controller2.getStartButton(),            //Aimbot AprilTag
                () ->  controller1.getRightTriggerAxis(),       //Shoots and Feeds Flywheel
                () ->  controller1.getLeftTriggerAxis(),        //Vacuum Intake
                () ->  controller2.getYButton(),                //Arm and Wrist Low Score Angle
                () ->  controller2.getXButton(),                //Arm and Wrist Pick Up Angle
                () ->  controller2.getBButton()));              //Arm and Wrist Feed Angle

        configureButtonBindings();

        cam = CameraServer.startAutomaticCapture();
        cam.setFPS(15);
        cam.setResolution(320, 240);
        cam.setPixelFormat(PixelFormat.kMJPEG);
    }

    private void configureButtonBindings() {
        /*--------------------------------------------------------------------------
        *  Zero Heading - Manual Control (1)
        *-------------------------------------------------------------------------*/
        if(controller1.getAButton()){
          swerveSubsystem.zeroHeading();
        }

    }


    public Command getLeft2Low1HighAutonomousCommand() {

        // 1. Create trajectory settings

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "output/Score Low #1 (Pre-Load).wpilib.json";
        String trajectoryJSON2 = "output/Grab Close Piece.wpilib.json";
        String trajectoryJSON3 = "output/Score Low #2.wpilib.json";
        String trajectoryJSON4 = "output/Grab Center Piece (2nd from top).wpilib.json";
        String trajectoryJSON5 = "output/Return to Start (Prepare Tele-Op).wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();
        Trajectory trajectory5 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);



        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2);//, trajectory3, trajectory4, trajectory5);

        //Trajectory finalTrajectory = trajectory1.concatenate(trajectory2).concatenate(trajectory3).concatenate(trajectory4).concatenate(trajectory5);


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        // Create a SequentialCommandGroup to execute trajectories sequentially
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        int Counter = 1;
        Command a = new Command() {};
        Command b = new Command() {};
        Command c = new Command() {};
        Command d = new Command() {};
        Command e = new Command() {};
        Command f = new Command() {};
        Command g = new Command() {};
        Command h = new Command() {};
        Command i = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                switch(Counter){
                        case 1: //Score Low #1
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOW_SCORE_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));
                        e = new WaitCommand(0.5);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        h = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE)); //wrist's "PICK_UP_ANGLE" is the same when feeding flywheel/ stowed in robot
                        i = new WaitCommand(0.25);
                        break;
                        case 2: //Grab Close
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new InstantCommand(() -> intake.vacuum(1));
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        break;
                        case 3: //Score Low #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOW_SCORE_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));
                        e = new WaitCommand(0.5);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        h = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE)); //wrist's "PICK_UP_ANGLE" is the same when feeding flywheel/ stowed in robot
                        i = new WaitCommand(0.25);
                        break;
                        case 4: //Grab Center
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new InstantCommand(() -> intake.vacuum(1));
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        break;
                        //Runs last trajectroy back to start
                        
                }
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                a,b,c,
                new SwerveControllerCommand(
                        trajectory,
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()), 
                d,e,f,g,h,i
    );
    Counter ++;
}



        // 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }
    



   public Command getRight2Low1HighAutonomousCommand() {

        // 1. Create trajectory settings

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory 
        
        String trajectoryJSON1 = "output/Score Low #1 (Pre-Load)_0.wpilib.json";
        String trajectoryJSON2 = "output/Grab Close Piece_0.wpilib.json";
        String trajectoryJSON3 = "output/Score Low #2_0.wpilib.json";
        String trajectoryJSON4 = "output/Grab Center Piece (2nd from top)_0.wpilib.json";
        String trajectoryJSON5 = "output/Return to Start (Prepare Tele-Op)_0.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();
        Trajectory trajectory5 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);



        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2);//, trajectory3, trajectory4, trajectory5);

        //Trajectory finalTrajectory = trajectory1.concatenate(trajectory2).concatenate(trajectory3).concatenate(trajectory4).concatenate(trajectory5);


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        // Create a SequentialCommandGroup to execute trajectories sequentially
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        int Counter = 1;
        Command a = new Command() {};
        Command b = new Command() {};
        Command c = new Command() {};
        Command d = new Command() {};
        Command e = new Command() {};
        Command f = new Command() {};
        Command g = new Command() {};
        Command h = new Command() {};
        Command i = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                switch(Counter){
                         case 1: //Score Low #1
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOW_SCORE_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));
                        e = new WaitCommand(0.5);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        h = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE)); //wrist's "PICK_UP_ANGLE" is the same when feeding flywheel/ stowed in robot
                        i = new WaitCommand(0.25);
                        break;
                        case 2: //Grab Close
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new InstantCommand(() -> intake.vacuum(1));
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        break;
                        case 3: //Score Low #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOW_SCORE_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));
                        e = new WaitCommand(0.5);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        h = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE)); //wrist's "PICK_UP_ANGLE" is the same when feeding flywheel/ stowed in robot
                        i = new WaitCommand(0.25);
                        break;
                        case 4: //Grab Center
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new InstantCommand(() -> intake.vacuum(1));
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        break;
                        //Runs last trajectroy back to start
                }
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new SwerveControllerCommand(
                        trajectory,
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()), 
                a,b,c,d,e,f,g,h,i
    );
    Counter ++;
}



        // 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    } 
}
