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
import edu.wpi.first.wpilibj.XboxController.Button;
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


public class RobotContainer {

    public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    
    public final static XboxController controller1 = new XboxController(0);
    public final static XboxController controller2 = new XboxController(1);

    public final static FlyWheel flyWheel = new FlyWheel();

    public final static Arm arm = new Arm();

    public final static Intake intake = new Intake();

    

    



    private double xPosition;
    private double yPosition;
    private double thetaPosition;
    


    public RobotContainer() { //Note for future coder: Order matters when you pass suppliers to get function data, must match instantiation order in SwerveJoystickCmd
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -controller1.getLeftY(),                  //Translate Left(+)/ Right(-)
                () -> -controller1.getLeftX(),                  //Translate Forward(+)/ Backward(-)
                () -> -controller1.getRightX(),                 //Turn Right(+), Turn Left(-)
                () -> !controller2.getBackButton(),             //Field Oriented Function Unless Pressed
                () ->  controller1.getXButton(),                //Aimbot AprilTag
                () ->  controller1.getRightTriggerAxis(),       //Shoots Intake
                () ->  controller1.getLeftTriggerAxis(),        //Vacuum Intake
                () ->  controller2.getYButton(),                //Arm and Wrist Low Score Angle
                () ->  controller2.getXButton(),                //Arm and Wrist Pick Up Angle
                () ->  controller2.getBButton(),                //Arm and Wrist Feed Angle
                () ->  controller2.getRightTriggerAxis(),       //Climber Up
                () ->  controller2.getAButton(),                //Manual move the wrist up
                () ->  controller2.getBackButton(),             //Manual move the wrist down
                () ->  controller1.getLeftBumper(),             //Slow Drive Mode (for fine tune adjustments)
                () ->  controller1.getRightBumper(),            //Fly Wheel Rev-Up
                () ->  controller1.getAButton(),
                () ->  controller2.getLeftTriggerAxis()));              //Reset Heading

        //configureButtonBindings();
        
    }

    /*private void configureButtonBindings() {
        /*--------------------------------------------------------------------------
        *  Zero Heading - Manual Control (1)
        *-------------------------------------------------------------------------*/
        /*if(controller1.getAButton()){
          swerveSubsystem.zeroHeading();
        }

    }*/





    //////////////LIST OF AUTON CODES///////////////////LIST OF AUTON CODES///////////////////LIST OF AUTON CODES/////////////





    public Command getBlueLeft3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/BL Score High#1.wpilib.json";
        //String trajectoryJSON2 = "paths/RR Grab Piece#1.wpilib.json";
        //String trajectoryJSON3 = "paths/BL Score High#2.wpilib.json";
        //String trajectoryJSON4 = "paths/BL Grab Piece#2.wpilib.json";
        //String trajectoryJSON5 = "paths/BL Score High#3.wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        //Trajectory trajectory2 = new Trajectory();
        //Trajectory trajectory3 = new Trajectory();
        //Trajectory trajectory4 = new Trajectory();
        //Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                //Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                //Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                //Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                //Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                //trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                //trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                //trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                //trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1);//, trajectory2);//, trajectory3, trajectory4, trajectory5);
        
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
        Command j = new Command() {};
        Command k = new Command() {};
        Command l = new Command() {};
        Command m = new Command() {};
        Command n = new Command() {};
        Command o = new Command() {};
        Command p = new Command() {};
        Command q = new Command() {};
        Command aimBotOn = new Command() {};
        Command aimBotOff = new Command() {};



        Command z = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        aimBotOn = new InstantCommand(() -> swerveSubsystem.setAimBotState(true));
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.55);
                        c = new InstantCommand(() -> intake.feed(0));
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1.25);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(1);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        aimBotOff = new InstantCommand(() -> swerveSubsystem.setAimBotState(false));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        l = new InstantCommand(() -> intake.vacuum(1));
                        //complete trajectory
                        m = new InstantCommand(() -> intake.vacuum(0));
                        n = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        o = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        p = new WaitCommand(0);
                        q = new WaitCommand(0);
                        z = new WaitCommand(0);
                        }
                        /*if(Counter==2){ //Grab Close Piece #1
                        a = new InstantCommand(() -> intake.feed(1));
                        b = new WaitCommand(0.4);
                        c = new InstantCommand(() -> intake.feed(0));
                        d = new WaitCommand(0);
                        e = new WaitCommand(0);
                        f = new WaitCommand(0);
                        g = new WaitCommand(0);
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        l = new WaitCommand(0);
                        //complete trajectory
                        m = new InstantCommand(() -> flyWheel.shoot(1));
                        n = new WaitCommand(1.25);
                        o = new InstantCommand(() -> intake.vacuum(1));
                        p = new WaitCommand(1);
                        q = new InstantCommand(() -> flyWheel.shoot(0));
                        z = new InstantCommand(() -> intake.vacuum(0));
                        }
                        /*if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new InstantCommand(() -> intake.vacuum(1));
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(1.5);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(1);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        /*if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                /*if (Counter==1){
                        z = new WaitCommand(0);
                }
                else{
                        z = new WaitCommand(0);
                }*/

                Counter ++;

                
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(0.5),
                aimBotOn,a,b,c,d,e,f,g,h,i,aimBotOff,j,k,l,
                 //3 second optional delay to avoid collision
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                m,n,o,p,q,z
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }
    




    public Command getBlueMid3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/BL Score High#1.wpilib.json"; //TODO: create Mid auton and replace BL paths
        String trajectoryJSON2 = "paths/BL Grab Piece#1.wpilib.json";
        //String trajectoryJSON3 = "output/Score Low #2.wpilib.json";
        //String trajectoryJSON4 = "output/Grab Center Piece.wpilib.json";
        //String trajectoryJSON5 = "output/Return to Start (Prepare Tele-Op).wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        //Trajectory trajectory3 = new Trajectory();
        //Trajectory trajectory4 = new Trajectory();
        //Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                //Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                //Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                //Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                //trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                //trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                //trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2);//, trajectory3);

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
        Command j = new Command() {};
        Command k = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.4);
                        c = new InstantCommand(() -> intake.feed(0));
                        //completes trajectory
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(0.75);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        }
                        if(Counter==2){ //Grab Close Piece #1
                        a = new WaitCommand(0.75);
                        b = new InstantCommand(() -> intake.vacuum(1));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        /*if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                Counter ++;
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(1),
                a,b,c,
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                d,e,f,g,h,i,j,k
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }







    public Command getBlueRight3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/BL Score High#1.wpilib.json";           //TODO: replace BL with BR
        String trajectoryJSON2 = "paths/BL Grab Piece#1.wpilib.json";
        //String trajectoryJSON3 = "output/Score Low #2.wpilib.json";
        //String trajectoryJSON4 = "output/Grab Center Piece.wpilib.json";
        //String trajectoryJSON5 = "output/Return to Start (Prepare Tele-Op).wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        //Trajectory trajectory3 = new Trajectory();
        //Trajectory trajectory4 = new Trajectory();
        //Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                //Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                //Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                //Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                //trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                //trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                //trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2);//, trajectory3);

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
        Command j = new Command() {};
        Command k = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.4);
                        c = new InstantCommand(() -> intake.feed(0));
                        //completes trajectory
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(0.75);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        }
                        if(Counter==2){ //Grab Close Piece #1
                        a = new WaitCommand(0.75);
                        b = new InstantCommand(() -> intake.vacuum(1));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        /*if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                Counter ++;
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(1),
                a,b,c,
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                d,e,f,g,h,i,j,k
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }






        //Mirrored BlueLeft3High for RedRight3High

   public Command getRedRight3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/RR Score High#1.wpilib.json";
        //String trajectoryJSON2 = "paths/RR Grab Piece#1.wpilib.json";
        //String trajectoryJSON3 = "paths/RR Score High#2.wpilib.json";
        //String trajectoryJSON4 = "paths/RR Grab Piece#2.wpilib.json";
        //String trajectoryJSON5 = "paths/RR Score High#3.wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        //Trajectory trajectory2 = new Trajectory();
        //Trajectory trajectory3 = new Trajectory();
        //Trajectory trajectory4 = new Trajectory();
        //Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                //Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                //Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                //Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                //Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                //trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                //trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                //trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                //trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1);//, trajectory2);//, trajectory3, trajectory4, trajectory5);

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
        Command j = new Command() {};
        Command k = new Command() {};
        Command l = new Command() {};
        Command m = new Command() {};
        Command n = new Command() {};
        Command o = new Command() {};
        Command p = new Command() {};
        Command q = new Command() {};
        Command aimBotOn = new Command() {};
        Command aimBotOff = new Command() {};



        Command z = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        aimBotOn = new InstantCommand(() -> swerveSubsystem.setAimBotState(true));
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.55);
                        c = new InstantCommand(() -> intake.feed(0));
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1.25);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(1);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        aimBotOff = new InstantCommand(() -> swerveSubsystem.setAimBotState(false));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        l = new InstantCommand(() -> intake.vacuum(1));
                        //complete trajectory
                        m = new InstantCommand(() -> intake.vacuum(0));
                        n = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        o = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        p = new WaitCommand(0);
                        q = new WaitCommand(0);
                        z = new WaitCommand(0);
                        }
                        if(Counter==2){ //Grab Close Piece #1
                        a = new InstantCommand(() -> intake.feed(1));
                        b = new WaitCommand(0.55);
                        c = new InstantCommand(() -> intake.feed(0));
                        d = new WaitCommand(0);
                        e = new WaitCommand(0);
                        f = new WaitCommand(0);
                        g = new WaitCommand(0);
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        l = new WaitCommand(0);
                        //complete trajectory
                        m = new InstantCommand(() -> flyWheel.shoot(1));
                        n = new WaitCommand(1.25);
                        o = new InstantCommand(() -> intake.vacuum(1));
                        p = new WaitCommand(1);
                        q = new InstantCommand(() -> flyWheel.shoot(0));
                        z = new InstantCommand(() -> intake.vacuum(0));
                        }
                        /*if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new InstantCommand(() -> intake.vacuum(1));
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(1.5);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(1);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        /*if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                /*if (Counter==1){
                        z = new WaitCommand(0);
                }
                else{
                        z = new WaitCommand(0);
                }*/

                Counter ++;

                
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(0.5),
                aimBotOn,a,b,c,d,e,f,g,h,i,aimBotOff,j,k,l,
                 //3 second optional delay to avoid collision
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                m,n,o,p,q,z
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }




        //Mirrored BlueMid3High for RedMid3High

    public Command getRedMid3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/Score High#1.wpilib.json";
        String trajectoryJSON2 = "paths/Grab Piece#1.wpilib.json";
        //String trajectoryJSON3 = "output/Score Low #2.wpilib.json";
        //String trajectoryJSON4 = "output/Grab Center Piece.wpilib.json";
        //String trajectoryJSON5 = "output/Return to Start (Prepare Tele-Op).wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        //Trajectory trajectory3 = new Trajectory();
        //Trajectory trajectory4 = new Trajectory();
        //Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                //Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                //Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                //Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                //trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                //trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                //trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2);//, trajectory3);

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
        Command j = new Command() {};
        Command k = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.4);
                        c = new InstantCommand(() -> intake.feed(0));
                        //completes trajectory
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(0.75);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        }
                        if(Counter==2){ //Grab Close Piece #1
                        a = new WaitCommand(0.75);
                        b = new InstantCommand(() -> intake.vacuum(1));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        /*if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                Counter ++;
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(1),
                a,b,c,
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                d,e,f,g,h,i,j,k
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }




        //Mirrored BlueRight3High for RedLeft3High

    public Command getRedLeft3HighAutonomousCommand() {

        // 1. Create trajectory settings
         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        
        String trajectoryJSON1 = "paths/Score High#1.wpilib.json";
        String trajectoryJSON2 = "paths/Grab Piece#1.wpilib.json";
        String trajectoryJSON3 = "paths/Score High#2.wpilib.json";
        String trajectoryJSON4 = "paths/Grab Piece#2.wpilib.json";
        String trajectoryJSON5 = "paths/Score High#3.wpilib.json";
        //String trajectoryJSON6 = "output/Grab Close #2.wpilib.json";
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();
        Trajectory trajectory5 = new Trajectory();
        //Trajectory trajectory6 = new Trajectory();

        try{
                Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
                Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
                Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON5);
                //Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON6);

                trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
                trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
                trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
                trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
                //trajectory6 = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);


        }       catch (IOException ex){
                DriverStation.reportError("Unable to open a trajectoryJSON File.", ex.getStackTrace());
        }

        List<Trajectory> trajectories = Arrays.asList(trajectory1, trajectory2, trajectory3, trajectory4, trajectory5);

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
        Command j = new Command() {};
        Command k = new Command() {};

        // Add commands for each trajectory with a delay between them (   I tried to make this user friendly :>    )
        for (Trajectory trajectory : trajectories) {
                
                        if(Counter==1){//Score High #1
                        a = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        b = new WaitCommand(0.4);
                        c = new InstantCommand(() -> intake.feed(0));
                        //completes trajectory
                        d = new InstantCommand(() -> flyWheel.shoot(1));
                        e = new WaitCommand(1);
                        f = new InstantCommand(() -> intake.vacuum(1));
                        g = new WaitCommand(0.75);
                        h = new InstantCommand(() -> flyWheel.shoot(0));
                        i = new InstantCommand(() -> intake.vacuum(0));
                        j = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        k = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        }
                        if(Counter==2){ //Grab Close Piece #1
                        a = new WaitCommand(0.75);
                        b = new InstantCommand(() -> intake.vacuum(1));
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==3){ //Score High #2
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        /*if(Counter==4){//Grab Close Piece #2
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }
                        if(Counter==5){//Score High #3
                        a = new WaitCommand(0);
                        b = new WaitCommand(0);
                        c = new WaitCommand(0);
                        //completes trajectory
                        d = new InstantCommand(() -> intake.feed(1));           //Alex Maneuver
                        e = new WaitCommand(0.4);
                        f = new InstantCommand(() -> intake.feed(0));
                        g = new InstantCommand(() -> flyWheel.shoot(1));
                        h = new WaitCommand(1);
                        i = new InstantCommand(() -> intake.vacuum(1));
                        j = new WaitCommand(0.75);
                        k = new InstantCommand(() -> flyWheel.shoot(0));
                        }
                        if(Counter==6){//Grab Close Piece #3
                        a = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE));
                        b = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE));
                        c = new WaitCommand(0.75);
                        //completes trajectory
                        d = new WaitCommand(0);
                        e = new InstantCommand(() -> intake.vacuum(0));
                        f = new InstantCommand(() -> arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE));
                        g = new InstantCommand(() -> intake.setPivotTargetAngle(WristConstants.FEED_ANGLE));
                        h = new WaitCommand(0.75);
                        i = new WaitCommand(0);
                        j = new WaitCommand(0);
                        k = new WaitCommand(0);
                        }*/
                        
                Counter ++;
        
                commandGroup.addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new WaitCommand(1),
                a,b,c,
                new SwerveControllerCommand(
                        trajectory,                     //TODO: Idea! To lock heading of robot while driving a trajectory, create an InstantCommand that sets the Ptheta PID variable to 0, then reinstate the value at a later time in the trajectory
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                d,e,f,g,h,i,j,k
                );
        }
        

        
        /*// 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();*/

        
        // 6. Add some init and wrap-up, and return everything
        commandGroup.addCommands(new InstantCommand(() -> swerveSubsystem.stopModules()));
        return commandGroup;
    }
}
