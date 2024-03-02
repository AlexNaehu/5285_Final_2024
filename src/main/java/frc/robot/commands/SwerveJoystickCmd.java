package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; //Was CommandBase
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.RobotContainer;

public class SwerveJoystickCmd extends Command { //Was "extends CommandBase"

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, flyWheelShootFunction;
    private final Supplier<Double> intakeVacuumFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> aimBotFunction;
    private final Supplier<Boolean> armLowScoreFunction;
    private final Supplier<Boolean> armPickUpFunction;
    private final Supplier<Boolean> armFeedFunction;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> aimBotFunction, Supplier<Double> flyWheelShootFunction,
            Supplier<Double> intakeVacuumFunction,
            Supplier<Boolean> armLowScoreFunction, Supplier<Boolean> armPickUpFunction, Supplier<Boolean> armFeedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.aimBotFunction = aimBotFunction;
        this.flyWheelShootFunction = flyWheelShootFunction;
        this.intakeVacuumFunction = intakeVacuumFunction;
        this.armLowScoreFunction = armLowScoreFunction;
        this.armPickUpFunction = armPickUpFunction; 
        this.armFeedFunction = armFeedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);



        //FlyWheel Shooter
        RobotContainer.flyWheel.shoot(flyWheelShootFunction.get());



        //Intake Vacuum/ Feeder
        RobotContainer.intake.vacuum(intakeVacuumFunction.get());
        RobotContainer.intake.feed(flyWheelShootFunction.get());



        //Arm Movement
        if(armPickUpFunction.get()){
            RobotContainer.arm.setPivotTargetAngle(ArmConstants.PICK_UP_ANGLE);
        }
        if(armLowScoreFunction.get()){
            RobotContainer.arm.setPivotTargetAngle(ArmConstants.LOW_SCORE_ANGLE);
        }
        if(armFeedFunction.get()){
            RobotContainer.arm.setPivotTargetAngle(ArmConstants.LOAD_SHOOTER_ANGLE);
        }

        

        //Wrist Movement
        if(armPickUpFunction.get()){
            RobotContainer.intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE);
        }
        if(armLowScoreFunction.get()){
            RobotContainer.intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE);
        }
    }


    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
