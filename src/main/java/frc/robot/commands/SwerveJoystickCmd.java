package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.core.json.WriterBasedJsonGenerator;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; //Was CommandBase
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SwerveJoystickCmd extends Command { //Was "extends CommandBase"

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, flyWheelFeedFunction;
    private final Supplier<Double> intakeVacuumFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> aimBotFunction;
    private final Supplier<Boolean> armLowScoreFunction;
    private final Supplier<Boolean> armPickUpFunction;
    private final Supplier<Boolean> armFeedFunction;
    private final Supplier<Double> climberFunction;
    private final Supplier<Boolean> manualWristUpFunction;
    private final Supplier<Boolean> manualWristDownFunction;
    private final Supplier<Boolean> slowDriveFunction;
    private final Supplier<Boolean> flyWheelShootFunction;
    private final Supplier<Boolean> resetHeadingFunction;
    private final Supplier<Double> wristOnlyFunction;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> aimBotFunction, Supplier<Double> flyWheelFeedFunction,
            Supplier<Double> intakeVacuumFunction,
            Supplier<Boolean> armLowScoreFunction, Supplier<Boolean> armPickUpFunction, Supplier<Boolean> armFeedFunction,
            Supplier<Double> climberFunction, Supplier<Boolean> manualWristUpFunction,
            Supplier<Boolean> manualWristDownFunction, Supplier<Boolean> slowDriveFunction, Supplier<Boolean> flyWheelShootFunction,
            Supplier<Boolean> resetHeadingFunction, Supplier<Double> wristOnlyFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.aimBotFunction = aimBotFunction;
        this.flyWheelFeedFunction = flyWheelFeedFunction;
        this.intakeVacuumFunction = intakeVacuumFunction;
        this.armLowScoreFunction = armLowScoreFunction;
        this.armPickUpFunction = armPickUpFunction; 
        this.armFeedFunction = armFeedFunction;
        this.climberFunction = climberFunction;
        this.manualWristUpFunction = manualWristUpFunction;
        this.manualWristDownFunction = manualWristDownFunction;
        this.slowDriveFunction = slowDriveFunction;
        this.flyWheelShootFunction = flyWheelShootFunction;
        this.resetHeadingFunction = resetHeadingFunction;
        this.wristOnlyFunction = wristOnlyFunction;
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

        //1.25 Slow Drive Function
        if(slowDriveFunction.get()){
                xSpeed = xSpeed*0.15;
                ySpeed = ySpeed*0.15;
                turningSpeed = turningSpeed*0.25;
        }

        //1.5. AimBot Function
        if(aimBotFunction.get()){
        RobotContainer.swerveSubsystem.aimBotState = true;
        turningSpeed = RobotContainer.swerveSubsystem.x_steering_command;
        //ySpeed = RobotContainer.swerveSubsystem.y_steering_command; 
        }        
        
        else{
        RobotContainer.swerveSubsystem.aimBotState = false;
        }

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
        if (fieldOrientedFunction.get() && Robot.inAuton == false) {
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

        if(flyWheelShootFunction.get()){
        RobotContainer.flyWheel.shoot(1);
        }
        else{
        RobotContainer.flyWheel.stop();
        }
            


        //Intake Vacuum/ Feeder
        RobotContainer.intake.feed(flyWheelFeedFunction.get());
        if(intakeVacuumFunction.get()!=0){
        RobotContainer.intake.vacuum(intakeVacuumFunction.get());
        }
        


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
        if(armPickUpFunction.get() && wristOnlyFunction.get()>0.1){
            RobotContainer.intake.setPivotTargetAngle(WristConstants.PICK_UP_ANGLE);
        }
        if(armLowScoreFunction.get() && wristOnlyFunction.get()>0.1){
            RobotContainer.intake.setPivotTargetAngle(WristConstants.LOW_SCORE_ANGLE);
        }
        if(armFeedFunction.get() && wristOnlyFunction.get()>0.1){
            RobotContainer.intake.setPivotTargetAngle(WristConstants.FEED_ANGLE);
        }

        if(manualWristUpFunction.get()){
            RobotContainer.intake.increaseTargetAngle();
        }
        if(manualWristDownFunction.get()){
            RobotContainer.intake.decreaseTargetAngle();
        }

        if(resetHeadingFunction.get()){
            RobotContainer.swerveSubsystem.zeroHeading();
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
