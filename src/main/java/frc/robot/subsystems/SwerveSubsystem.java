package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private SwerveModulePosition[] positions = new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };

    public final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), positions);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), positions, pose);
    }

    public void resetHeading(Rotation2d angle){
        resetOdometry(new Pose2d(getPose().getTranslation(), angle));
    }

    

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), positions);
        /*SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("FL Velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Velocity", backRight.getDriveVelocity());

        SmartDashboard.putNumber("FL Angle", frontLeft.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("FR Angle", frontRight.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("BL Angle", backLeft.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("BR Angle", backRight.getAbsoluteEncoderRad() * 180/Math.PI);*/
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        /*SmartDashboard.putNumber("Goal Angle FL", desiredStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Goal Angle FR", desiredStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Goal Angle BL", desiredStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Goal Angle BR", desiredStates[3].angle.getDegrees());*/
    }



    private Thread aimBotThread;
    public boolean aimBotState = true;
    public double x_steering_command = 0;
    public double y_steering_command = 0;

    public void setAimBotState(boolean state){
        aimBotState = state;
    }


    public void aimBotPID(){
         
        aimBotThread = new Thread(() ->
    {
      double kAimP = 0.025f;  //may need to calibrate kAimP or min_command if aiming causes occilation
      double kZoomP = 0.02f;
      
  
      double x_heading_error;
      double x_steering_adjust;

      //double y_heading_error;
      //double y_steering_adjust;

      while (true)                                  
      {
        //FIRST DO X VALUES

        //Trying to make the heading error 0 relative to the center of the usb camera display
        x_heading_error = -((SmartDashboard.getNumber("Limelight X", 0.0)));//half the width (in pixels) of the camera display
        x_steering_adjust = 0.0;

        SmartDashboard.putNumber("X Heading Error", x_heading_error);

        if(aimBotState==true)// && validTarget == 0.0) //(limelight isn't focusing on a target)
            {

  
            if (Math.abs(x_heading_error) > 1.0)
            {
              if (x_heading_error < 0)
              {
                      x_steering_adjust = kAimP*x_heading_error;
              }
                else if (x_heading_error > 0)
                {
                    x_steering_adjust = kAimP*x_heading_error;
                }
                    
                x_steering_command = x_steering_adjust;// number between -0.32 and 0.32
            
            }

            else
            {
                x_steering_command = 0;
            }
            
            
        
        
            }
    
            SmartDashboard.putNumber("X AimBot Command", x_steering_command);


            //NOW FOR Y VALUES

        //Trying to make the heading error 0 relative to the center of the usb camera display
        /*y_heading_error = -((SmartDashboard.getNumber("Limelight Y", 0.0)));//half the width (in pixels) of the camera display
        y_steering_adjust = 0.0;

        SmartDashboard.putNumber("Y Heading Error", y_heading_error);

        if(aimBotState==true)// && validTarget == 0.0) //(limelight isn't focusing on a target)
            {

  
            if (Math.abs(y_heading_error) > 1.0)
            {
              if (y_heading_error < 0)
              {
                      y_steering_adjust = kZoomP*y_heading_error;
              }
                else if (y_heading_error > 0)
                {
                    y_steering_adjust = kZoomP*y_heading_error;
                }
                    
                y_steering_command = y_steering_adjust;// number between -0.32 and 0.32
            
            }

            else
            {
                y_steering_command = 0;
            }
            
            
        
        
            }
    
            SmartDashboard.putNumber("Y AimBot Command", y_steering_command);
            */
        }
      
    });
        aimBotThread.start();
    
    }

    
}
