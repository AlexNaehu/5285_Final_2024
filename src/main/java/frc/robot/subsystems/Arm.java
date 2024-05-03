package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class Arm extends SubsystemBase{

    public final DutyCycleEncoder armPivotEnc;

    private final TalonSRX leftShoulder;
    private final TalonSRX rightShoulder;

    private final int leftShoulderID = 2; //TalonSRX ID (not CanSparkMax ID)
    private final int rightShoulderID = 4; //TalonSRX ID (not CanSparkMax ID)

    private volatile double targetAngle;

    private final double ARM_PIVOT_MAX_ANGLE = 175;
    private final double ARM_PIVOT_MIN_ANGLE = -65;

    private final int ARM_PIVOT_ENCODER_PORT = 8; //IO ID
    
    //reference the following for pinout configurations on the NavX: https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/

    private final double PIVOT_OFFSET = 122.13;

    private boolean armTargetHit = false;

    private final double INPUT_THRESHOLD = 0.01;

    public Arm(){
        armPivotEnc = new DutyCycleEncoder(ARM_PIVOT_ENCODER_PORT);
        armPivotEnc.setPositionOffset(PIVOT_OFFSET/360);

        leftShoulder = new TalonSRX(leftShoulderID);
        rightShoulder = new TalonSRX(rightShoulderID);

        leftShoulder.setNeutralMode(NeutralMode.Brake);
        rightShoulder.setNeutralMode(NeutralMode.Brake);

        leftShoulder.configContinuousCurrentLimit(10);
        leftShoulder.configPeakCurrentLimit(15);
        leftShoulder.configPeakCurrentDuration(100);

        rightShoulder.configContinuousCurrentLimit(10);
        rightShoulder.configPeakCurrentLimit(15);
        rightShoulder.configPeakCurrentDuration(100);
        
    }

    public double getPivotAngle() 
    {   //returns a decimal 0-1 of a full rotation (hopefuly)
        return armPivotEnc.getAbsolutePosition()*360-PIVOT_OFFSET;
    }
  
    public double getPivotTargetAngle()
    {
        return targetAngle;
    }

    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public void increaseTargetAngle()
    {
        targetAngle+=5;
    }

    public void decreaseTargetAngle()
    {
        targetAngle-=5; //smaller because gravity pulls the arm that is already trying to go down
    }

    public void setArmTargetHit(boolean state)
    {
        armTargetHit = state;
    }

    public boolean getArmTargetHit()
    {
        return armTargetHit;
    }


    public void setPosition(double pivotTargetAngle)
    {
        RobotContainer.arm.setPivotTargetAngle(pivotTargetAngle);

        RobotContainer.arm.setArmTargetHit(false);
    }

    private SlewRateLimiter powerLimiter = new SlewRateLimiter(10);
    public void pivotPID()
    {
        Thread t = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = -0.009;//TODO: Calibrate kP second
            final double kD = 0.00;
            final double kI = 0.0;
            final double kA = 0.000;//TODO: Calibrate kA first
            final double kF = 0.0;

            double power;            
            double kPpower;
            double kIpower;
            double kDpower;
            double kApower;
            double kFpower;

            Timer armTimer = new Timer();
            armTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 0; 

            double previousDerivative = 0;
            double currentDerivative;    
            double filteredDerivative;  // filtered to prevent derivative jumps, and provide 
                                        //a smoother transition to a new slope of the dE v. dt graph
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentAngle;

            double integral = 0;

            while(true)
            {
                if(targetAngle == ArmConstants.INVALID_ANGLE)
                {
                    Timer.delay(ArmConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = armTimer.get();
                    currentAngle = getPivotAngle(); //TODO: check if cw or ccw means increase or decrease in angle in order to make sure that positive power values calculate when the target angle is ABOVE the current angle

                    currentError = targetAngle - currentAngle;
                    
                    deltaError = currentError - previousError;
                    deltaTime  = currentTime  - previousTime;

                    integral += deltaTime * currentError;

                    currentDerivative = (deltaError / deltaTime);
                    //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);

                        kPpower = kP * currentError;
                        kIpower = kI * integral;
                        kDpower = kD * currentDerivative;//filteredDerivative;
                        kApower = (kA * (Math.cos(Math.toRadians(210 - currentAngle))));//Compensates for angle of the arm
                        kFpower = kF;

                        power = kPpower + kIpower + kDpower + kApower + kFpower;

                        pivotArm(power);
                    
                        //Set up 
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        /*SmartDashboard.putNumber("P Power", kPpower);
                        SmartDashboard.putNumber("I Power", kIpower);
                        SmartDashboard.putNumber("D Power", kDpower);
                        SmartDashboard.putNumber("A Power", kApower);
                        SmartDashboard.putNumber("F Power", kFpower);
                        SmartDashboard.putNumber("Total Arm Power", power);*/

                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        t.start();
    }

    public void pivotArm(double power){
        //using limits

        if (power > 0.7)
        {
            power = 0.7;
        }
        if (power < -0.7)
        {
            power = -0.7;
        }

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                power = 0;
            }
            
                if (pivotAngle >= ARM_PIVOT_MAX_ANGLE) //added the >'(=)' to prevent trying to push past mechanical stops
                {
                    leftShoulder.set(ControlMode.PercentOutput, 0.0);
                    rightShoulder.set(ControlMode.PercentOutput, 0.0);
                    setPivotTargetAngle(ARM_PIVOT_MAX_ANGLE);
                }
                else if (pivotAngle <= ARM_PIVOT_MIN_ANGLE) //added the >'(=)' to prevent trying to push past mechanical stops
                {
                    leftShoulder.set(ControlMode.PercentOutput, 0.0);
                    rightShoulder.set(ControlMode.PercentOutput, 0.0);
                    setPivotTargetAngle(ARM_PIVOT_MIN_ANGLE);
                }
                    else
                    {                             
                        leftShoulder.set(ControlMode.PercentOutput, power);  //both shoulders rotate in the same direction
                        rightShoulder.set(ControlMode.PercentOutput, power); //ccw(positive power values) = up, cw(negative power values) = down
                    }
            
            
            
            

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PIVOT: Target Angle", getPivotTargetAngle());
        SmartDashboard.putNumber("PIVOT: Encoder Angle", getPivotAngle());
    }
    
}
