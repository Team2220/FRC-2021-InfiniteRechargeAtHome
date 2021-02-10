package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    CANSparkMax leftLeader = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightLeader = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax leftFollower = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    AHRS gyro = new AHRS();
    Pose2d pose;
    double trackWidth = 21;
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
                                                                                                               
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.268, 1.83, 2.44);// kv,ka,ks values in robot
                                                                                       // characterization data logger
    PIDController leftPidController = new PIDController(9.95, 0, 0);// robot characteriarion..
    PIDController rightPidController = new PIDController(9.95, 0, 0);

    public DriveTrain() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftLeader.setInverted(false);
        rightLeader.setInverted(true);

    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    final double gearRatio = 10.75; 
    final double wheelRadius = 3.0; 
    final double wheelDiameter = wheelRadius * 2;
    private final double circumference = Math.PI * wheelDiameter;


    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftLeader.getEncoder().getVelocity() / gearRatio * Units.inchesToMeters(circumference) / 60,
                rightLeader.getEncoder().getVelocity() / gearRatio * Units.inchesToMeters(circumference) / 60);
    }


    public SimpleMotorFeedforward getFeedforward() {
        return feedForward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setOutput(double leftVolts, double rightVolts) {
        leftLeader.set(leftVolts / 12);
        rightLeader.set(rightVolts / 12);
    }

    public PIDController getLeftPIDController() {
        return leftPidController;
    }

    public PIDController getRightPIDController() {
        return rightPidController;
    }

    @Override
    public void periodic() {
        pose = odometry.update(
            getHeading(),
            leftLeader.getEncoder().getPosition() / gearRatio * Units.inchesToMeters(circumference),
            rightLeader.getEncoder().getPosition() / gearRatio * Units.inchesToMeters(circumference)
        );
    }

}
