package frc.robot;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;

public class DebugRamseteCommand extends RamseteCommand {

    public DebugRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem[] requirements) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
    }
    public DebugRamseteCommand(
        Trajectory trajectory,
        Supplier<Pose2d> pose,
        RamseteController controller,
        SimpleMotorFeedforward feedforward,
        DifferentialDriveKinematics kinematics,
        Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
        PIDController leftController,
        PIDController rightController,
        BiConsumer<Double, Double> outputVolts,
        Subsystem... requirements) {
            super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements);
        }


	@Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
      //  System.out.println("ABCDEFEXECUTE!!!!!!!!!!!!!!!!!!!!!!*****************" );
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
     //   System.out.println("ABCDEFINITIALIZE!!!!!!!!!!!!!!!!!!!!!!*****************!!!**" );
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        System.out.println("ABCDEFEND!!!!!!!!!!!!!!!!!!!!!!*****************!!!" );
    }
    
}
