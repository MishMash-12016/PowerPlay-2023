package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class AutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.regularAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.regularAuto.initializeTrajectories(true);

        waitForStart();
        RobotSystem.regularAuto.checkParkingPosition();

        RobotSystem.regularAuto.startAllAutonomousControllers(this::opModeIsActive);
        puffer.grab();
        puffer.goToMid();

        try {
            RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.get("startToScore"));
//            RobotSystem.regularAuto.keepAngle(-RobotSystem.regularAuto.positions.get("score").getHeading() + Math.toRadians(90));
//            RobotSystem.await(driveTrain::atWantedAngle);
            RobotSystem.regularAuto.cycle(5);
//            driveTrain.angleHolder.interrupt();
            RobotSystem.regularAuto.park();
        } catch (InterruptedException e){
            RobotSystem.regularAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.regularAuto.terminate();
    }
}
