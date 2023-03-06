package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous(group = "competition op modes")
public class AutonomousRight extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.regularAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.regularAuto.initializeTrajectories(false);

        waitForStart();
        RobotSystem.regularAuto.checkParkingPosition();

        RobotSystem.regularAuto.startAllAutonomousControllers(this::opModeIsActive);
        puffer.grab();
        puffer.goToMid();

        try {
            RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.get("startToScore"));
            RobotSystem.regularAuto.cycle(5);
            RobotSystem.regularAuto.park();

            while (opModeIsActive() && !RobotSystem.isStopRequested){}

            RobotSystem.regularAuto.terminate();
        } catch (Exception e){
            RobotSystem.regularAuto.terminate();
        }
    }
}
