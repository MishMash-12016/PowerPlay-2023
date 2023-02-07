package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class safeAutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.safeAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.safeAuto.initializeTrajectories(true);

        waitForStart();
        RobotSystem.safeAuto.checkParkingPosition();

        RobotSystem.safeAuto.startAllAutonomousControllers(this::opModeIsActive);
        puffer.grab();
        puffer.goToMid();

        try {
            RobotSystem.safeAuto.follow(RobotSystem.safeAuto.trajectories.get("startToScore"));
            //RobotSystem.regularAuto.cycle(5);
            RobotSystem.safeAuto.park();
        } catch (InterruptedException e){
            RobotSystem.safeAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.safeAuto.terminate();
    }
}
