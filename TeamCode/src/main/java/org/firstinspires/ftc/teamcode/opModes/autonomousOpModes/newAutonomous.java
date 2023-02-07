package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class newAutonomous extends LinearOpMode {
    @Override
    public void runOpMode(){/*
        RobotSystem.safeAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()){}
            RobotSystem.safeAuto.terminate();
        }).start();

        RobotSystem.startAllAutonomousControllers();
        try {
            RobotSystem.safeAuto.follow(RobotSystem.safeAuto.trajectories.startToScore);
            puffer.grab();
            puffer.goToMid();
            RobotSystem.await(RobotSystem.safeAuto::isStationary);
            RobotSystem.safeAuto.score();
            RobotSystem.safeAuto.cycle(1);
            //RobotSystem.await(() -> !elevator.isUp());
            //RobotSystem.safeAuto.follow(RobotSystem.safeAuto.trajectories.scoreToPark1);
        } catch (InterruptedException e){
            RobotSystem.safeAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}
    */}
}
