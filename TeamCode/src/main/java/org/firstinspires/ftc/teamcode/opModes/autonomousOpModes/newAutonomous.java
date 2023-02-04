package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class newAutonomous extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.auto.safeAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()){}
            RobotSystem.auto.safeAuto.terminate();
        }).start();

        RobotSystem.startAllAutonomousControllers();
        try {
            RobotSystem.auto.safeAuto.follow(RobotSystem.auto.safeAuto.trajectories.startToScore);
            puffer.grab();
            puffer.goToMid();
            RobotSystem.await(RobotSystem.auto.safeAuto::isStationary);
            RobotSystem.auto.safeAuto.score();
            RobotSystem.auto.safeAuto.cycle(1);
            //RobotSystem.await(() -> !elevator.isUp());
            //RobotSystem.safeAuto.follow(RobotSystem.safeAuto.trajectories.scoreToPark1);
        } catch (InterruptedException e){
            RobotSystem.auto.safeAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}
    }
}
