package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class AutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.auto.regularAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        RobotSystem.auto.regularAuto.getLatestDetection();
        RobotSystem.auto.regularAuto.closeCamera();

        new Thread(() -> {
            while (opModeIsActive()){}
            RobotSystem.auto.regularAuto.terminate();
        }).start();

        RobotSystem.startAllAutonomousControllers();
        try {
            RobotSystem.auto.regularAuto.follow(RobotSystem.auto.regularAuto.trajectories.startToScore);
            puffer.grab();
            puffer.goToMid();
            RobotSystem.await(RobotSystem.auto.regularAuto::isStationary);
            RobotSystem.auto.regularAuto.cycle(5);

            switch (RobotSystem.auto.regularAuto.detection){
                case (1):{
                    RobotSystem.auto.regularAuto.follow(RobotSystem.auto.regularAuto.trajectories.scoreToPark1);
                    break;
                }
                case (2):{
                    RobotSystem.auto.regularAuto.follow(RobotSystem.auto.regularAuto.trajectories.scoreToPark2);
                    break;
                }
                case (3):{
                    RobotSystem.auto.regularAuto.follow(RobotSystem.auto.regularAuto.trajectories.scoreToPark3);
                    break;
                }
            }
        } catch (InterruptedException e){}

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.auto.regularAuto.terminate();
    }
}
