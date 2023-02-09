package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.concurrent.Callable;

public class RobotSystem {
    // region SYSTEM CONSTANTS
    public static final InterruptedException hardStopRequest = new InterruptedException("stop everything !");
    public static final InterruptedException softStopRequest = new InterruptedException("stop running safely");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static boolean allowScoring;
    public static int currentConeHeight;
    // endregion

    // region INITIALIZATION
    public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);

        led.initialize();
        arm.initialize();
        puffer.initialize();
        grabber.initialize();
        elevator.initialize();
        driveTrain.initialize();
    }
    public static void resetAll(){
        RobotSystem.reset();

        grabber.reset();
        elevator.reset();
        driveTrain.reset();
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.reset();

        RobotSystem.hardwareMap = hardwareMap;
        RobotSystem.telemetry   = telemetry  ;
        RobotSystem.gamepad1    = gamepad1   ;
        RobotSystem.gamepad2    = gamepad2   ;
    }
    public static void reset(){
        isStopRequested = false;
        currentConeHeight = 0;
        allowScoring = true;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllTeleOpControllers(){
        RobotSystem.operatorController.start();
        RobotSystem.cycleController.start();
        driveTrain.controller.start();
        elevator.controller.start();
    }

    public static void terminate(){
        resetAll();
        isStopRequested = true;

        telemetry.update();
    }
    // endregion

    // region GENERAL FUNCTIONALITY
    private static final ElapsedTime timePassed = new ElapsedTime();
    public static void sleep(int wantedSleepTime) throws InterruptedException{
        timePassed.reset();
        while(wantedSleepTime > timePassed.milliseconds()){
            if (isStopRequested) {
                throw hardStopRequest;
            }
            Thread.sleep(20);
        }
    }
    public static void await(Callable<Boolean> requirement) throws InterruptedException{
        try{
            while (!requirement.call()) {
                if (isStopRequested){
                    throw hardStopRequest;
                }
                if (Thread.interrupted()){
                    throw softStopRequest;
                }
                Thread.sleep(20);
            }
        }catch (Exception e){
            throw softStopRequest;
        }
    }
    public static void await(Callable<Boolean> requirement, int maxWaitingTime) throws InterruptedException{
        timePassed.reset();
        try{
            while (!requirement.call() && maxWaitingTime > timePassed.milliseconds()) {
                if (isStopRequested){
                    throw hardStopRequest;
                }
                if (Thread.interrupted()){
                    throw softStopRequest;
                }
                Thread.sleep(20);
            }
        }catch (Exception e){
            throw softStopRequest;
        }
    }

    public static class buttonController1 {
        public static boolean a = true;
        public static boolean firstAPress(){
            if (gamepad1.a){
                if(a){
                    a = false;
                    return true;
                }
            } else a = true;
            return false;
        }

        public static boolean x = true;
        public static boolean firstXPress(){
            if (gamepad1.x){
                if(x){
                    x = false;
                    return true;
                }
            } else x = true;
            return false;
        }

        public static boolean y = true;
        public static boolean firstYPress(){
            if (gamepad1.y){
                if(y){
                    y = false;
                    return true;
                }
            } else y = true;
            return false;
        }

        public static boolean b = true;
        public static boolean firstBPress(){
            if (gamepad1.b){
                if(b){
                    b = false;
                    return true;
                }
            } else b = true;
            return false;
        }

        public static boolean right_bumperPress = true;
        public static boolean firstRight_bumperPress(){
            if (gamepad1.right_bumper){
                if(right_bumperPress){
                    right_bumperPress = false;
                    return true;
                }
            } else right_bumperPress = true;
            return false;
        }

        public static boolean right_bumperRelease = true;
        public static boolean firstRight_bumperRelease(){
            if (!gamepad1.right_bumper){
                if(right_bumperRelease){
                    right_bumperRelease = false;
                    return true;
                }
            } else right_bumperRelease = true;
            return false;
        }

        public static boolean dpad_up = true;
        public static boolean firstDpad_up(){
            if (gamepad1.dpad_up){
                if(dpad_up){
                    dpad_up = false;
                    return true;
                }
            } else dpad_up = true;
            return false;
        }

        public static boolean dpad_right = true;
        public static boolean firstDpad_right(){
            if (gamepad1.dpad_right){
                if(dpad_right){
                    dpad_right = false;
                    return true;
                }
            } else dpad_right = true;
            return false;
        }

        public static boolean dpad_left = true;
        public static boolean firstDpad_left(){
            if (gamepad1.dpad_left){
                if(dpad_left){
                    dpad_left = false;
                    return true;
                }
            } else dpad_left = true;
            return false;
        }

        public static boolean dpad_down = true;
        public static boolean firstDpad_down(){
            if (gamepad1.dpad_down){
                if(dpad_down){
                    dpad_down = false;
                    return true;
                }
            } else dpad_down = true;
            return false;
        }
    }
    public static class buttonController2 {
        public static boolean a = true;
        public static boolean firstAPress(){
            if (gamepad2.a){
                if(a){
                    a = false;
                    return true;
                }
            } else a = true;
            return false;
        }

        public static boolean x = true;
        public static boolean firstXPress(){
            if (gamepad2.x){
                if(x){
                    x = false;
                    return true;
                }
            } else x = true;
            return false;
        }

        public static boolean y = true;
        public static boolean firstYPress(){
            if (gamepad2.y){
                if(y){
                    y = false;
                    return true;
                }
            } else y = true;
            return false;
        }

        public static boolean b = true;
        public static boolean firstBPress(){
            if (gamepad2.b){
                if(b){
                    b = false;
                    return true;
                }
            } else b = true;
            return false;
        }

        public static boolean right_bumperPress = true;
        public static boolean firstRight_bumperPress(){
            if (gamepad2.right_bumper){
                if(right_bumperPress){
                    right_bumperPress = false;
                    return true;
                }
            } else right_bumperPress = true;
            return false;
        }

        public static boolean right_bumperRelease = true;
        public static boolean firstRight_bumperRelease(){
            if (!gamepad2.right_bumper){
                if(right_bumperRelease){
                    right_bumperRelease = false;
                    return true;
                }
            } else right_bumperRelease = true;
            return false;
        }

        public static boolean dpad_up = true;
        public static boolean firstDpad_up(){
            if (gamepad2.dpad_up){
                if(dpad_up){
                    dpad_up = false;
                    return true;
                }
            } else dpad_up = true;
            return false;
        }

        public static boolean dpad_right = true;
        public static boolean firstDpad_right(){
            if (gamepad2.dpad_right){
                if(dpad_right){
                    dpad_right = false;
                    return true;
                }
            } else dpad_right = true;
            return false;
        }

        public static boolean dpad_left = true;
        public static boolean firstDpad_left(){
            if (gamepad2.dpad_left){
                if(dpad_left){
                    dpad_left = false;
                    return true;
                }
            } else dpad_left = true;
            return false;
        }

        public static boolean dpad_down = true;
        public static boolean firstDpad_down(){
            if (gamepad2.dpad_down){
                if(dpad_down){
                    dpad_down = false;
                    return true;
                }
            } else dpad_down = true;
            return false;
        }
    }
    // endregion

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.cycleController.isInterrupted()) {
            if (!manual.asyncCollect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.asyncHighCollect.interrupt();
                manual.collect();
            }
            else if (!manual.asyncHighCollect.isAlive()) {
                if (buttonController1.firstDpad_up()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(4);
                }
                if (buttonController1.firstDpad_right()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(3);
                }
                if (buttonController1.firstDpad_down()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(2);
                }
                if (buttonController1.firstDpad_left()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(1);
                }
            } else {
                if (buttonController1.firstDpad_up()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController1.firstDpad_right()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController1.firstDpad_down()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController1.firstDpad_left()) {
                    manual.asyncHighCollect.interrupt();
                }
            }

            if (allowScoring) {
                if      (buttonController1.firstAPress()) { manual.score(elevator.highPosition  ); }
                else if (buttonController1.firstXPress()) { manual.score(elevator.middlePosition); }
                else if (buttonController1.firstYPress()) { manual.score(elevator.lowPosition   ); }
            }
            if (buttonController1.firstBPress()) { manual.asyncScore.interrupt(); }

            if (buttonController1.firstRight_bumperPress()){
                driveTrain.slowMode();
            }
            if (buttonController1.firstRight_bumperRelease() && !manual.asyncCollect.isAlive() && !manual.asyncScore.isAlive()) {
                driveTrain.fastMode();
            }
        }
    });

    public static class manual{
        public static void collect() {
            asyncCollect.start();
        }

        public static void highCollect(int coneNumber) {
            manual.asyncCollect.interrupt();
            currentConeHeight = coneNumber;
            asyncHighCollect.start();
        }
        public static void score(int height) {
            elevator.setWantedPosition(height);
            if (!manual.asyncScore.isAlive()){
                asyncScore.start();
            }
        }

        public static final Thread asyncHighCollect = new Thread(() -> {
            try {
                if (!elevator.isUp()){
                    puffer.goToIn();
                    puffer.release();
                }
                driveTrain.slowMode();
                grabber.goToConeSlow(currentConeHeight);
                grabber.release();

                await(() -> grabber.coneIsInRange() && !grabber.isMoving());

                if (grabber.coneIsInRange()) {
                    grabber.grab();

                    await(grabber::hasCone, 600);
                    if (!manual.asyncScore.isAlive()) {
                        driveTrain.fastMode();
                    }
                    await(() -> !arm.isOut() && !elevator.isUp() && elevator.wantedPosition == elevator.bottomPosition);
                    allowScoring = false;
                    manual.asyncScore.interrupt();

                    grabber.goToIn();

                    await(() -> !grabber.isOut());

                    puffer.grab();
                    puffer.goToMid();
                    grabber.release();

                } else {
                    grabber.goToIn();
                    grabber.release();
                }
            }catch (Exception e){
                if (e == softStopRequest){
                    arm.goToRelativePosition(0);
                    grabber.release();
                    grabber.goToIn();
                }
            }
            if (!manual.asyncScore.isAlive()) {
                driveTrain.fastMode();
            }
            allowScoring = true;
        });

        public static final Thread asyncCollect = new Thread(() -> {
            try {
                if (!elevator.isUp()){
                    puffer.goToIn();
                    puffer.release();
                }
                driveTrain.slowMode();
                grabber.goToCone(0);
                grabber.release();
                sleep(200);
                while (!RobotSystem.manual.asyncCollect.isInterrupted() && !isStopRequested &&
                        (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                    arm.goToRelativePosition(gamepad1.left_trigger);

                    if (grabber.coneIsInRange()) {
                        if (!grabber.isGrabbing()) {
                            grabber.grab();
                        }
                    } else {
                        if (grabber.isGrabbing() && !gamepad2.b) {
                            grabber.release();
                        }
                    }
                }

                if (!manual.asyncScore.isAlive()) {
                    driveTrain.fastMode();
                }

                if (isStopRequested){
                    throw hardStopRequest;
                }

                if (RobotSystem.manual.asyncCollect.isInterrupted()){
                    throw softStopRequest;
                }

                grabber.goToMid();

                arm.goToRelativePosition(0);

                await(() -> !arm.isOut() && !elevator.isUp() && elevator.wantedPosition == elevator.bottomPosition);

                allowScoring = false;
                manual.asyncScore.interrupt();

                grabber.goToIn();

                await(() -> !grabber.isOut(), 900);

                grabber.release();
                puffer.grab();
                puffer.goToMid();


            }catch (Exception e){
                if (e == softStopRequest){
                    arm.goToRelativePosition(0);
                    grabber.release();
                    grabber.goToIn();
                } else if (e != hardStopRequest){
                    telemetry.addData("stopped manual collect completely", e);
                    telemetry.update();
                }
            }
            if (!manual.asyncScore.isAlive()) {
                driveTrain.fastMode();
            }
            allowScoring = true;
        });
        public static final Thread asyncScore = new Thread(()-> {
            try {
                await(elevator::almostReachedWantedPosition);

                puffer.goToOut();

                sleep(275);
                await(() -> gamepad1.right_trigger > 0);

                puffer.release();

                await(() -> gamepad1.right_trigger == 0);

                throw softStopRequest;
            }catch (Exception e1){
                if (!e1.equals(hardStopRequest)){
                    try {
                        puffer.goToIn();

                        sleep(200);

                        elevator.setWantedPosition(elevator.bottomPosition);

                        await(() -> !elevator.isUp());

                        sleep(300);
                        if (!grabber.isOut() && grabber.hasCone()){
                            puffer.goToMid();
                        }
                        else {
                            puffer.release();
                        }

                    } catch (Exception e2){
                        telemetry.addData("stopped manual score from safely stopping", e2);
                        telemetry.update();
                    }
                }
            }
        });
    }
    // endregion

    // region OPERATOR CONTROLLED FUNCTIONS
    private static final Thread operatorController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.operatorController.isInterrupted()) {
            if(buttonController2.firstAPress()){
                driveTrain.resetFieldOriented();
            }

            if (buttonController2.firstBPress()){
                if (grabber.isGrabbing()){
                    grabber.release();
                } else {
                    grabber.grab();
                }
            }

            if (buttonController2.firstXPress()){
                if (puffer.isGrabbing()){
                    puffer.release();
                    puffer.goToIn();
                } else {
                    puffer.grab();
                    puffer.goToMid();
                }
            }

            if (buttonController2.firstYPress()){
                if (puffer.isGrabbing()){
                    puffer.release();
                    puffer.goToIn();
                } else {
                    puffer.grab();
                    puffer.goToOut();
                }
            }

            if (buttonController2.firstDpad_down()){
                grabber.offset -= 0.01;
            } else if (buttonController2.firstDpad_up()){
                grabber.offset += 0.01;

            } else if (buttonController2.firstDpad_right()){
                grabber.offset = 0;
            }
        }
    });
    // endregion

    // region AUTONOMOUS FUNCTIONS
    private static abstract class auto {
        // region INITIALIZATION
        public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
            RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);
            initialize();

            led.initialize();
            arm.initialize();
            puffer.initialize();
            grabber.initialize();
            elevator.initialize();
            driveTrain.initializeForAuto(drive.imu, drive.motors);
        }

        public static void initialize(){
            // region INITIALIZE CAMERA
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
            // endregion

            // region INITIALIZE DRIVE
            drive = new SampleMecanumDrive(hardwareMap);
            // endregion
        }

        public static void startAllAutonomousControllers(Callable<Boolean> opModeIsActive){
            auto.opModeIsActive = opModeIsActive;
            elevator.controller.start();
            safetyListener.start();
        }
        private static final Thread safetyListener = new Thread(() -> {
            try {
                while (auto.opModeIsActive.call()){
                }
            } catch(Exception e){}

            terminate();
        });
        private static Callable<Boolean> opModeIsActive;

        public static void terminate(){
            RobotSystem.terminate();
            driveTrain.resetFieldOriented();
        }
        // endregion

        // region MOVEMENT
        protected static SampleMecanumDrive drive;
        public static final HashMap<String, Pose2d> positions = new HashMap<>();
        public static final HashMap<String, TrajectorySequence> trajectories = new HashMap<>();

        public static void follow(TrajectorySequence sequence) throws InterruptedException{
            drive.followTrajectorySequenceAsync(sequence);
            while (!isStationary()){
                if (isStopRequested) {
                    throw hardStopRequest;
                }
                drive.update();
            }
        }
        public static void asyncFollow(TrajectorySequence sequence){
            drive.followTrajectorySequenceAsync(sequence);
            followThread.start();
        }
        private static final Thread followThread = new Thread(() -> {
            while (!isStationary() && !isStopRequested){
                drive.update();
            }
        });

        public static void park() throws InterruptedException{
            switch (detection) {
                case (1): {
                    follow(trajectories.get("scoreToPark1"));
                    break;
                }
                case (2): {
                    follow(trajectories.get("scoreToPark2"));
                    break;
                }
                case (3): {
                    follow(trajectories.get("scoreToPark3"));
                    break;
                }

            }
        }

        public static boolean isStationary(){
            return !drive.isBusy();
        }
        // endregion

        // region CAMERA
        // region CAMERA CONSTANTS
        private static final double fx = 578.272;
        private static final double fy = 578.272;
        private static final double cx = 402.145;
        private static final double cy = 221.506;

        // UNITS ARE METERS
        private static final double tagsize = 0.04;
        // endregion
        private static final AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        private static OpenCvWebcam camera;
        public  static int detection = 3;

        public static void closeCamera(){
            camera.closeCameraDevice();
        }
        public static void checkParkingPosition(){
            if(aprilTagDetectionPipeline.getLatestDetections().size() == 1)
            {
                detection = aprilTagDetectionPipeline.getLatestDetections().get(0).id;
            }
            closeCamera();
        }
        // endregion
    }

    public static class safeAuto extends auto{
        public static void initializeTrajectories(boolean isLeft){
            if (isLeft) {

                // region POSITIONS
                positions.put("park1", new Pose2d(60.0, 24.0, Math.toRadians(0.0)));
                positions.put("park2", new Pose2d(36.0, 24.0, Math.toRadians(0.0)));
                positions.put("park3", new Pose2d(12.0, 24.0, Math.toRadians(0.0)));
                positions.put("score", new Pose2d(23.0, 14.0, Math.toRadians(-90.0)));
                positions.put("start", new Pose2d(30.7, 61.4, Math.toRadians(-90.0)));
                positions.put("collect", new Pose2d(36.0, 12.0, Math.toRadians(180.0)));
                positions.put("startToScore_temp0", new Pose2d(36.0, 48.0, Math.toRadians(0.0)));
                positions.put("startToScore_temp1", new Pose2d(36.0, 24.0, Math.toRadians(0.0)));
                positions.put("scoreToPark1_temp0", new Pose2d(48.0, 12.0, Math.toRadians(180.0)));
                positions.put("scoreToPark3_temp0", new Pose2d(14.0, 14.0, Math.toRadians(-45.0)));
                positions.put("scoreToPark2_temp0", new Pose2d(34.0, 14.0, Math.toRadians(-135.0)));
                // endregion

                // region TRAJECTORIES
                // region COLLECT TO SCORE
                trajectories.put("collectToScore", drive.trajectorySequenceBuilder(positions.get("collect"))
                        .setTangent(Math.toRadians(180.0))
                        .splineToSplineHeading(positions.get("score"), Math.toRadians(180.0))
                        .build());
                // endregion
                // region SCORE TO COLLECT
                trajectories.put("scoreToCollect", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(0.0))
                        .splineToSplineHeading(positions.get("collect"), Math.toRadians(0.0))
                        .build());
                // endregion
                // region SCORE TO PARK1
                trajectories.put("scoreToPark1", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(0.0))
                        .splineToSplineHeading(positions.get("scoreToPark1_temp0"), Math.toRadians(0.0))
                        .splineTo(positions.get("park1"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK2
                trajectories.put("scoreToPark2", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(0.0))
                        .splineToSplineHeading(positions.get("scoreToPark2_temp0"), Math.toRadians(45.0))
                        .splineTo(positions.get("park2"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK3
                trajectories.put("scoreToPark3", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(180.0))
                        .splineToSplineHeading(positions.get("scoreToPark3_temp0"), Math.toRadians(135.0))
                        .splineTo(positions.get("park3"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region START TO SCORE
                trajectories.put("startToScore", drive.trajectorySequenceBuilder(positions.get("start"))
                        .setTangent(Math.toRadians(-30.0))
                        .splineToConstantHeading(positions.get("startToScore_temp0"), Math.toRadians(-90.0))
                        .splineToConstantHeading(positions.get("startToScore_temp1"), Math.toRadians(-90.0))
                        .splineToConstantHeading(positions.get("score"), Math.toRadians(180.0))
                        .build());
                // endregion
                // endregion

            } else {

                // region POSITIONS
                positions.put("collect", new Pose2d(-36.0, 12.0, Math.toRadians(0.0)));
                positions.put("score", new Pose2d(-24.0, 12.0, Math.toRadians(270.0)));
                positions.put("park3", new Pose2d(-60.0, 24.0, Math.toRadians(180.0)));
                positions.put("park2", new Pose2d(-36.0, 24.0, Math.toRadians(180.0)));
                positions.put("park1", new Pose2d(-12.0, 24.0, Math.toRadians(180.0)));
                positions.put("start", new Pose2d(-30.7, 61.4, Math.toRadians(270.0)));
                positions.put("scoreToPark3_temp0", new Pose2d(-48.0, 12.0, Math.toRadians(0.0)));
                positions.put("scoreToPark2_temp0", new Pose2d(-34.0, 14.0, Math.toRadians(315.0)));
                positions.put("scoreToPark1_temp0", new Pose2d(-14.0, 14.0, Math.toRadians(225.0)));
                positions.put("startToScore_temp0", new Pose2d(-36.0, 48.0, Math.toRadians(180.0)));
                positions.put("startToScore_temp1", new Pose2d(-36.0, 24.0, Math.toRadians(180.0)));
                // endregion

                // region TRAJECTORIES
                // region COLLECT TO SCORE
                trajectories.put("collectToScore", drive.trajectorySequenceBuilder(positions.get("collect"))
                        .setTangent(Math.toRadians(0.0))
                        .splineToSplineHeading(positions.get("score"), Math.toRadians(0.0))
                        .build());
                // endregion
                // region SCORE TO COLLECT
                trajectories.put("scoreToCollect", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(180.0))
                        .splineToSplineHeading(positions.get("collect"), Math.toRadians(180.0))
                        .build());
                // endregion
                // region SCORE TO PARK3
                trajectories.put("scoreToPark3", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(180.0))
                        .splineToSplineHeading(positions.get("scoreToPark3_temp0"), Math.toRadians(180.0))
                        .splineTo(positions.get("park3"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK2
                trajectories.put("scoreToPark2", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(180.0))
                        .splineToSplineHeading(positions.get("scoreToPark2_temp0"), Math.toRadians(135.0))
                        .splineTo(positions.get("park2"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK1
                trajectories.put("scoreToPark1", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(0.0))
                        .splineToSplineHeading(positions.get("scoreToPark1_temp0"), Math.toRadians(45.0))
                        .splineTo(positions.get("park1"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region START TO SCORE
                trajectories.put("startToScore", drive.trajectorySequenceBuilder(positions.get("start"))
                        .setTangent(Math.toRadians(210.0))
                        .splineToConstantHeading(positions.get("startToScore_temp0"), Math.toRadians(270.0))
                        .splineToConstantHeading(positions.get("startToScore_temp1"), Math.toRadians(270.0))
                        .splineToConstantHeading(positions.get("score"), Math.toRadians(0.0))
                        .build());
                // endregion
                // endregion

            }

            drive.setPoseEstimate(positions.get("start"));
        }

        // region CYCLE
        public static void cycle(int cycleAmount) throws InterruptedException{
            for (int coneHeight = 4; coneHeight > 4 - cycleAmount; coneHeight--) {
                score();
                collect(coneHeight);
            }
            score();
        }

        public static void score() throws InterruptedException{
            elevator.wantedPosition = elevator.highPosition;
            await(elevator::almostReachedWantedPosition, 1000);
            puffer.autonomousGoToOut();
            sleep(275);
            puffer.release();
            sleep(300);
            puffer.autonomousGoToIn();
            sleep(100);
            elevator.wantedPosition = elevator.bottomPosition;
        }
        public static void collect(int coneHeight) throws InterruptedException{
            asyncFollow(trajectories.get("scoreToCollect"));
            await(safeAuto::isStationary);
            grabber.goToConeSlow(coneHeight);
            arm.goToRelativePosition(1);
            await(grabber::coneIsInRange, 1500);
            grabber.grab();
            await(grabber::hasCone, 800);
            sleep(400);
            grabber.goToMid();
            sleep(300);
            arm.goToRelativePosition(0);
            await(() -> !arm.isOut() && !elevator.isUp(), 1000);
            asyncFollow(trajectories.get("collectToScore"));
            grabber.goToIn();
            await(() -> !grabber.isOut(), 1000);
            grabber.release();
            puffer.grab();
            puffer.goToMid();
        }
        // endregion
    }

    public static class regularAuto extends auto{
        public static void initializeTrajectories(boolean isLeft){
            if (isLeft) {
                // region POSITIONS
                positions.put("park1", new Pose2d(60.0, 36.0, Math.toRadians(0.0)));
                positions.put("park2", new Pose2d(36.0, 36.0, Math.toRadians(0.0)));
                positions.put("park3", new Pose2d(12.0, 36.0, Math.toRadians(0.0)));
                positions.put("score", new Pose2d(37.0, 5, Math.toRadians(-160.0)));
                positions.put("start", new Pose2d(31.0, 63.0, Math.toRadians(-90.0)));
                positions.put("scoreToPark1_temp1", new Pose2d(60.0, 24.0, Math.toRadians(0.0)));
                positions.put("scoreToPark3_temp0", new Pose2d(24.0, 12.0, Math.toRadians(0.0)));
                positions.put("scoreToPark3_temp1", new Pose2d(12.0, 24.0, Math.toRadians(0.0)));
                positions.put("startToScore_temp0", new Pose2d(36.0, 48.0, Math.toRadians(0.0)));
                positions.put("startToScore_temp1", new Pose2d(36.0, 24.0, Math.toRadians(0.0)));
                positions.put("scoreToPark1_temp0", new Pose2d(48.0, 12.0, Math.toRadians(180.0)));
                positions.put("scoreToPark2_temp0", new Pose2d(36.0, 24.0, Math.toRadians(-90.0)));
                // endregion

                // region TRAJECTORIES
                // region SCORE TO PARK1
                trajectories.put("scoreToPark1", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(30.0))
                        .splineToSplineHeading(positions.get("scoreToPark1_temp0"), Math.toRadians(0.0))
                        .splineTo(positions.get("scoreToPark1_temp1"), Math.toRadians(90.0))
                        .splineTo(positions.get("park1"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK2
                trajectories.put("scoreToPark2", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(90.0))
                        .splineToSplineHeading(positions.get("scoreToPark2_temp0"), Math.toRadians(90.0))
                        .splineToConstantHeading(positions.get("park2"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK3
                trajectories.put("scoreToPark3", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(120.0))
                        .splineToSplineHeading(positions.get("scoreToPark3_temp0"), Math.toRadians(180.0))
                        .splineTo(positions.get("scoreToPark3_temp1"), Math.toRadians(90.0))
                        .splineTo(positions.get("park3"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region START TO SCORE
                trajectories.put("startToScore", drive.trajectorySequenceBuilder(positions.get("start"))
                        .setTangent(Math.toRadians(-40.0))
                        .splineToConstantHeading(positions.get("startToScore_temp0"), Math.toRadians(-90.0))
                        .splineToConstantHeading(positions.get("startToScore_temp1"), Math.toRadians(-90.0))
                        .splineToSplineHeading(positions.get("score"), Math.toRadians(-90.0))
                        .build());
                // endregion
                // endregion

            } else {

                // region POSITIONS
                positions.put("score", new Pose2d(-36.0, 6.0, Math.toRadians(340.0)));
                positions.put("park3", new Pose2d(-60.0, 36.0, Math.toRadians(180.0)));
                positions.put("park2", new Pose2d(-36.0, 36.0, Math.toRadians(180.0)));
                positions.put("park1", new Pose2d(-12.0, 36.0, Math.toRadians(180.0)));
                positions.put("start", new Pose2d(-31.0, 63.0, Math.toRadians(270.0)));
                positions.put("scoreToPark3_temp0", new Pose2d(-48.0, 12.0, Math.toRadians(0.0)));
                positions.put("scoreToPark3_temp1", new Pose2d(-60.0, 24.0, Math.toRadians(180.0)));
                positions.put("scoreToPark2_temp0", new Pose2d(-36.0, 24.0, Math.toRadians(270.0)));
                positions.put("scoreToPark1_temp0", new Pose2d(-24.0, 12.0, Math.toRadians(180.0)));
                positions.put("scoreToPark1_temp1", new Pose2d(-12.0, 24.0, Math.toRadians(180.0)));
                positions.put("startToScore_temp0", new Pose2d(-36.0, 48.0, Math.toRadians(180.0)));
                positions.put("startToScore_temp1", new Pose2d(-36.0, 24.0, Math.toRadians(180.0)));
                // endregion

                // region TRAJECTORIES
                // region SCORE TO PARK3
                trajectories.put("scoreToPark3", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(150.0))
                        .splineToSplineHeading(positions.get("scoreToPark3_temp0"), Math.toRadians(180.0))
                        .splineTo(positions.get("scoreToPark3_temp1"), Math.toRadians(90.0))
                        .splineTo(positions.get("park3"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK2
                trajectories.put("scoreToPark2", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(90.0))
                        .splineToSplineHeading(positions.get("scoreToPark2_temp0"), Math.toRadians(90.0))
                        .splineToConstantHeading(positions.get("park2"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region SCORE TO PARK1
                trajectories.put("scoreToPark1", drive.trajectorySequenceBuilder(positions.get("score"))
                        .setTangent(Math.toRadians(60.0))
                        .splineToSplineHeading(positions.get("scoreToPark1_temp0"), Math.toRadians(0.0))
                        .splineTo(positions.get("scoreToPark1_temp1"), Math.toRadians(90.0))
                        .splineTo(positions.get("park1"), Math.toRadians(90.0))
                        .build());
                // endregion
                // region START TO SCORE
                trajectories.put("startToScore", drive.trajectorySequenceBuilder(positions.get("start"))
                        .setTangent(Math.toRadians(220.0))
                        .splineToConstantHeading(positions.get("startToScore_temp0"), Math.toRadians(270.0))
                        .splineToConstantHeading(positions.get("startToScore_temp1"), Math.toRadians(270.0))
                        .splineToSplineHeading(positions.get("score"), Math.toRadians(270.0))
                        .build());
                // endregion
                // endregion

            }

            drive.setPoseEstimate(positions.get("start"));
            trajectories.put("temp", drive.trajectorySequenceBuilder(positions.get("start"))
                    .setTangent(Math.toRadians(0.0))
                    .splineToConstantHeading(new Pose2d(36, 36, Math.toRadians(-90)), Math.toRadians(-90.0))
                    .build());
        }

        // region CYCLE
        public static void cycle(int cycleAmount) throws InterruptedException{
            driveTrain.goToAngle(RobotSystem.regularAuto.positions.get("score").getHeading(), 1000);
            for (int coneHeight = 4; coneHeight > 4 - cycleAmount; coneHeight--) {
                scoreAndPrepare(coneHeight);
                driveTrain.goToAngle(RobotSystem.regularAuto.positions.get("score").getHeading(), 1000);
                collect();
            }
            score();
        }

        public static void score() throws InterruptedException{
            elevator.wantedPosition = elevator.highPosition;
            await(elevator::almostReachedWantedPosition, 1000);
            puffer.autonomousGoToOut();
            sleep(275);
            puffer.release();
            sleep(300);
            puffer.autonomousGoToIn();
            sleep(100);
            elevator.wantedPosition = elevator.bottomPosition;
        }
        public static void scoreAndPrepare(int coneHeight) throws InterruptedException{
            elevator.wantedPosition = elevator.highPosition;
            sleep(500);
            prepareForCollect(coneHeight);
            await(elevator::almostReachedWantedPosition, 3000);
            puffer.autonomousGoToOut();
            sleep(275);
            puffer.release();
            sleep(300);
            puffer.autonomousGoToIn();
            sleep(100);
            elevator.wantedPosition = elevator.bottomPosition;
        }
        public static void prepareForCollect(int cone) throws InterruptedException{
            grabber.goToConeSlow(cone);
            await(() -> !grabber.isMoving());
            arm.goToRelativePosition(0.65);
            grabber.release();
        }
        public static void collect() throws InterruptedException{
            arm.goToRelativePosition(1);
            await(grabber::coneIsInRange, 800);
            grabber.grab();
            await(grabber::hasCone, 800);
            sleep(400);
            grabber.goToMid();
            sleep(300);
            arm.goToRelativePosition(0);
            await(() -> !arm.isOut() && !elevator.isUp(), 1000);
            grabber.goToIn();
            await(() -> !grabber.isOut(), 1000);
            grabber.release();
            puffer.grab();
            puffer.goToMid();
        }
        // endregion
    }
    // endregion
}
