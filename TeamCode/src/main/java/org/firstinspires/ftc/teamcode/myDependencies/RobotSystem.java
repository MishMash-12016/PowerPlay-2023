package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.PipeLine;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
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

        arm.initialize();
        puffer.initialize();
        grabber.initialize();
        elevator.initialize();
        driveTrain.initialize();
    }
    public static void resetAll(){
        RobotSystem.reset();

        led.reset();
        elevator.reset();
        driveTrain.reset();
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.reset();

        RobotSystem.hardwareMap = hardwareMap;
        RobotSystem.telemetry   = telemetry  ;
        RobotSystem.gamepad1    = gamepad1   ;
        RobotSystem.gamepad2    = gamepad2   ;

        // region INITIALIZE IMU
        imu = RobotSystem.hardwareMap.get(BNO055IMU.class, "imu");

        // initializing the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        // endregion
    }
    public static void reset(){
        isStopRequested = false;
        currentConeHeight = 0;
        allowScoring = true;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllTeleOpControllers(){
        RobotSystem.cycleController.start();
        driveTrain.controller.start();
        elevator.controller.start();
    }
    public static void startAllAutonomousControllers(){
        elevator.controller.start();

    }

    public static void terminate(){
        resetAll();
        isStopRequested = true;

        telemetry.update();
    }

    private static BNO055IMU imu;
    public static double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) {
            angle += Math.PI * 2;
        }
        angle = Math.PI * 2 - angle;

        return angle;
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
                Thread.sleep(20);
            }
        }catch (Exception e){
            throw softStopRequest;
        }
        if (isStopRequested) {
            throw hardStopRequest;
        }
    }
    public static void await(Callable<Boolean> requirement, int maxWaitingTime) throws InterruptedException{
        timePassed.reset();
        try{
            while (!requirement.call() && maxWaitingTime > timePassed.milliseconds()) {
                if (isStopRequested){
                    throw hardStopRequest;
                }
                Thread.sleep(20);
            }
        }catch (Exception e){
            throw softStopRequest;
        }
        if (isStopRequested) {
            throw hardStopRequest;
        }
    }

    public static class buttonController {
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
    // endregion

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.cycleController.isInterrupted()) {
            if (!manual.asyncCollect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.asyncHighCollect.interrupt();
                manual.collect();
            }
            else if (!manual.asyncHighCollect.isAlive()) {
                if (buttonController.firstDpad_up()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(4);
                }
                if (buttonController.firstDpad_right()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(3);
                }
                if (buttonController.firstDpad_down()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(2);
                }
                if (buttonController.firstDpad_left()) {
                    manual.asyncCollect.interrupt();
                    manual.highCollect(1);
                }
            } else {
                if (buttonController.firstDpad_up()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController.firstDpad_right()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController.firstDpad_down()) {
                    manual.asyncHighCollect.interrupt();
                }
                if (buttonController.firstDpad_left()) {
                    manual.asyncHighCollect.interrupt();
                }
            }
            if (allowScoring) {
                if      (buttonController.firstAPress()) { manual.score(elevator.highPosition  ); }
                else if (buttonController.firstXPress()) { manual.score(elevator.middlePosition); }
                else if (buttonController.firstYPress()) { manual.score(elevator.lowPosition   ); }
            }
            else if (buttonController.firstBPress()) { manual.asyncScore.interrupt(); }

            if (buttonController.firstRight_bumperPress()){
                driveTrain.slowMode();
            }
            if (buttonController.firstRight_bumperRelease() && !manual.asyncCollect.isAlive() && !manual.asyncScore.isAlive()) {
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
                driveTrain.slowMode();
                grabber.goToConeSlow(currentConeHeight);
                grabber.release();

                await(() -> grabber.coneIsInRange() && !grabber.isMoving());
                if(manual.asyncCollect.isInterrupted()){
                    throw softStopRequest;
                }
                if (grabber.coneIsInRange()) {
                    grabber.grab();

                    await(grabber::hasCone);
                    if(manual.asyncCollect.isInterrupted()){
                        throw softStopRequest;
                    }
                    if (!manual.asyncScore.isAlive()) {
                        driveTrain.fastMode();
                    }
                    await(() -> !arm.isOut() && !elevator.isUp() && elevator.wantedPosition == elevator.bottomPosition);
                    allowScoring = false;
                    manual.asyncScore.interrupt();

                    grabber.goToIn();

                    await(() -> !grabber.isOut());

                    if (grabber.hasCone()) {
                        puffer.grab();
                        puffer.goToMid();
                    }
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
                        if (grabber.isGrabbing()) {
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

                if (grabber.hasCone()) {
                    allowScoring = false;
                    manual.asyncScore.interrupt();

                    grabber.goToIn();

                    await(() -> !grabber.isOut(), 900);

                    grabber.release();
                    puffer.grab();
                    puffer.goToMid();

                } else {
                    grabber.goToIn();
                    grabber.release();
                }

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
                await(() -> (gamepad1.right_trigger > 0 || RobotSystem.manual.asyncScore.isInterrupted()));

                if (manual.asyncScore.isInterrupted()){
                    throw softStopRequest;
                }

                puffer.release();

                await(() -> gamepad1.right_trigger == 0 || RobotSystem.manual.asyncScore.isInterrupted());

                throw softStopRequest;
            }catch (InterruptedException e1){
                if (e1.equals(softStopRequest)){
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
            } catch (Exception e){
                telemetry.addData("stopped manual score completely", e);
                telemetry.update();
            }
        });
    }
    // endregion

    // region AUTONOMOUS FUNCTIONS
    public static class auto {
        public static class safeAuto {
            // region INITIALIZATION
            public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
                RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);
                initialize();

                arm.initialize();
                puffer.initialize();
                grabber.initialize();
                elevator.initialize();
            }

            public static void initialize(){
                initializeCamera();

                // region INITIALIZE DRIVE
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(positions.start);
                // endregion

                // region CREATE TRAJECTORIES
                // region START TO SCORE
                trajectories.startToScore = drive.trajectorySequenceBuilder(positions.start)
                        .setTangent(positions.startToScoreStartingTangent)
                        .splineToConstantHeading(positions.startToScoreTemp1, Math.abs(-90))
                        .splineToConstantHeading(positions.startToScoreTemp2, Math.abs(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(positions.startToScoreTemp3, Math.toRadians(180))
                        .build();
                // endregion

                // region SCORE TO COLLECT
                trajectories.scoreToCollect = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(0)
                        .splineToSplineHeading(positions.collect, Math.toRadians(0))
                        .build();
                // endregion

                // region COLLECT TO SCORE
                trajectories.collectToScore = drive.trajectorySequenceBuilder(positions.collect)
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(positions.score, Math.toRadians(180))
                        .build();
                // endregion

                // region SCORE TO PARK1
                trajectories.scoreToPark1 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(positions.park1, Math.toRadians(0))
                        .build();
                // endregion

                // region SCORE TO PARK2
                trajectories.scoreToPark2 = drive.trajectorySequenceBuilder(positions.score)
                        .splineToConstantHeading(positions.park2, Math.toRadians(0))
                        .build();
                // endregion

                // region SCORE TO PARK3
                trajectories.scoreToPark3 = drive.trajectorySequenceBuilder(positions.score)
                        .splineToConstantHeading(positions.park3, Math.toRadians(180))
                        .build();
                // endregion
                // endregion
            }
            // endregion

            // region MOVEMENT
            public static class trajectories{
                public static TrajectorySequence startToScore;
                public static TrajectorySequence scoreToCollect;
                public static TrajectorySequence collectToScore;
                public static TrajectorySequence scoreToPark1;
                public static TrajectorySequence scoreToPark2;
                public static TrajectorySequence scoreToPark3;
            }
            private static class positions{
                public static final double startToScoreStartingTangent = Math.toRadians(-40);

                public static final Pose2d start   = new Pose2d(32.8, 58, Math.toRadians(-90));
                public static final Pose2d score   = new Pose2d(24, 12, Math.toRadians(-90));
                public static final Pose2d collect = new Pose2d(36, 12, Math.toRadians(180));
                public static final Vector2d park1 = new Vector2d(65, 12);
                public static final Vector2d park2 = new Vector2d(36, 12);
                public static final Vector2d park3 = new Vector2d(12, 12);

                public static final Vector2d startToScoreTemp1 = new Vector2d(36, 48);
                public static final Vector2d startToScoreTemp2 = new Vector2d(36, 12);
                public static final Vector2d startToScoreTemp3 = new Vector2d(score.getX(), score.getY());

            }
            // endregion

            // region FUNCTIONALITY
            public static void cycle(int cycleAmount) throws InterruptedException{
                for (int coneHeight = 4; coneHeight > 4 - cycleAmount; coneHeight--) {
                    goToCollect();

                    follow(trajectories.scoreToCollect);
                    collect(coneHeight);

                    follow(trajectories.collectToScore);
                    goToScore();
                    score();
                }
                finishScoring();
            }

            public static void goToCollect() throws InterruptedException{
                finishScoring();
                await(() -> !elevator.isUp());
            }
            public static void collect(int coneNum) throws InterruptedException{
                await(safeAuto::isStationary);
                grabber.goToConeSlow(coneNum);
                await(() -> !grabber.isMoving());
                grabber.release();
                arm.goToRelativePosition(1);
                sleep(500);
                await(grabber::coneIsInRange);
                grabber.grab();
                await(grabber::hasCone);
                grabber.goToMid();
                sleep(500);
                arm.goToRelativePosition(0);
                await(() -> !arm.isOut());
            }
            public static void goToScore() throws InterruptedException{
                grabber.goToIn();
                await(() -> !grabber.isOut());
                grabber.release();
                puffer.grab();
                puffer.goToMid();
            }
            public static void score() throws InterruptedException{
                elevator.wantedPosition = elevator.highPosition;
                await(elevator::almostReachedWantedPosition);
                puffer.goToOut();
                sleep(500);
                await(safeAuto::isStationary);
                puffer.release();
                sleep(500);
            }
            public static void finishScoring() throws InterruptedException{
                puffer.goToIn();
                sleep(500);
                elevator.wantedPosition = elevator.bottomPosition;
            }
            // endregion

            // region DRIVE
            private static SampleMecanumDrive drive;
            // endregion

            // region CAMERA
            private static OpenCvWebcam webcam;
            public static void initializeCamera(){
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "cam"),
                        hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
                webcam.setPipeline(new PipeLine(false));

                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {}
                });
            }
            // endregion

            // region GLOBAL FUNCTIONALITY
            public static boolean isStationary(){
                return !drive.isBusy();
            }
            public static void follow(TrajectorySequence sequence){
                drive.followTrajectorySequenceAsync(sequence);
                followThread.start();
            }
            private static final Thread followThread = new Thread(() -> {
                while (!isStationary() && !isStopRequested){
                    drive.update();
                }
            });
            public static void terminate(){
                RobotSystem.terminate();
            }
            // endregion
        }

        public static class regularAuto {
            // region INITIALIZATION
            public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
                RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);
                initialize();

                arm.initialize();
                puffer.initialize();
                grabber.initialize();
                elevator.initialize();
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
                drive.setPoseEstimate(positions.start);
                // endregion

                // region CREATE TRAJECTORIES
                // region START TO SCORE
                trajectories.startToScore = drive.trajectorySequenceBuilder(positions.start)
                        .setTangent(positions.startToScoreStartingTangent)
                        .splineToSplineHeading(positions.startToScoreTemp1, Math.toRadians(-90.0))
                        .splineToSplineHeading(positions.startToScoreTemp2, Math.toRadians(-90.0))
                        .splineToSplineHeading(positions.score, Math.toRadians(-90.0))
                        .build();
                // endregion

                // region SCORE TO PARKING 1
                trajectories.scoreToPark1 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(45.0))
                        .splineToSplineHeading(positions.scoreToPark1Temp1, Math.toRadians(0.0))
                        .splineTo(positions.park1, Math.toRadians(90.0))
                        .build();
                // endregion

                // region SCORE TO PARKING 2
                trajectories.scoreToPark2 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(90.0))
                        .splineToSplineHeading(positions.park2, Math.toRadians(90.0)).build();
                // endregion

                // region SCORE TO PARKING 3
                trajectories.scoreToPark3 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(100.0))
                        .splineToSplineHeading(positions.scoreToPark3Temp1, Math.toRadians(180.0))
                        .splineTo(positions.park3, Math.toRadians(90.0))
                        .build();
                // endregion
                // endregion
            }

            public static void terminate(){
                RobotSystem.terminate();
                driveTrain.resetFieldOriented();
            }
            // endregion

            // region MOVEMENT
            private static SampleMecanumDrive drive;
            public static class trajectories{
                public static TrajectorySequence startToScore;
                public static TrajectorySequence scoreToPark3;
                public static TrajectorySequence scoreToPark2;
                public static TrajectorySequence scoreToPark1;
            }
            private static class positions{
                public static final double startToScoreStartingTangent = Math.toRadians(-40);
                public static final Pose2d start = new Pose2d(30.7, 61.4, Math.toRadians(-90.0));
                public static final Pose2d score = new Pose2d(38.0, 5.5, Math.toRadians(-164.0));
                public static final Pose2d park2 = new Pose2d(36.0, 24.0, Math.toRadians(-90.0));
                public static final Vector2d park1 = new Vector2d(60, 24.0);
                public static final Vector2d park3 = new Vector2d(16.0, 24.0);

                public static final Pose2d scoreToPark1Temp1 = new Pose2d(48, 12, Math.toRadians(180.0));

                public static final Pose2d scoreToPark3Temp1 = new Pose2d(24, 16, Math.toRadians(0.0));

                public static final Pose2d startToScoreTemp1 = new Pose2d(36.0, 48.0, Math.toRadians(-90.0));
                public static final Pose2d startToScoreTemp2 = new Pose2d(36.0, 24.0, Math.toRadians(-90.0));
            }
            public static void follow(TrajectorySequence sequence){
                drive.followTrajectorySequenceAsync(sequence);
                while (!isStationary() && !isStopRequested){
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
            // endregion

            // region CYCLE
            public static void cycle(int cycleAmount) throws InterruptedException{
                for (int coneHeight = 4; coneHeight > 4 - cycleAmount; coneHeight--) {
                    scoreAndPrepare(coneHeight);
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
            public static void getLatestDetection(){
                if(aprilTagDetectionPipeline.getLatestDetections().size() == 1)
                {
                    detection = aprilTagDetectionPipeline.getLatestDetections().get(0).id;
                }
            }
            // endregion
        }
    }
    // endregion
}
