package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SingleDriverTele", group = "LinearOpMode")
public class singleDriver extends LinearOpMode {

    public DcMotor fl, fr, bl,br;//matthew is fat
    public DcMotor intakeR, intakeL;
    public DcMotor flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public Servo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public boolean ninja = false, reverse = false, magUp = false;
    public double multiplier = 1;
    public double lastTime = System.currentTimeMillis();
    public double lastTimeWobble = System.currentTimeMillis();
    public double lastTimeGrabber = System.currentTimeMillis();
    public boolean flapUp = false, armUp = false;
    public OdometryGlobalCoordinatePosition position;
    public final double COUNTS_PER_INCH = 3072;
    public DcMotor verticalLeft, verticalRight, horizontal;
    public Servo rightIntakeHolder, leftIntakeHolder;
    Coordinate shootingPos;
    double shootingAngle;
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //position = new OdometryGlobalCoordinatePosition(fr, br, bl, COUNTS_PER_INCH, 75, 0, 0);
        position = new OdometryGlobalCoordinatePosition(bl, br, fr, COUNTS_PER_INCH, 75, 0, 0, 0);
        //position = new OdometryGlobalCoordinatePosition(br, bl, fr, COUNTS_PER_INCH, 75, 0, 0);

        Thread positionThread = new Thread(position);
        positionThread.start();
        shootingPos = new Coordinate(position);
        shootingAngle = position.radians();
        waitForStart();

        while(opModeIsActive()){

            drive();
            wobbleArm();
            intake();
            shooter();
            if(gamepad1.y){
                goTo(shootingPos, 0.8, shootingAngle, 0.6);
            }

            telemetry.addData("X Position", position.getX() );
            telemetry.addData("Y Position", position.getY() );
            telemetry.addData("left encoder", bl.getCurrentPosition());
            telemetry.addData("right encoder", br.getCurrentPosition());
            telemetry.addData("horizontal encoder", fr.getCurrentPosition());
            telemetry.addData("Orientation (Degrees)", position.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.addData("flap pos:", flap.getPosition ());
            telemetry.update();
            //idle();
        }
        position.stop();
    }
    public void initialize(){
        fl = hardwareMap.get(DcMotor.class , "fl"); //green
        bl = hardwareMap.get(DcMotor.class , "bl"); //red
        fr = hardwareMap.get(DcMotor.class , "fr"); //blue
        br = hardwareMap.get(DcMotor.class , "br"); //white
        intakeR = hardwareMap.get(DcMotor.class, "intakeR"); //green
        intakeL = hardwareMap.get(DcMotor.class, "intakeL"); //red
        in1 = hardwareMap.get(Servo.class, "in1");
        in2 = hardwareMap.get(Servo.class, "in2");

        intakeR.setDirection( DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        flywheel = hardwareMap.get(DcMotor.class, "fw");//black
        flywheel1 = hardwareMap.get(DcMotor.class, "fw1");//silver
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        mag = hardwareMap.get(Servo.class, "mag");
        flap = hardwareMap.get(Servo.class, "flap");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        arm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        arm1.setPosition(0.1);
        arm2.setPosition (0.88);

        leftIntakeHolder = hardwareMap.get(Servo.class,"wallL");
        rightIntakeHolder = hardwareMap.get(Servo.class,"wallR");


        grabber = hardwareMap.get(Servo.class, "wobbleGrabber1");
        grabber2 = hardwareMap.get(Servo.class, "wobbleGrabber2");
        grabber.setPosition(0.13);
        grabber2.setPosition(0.83);
        mag.setPosition(0.5);
        tilt.setPosition(0.52);
        flap.setPosition(0.35);
        leftIntakeHolder.setPosition(0.91);
        rightIntakeHolder.setPosition(0.18);

        telemetry.addData("Status", "Initialized");
    }
    public void drive(){
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x*0.7;
        double v1 = r * Math.cos(robotAngle)*multiplier + rightX;
        double v2 = r * Math.sin(robotAngle)*multiplier - rightX;
        double v3 = r * Math.sin(robotAngle)*multiplier + rightX;
        double v4 = r * Math.cos(robotAngle)*multiplier - rightX;
        if(ninja == false && gamepad1.left_bumper == true && System.currentTimeMillis() >= lastTime + 300){
            ninja = true;
            lastTime = System.currentTimeMillis();
        }
        else if(ninja == true && gamepad1.left_bumper == true && System.currentTimeMillis() >= lastTime + 300){
            ninja = false;
            lastTime = System.currentTimeMillis();
        }
        if(reverse == false && gamepad1.right_bumper == true && System.currentTimeMillis() >= lastTime + 300){
            reverse = true;
            lastTime = System.currentTimeMillis();
        }
        else if(reverse == true && gamepad1.right_bumper == true && System.currentTimeMillis() >= lastTime + 300){
            reverse = false;
            lastTime = System.currentTimeMillis();
        }
        if(ninja==true){
            v1 /= 3;
            v2 /= 3;
            v3 /= 3;
            v4 /= 3;
        }
        if(reverse==true){
            multiplier = -1;
        }
        else{
            multiplier = 1;
        }
        fl.setPower(v1);
        fr.setPower(v2);
        bl.setPower(v3);
        br.setPower(v4);
    }
    public void wobbleArm(){
        if(gamepad1.a == true && armUp == false){
            arm1.setPosition(0.1);
            arm2.setPosition (0.88);
            armUp = true;
            sleep(400);
        }
        else if(gamepad1.a == true && armUp == true){

            arm1.setPosition(0.93);
            arm2.setPosition (0.07);
            armUp = false;
            sleep(400);
        }
        /*
        else if(gamepad2.a == true){
            arm1.setPosition(0.69);
            arm2.setPosition(0.3);
            armUp = false;
        }

         */
        if(gamepad1.x == true && grabber.getPosition()>0.3){
            grabber.setPosition(0.13);
            grabber2.setPosition(0.83);
            sleep(300);
        }
        else if(gamepad1.x == true && grabber.getPosition()<0.3){
            grabber.setPosition(0.63);
            grabber2.setPosition(0.29);
            sleep(300);
        }
    }

    public void intake() {
        if (gamepad1.dpad_down == true) {
            in1.setPosition(1);
            in2.setPosition(0);
            intakeL.setPower(1);
            intakeR.setPower(1);
        } else if (gamepad1.right_trigger >= 0.1) {
            in1.setPosition(0);
            in2.setPosition(1);
            intakeL.setPower(-1);
            intakeR.setPower(-1);
        }
        else{
            in1.setPosition(0.5);
            in2.setPosition(0.5);
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
        //double intakeSpeed = -gamepad2.right_stick_y;
        //intakeL.setPower(intakeSpeed);
        //intakeR.setPower(intakeSpeed);
        /*if(intakeSpeed<0){
            in1.setPosition(1);
            in2.setPosition(0);
        }
        else if(intakeSpeed>0){
            in1.setPosition(0);
            in2.setPosition(1);
        }
        else{
            in1.setPosition(0.5);
            in2.setPosition(0.5);
        }
        if(intakeSpeed!=0){
            telemetry.addData ("Intake", "on");
        }*/
        if (gamepad2.dpad_left == true && leftIntakeHolder.getPosition() > 0.5) {
            leftIntakeHolder.setPosition(.23);
            rightIntakeHolder.setPosition(.84);
            sleep(200);
        } else if (gamepad2.dpad_left == true && leftIntakeHolder.getPosition() < 0.5) {
            leftIntakeHolder.setPosition(0.91);
            rightIntakeHolder.setPosition(0.18);
            sleep(200);
        }
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(pt.x - position.getX(), pt.y - position.y);
        while(opModeIsActive() && distance > 1 && gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
            distance = Math.hypot(pt.x - position.x, pt.y - position.y);

            double absAngleToTarget = Math.atan2(pt.y - position.y, pt.x - position.x);

            double relAngleToPoint = AngleWrap(absAngleToTarget - position.radians() + Math.toRadians(90));
            //System.out.println("Rel " + relAngleToPoint);
            double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
            double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            double movement_x = movementXPower * power;
            double movement_y = movementYPower * power;
            double relTurnAngle = relAngleToPoint - Math.toRadians(90) + preferredAngle;
            double movement_turn = distance > 5 ? Range.clip(relTurnAngle / Math.toRadians(20), -1, 1) * turnSpeed : 0;
            double rx = turnSpeed*Range.clip((AngleWrap(preferredAngle - position.radians()))/Math.toRadians(20), -1, 1);
            //double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
            setMovement(movement_x, movement_y, -rx);
        }
        setMovement(0, 0, 0);
    }
    public void setMovement(double lx, double ly, double rx){
        fl.setPower(Range.clip(ly + lx + rx, -1, 1));
        fr.setPower(Range.clip(ly - lx - rx, -1, 1));
        bl.setPower(Range.clip(ly - lx + rx, -1, 1));
        br.setPower(Range.clip(ly + lx - rx, -1, 1));
    }
    public void shooter() {
        if (gamepad2.dpad_up == true) {
            flap.setPosition(0.48);
        } else if (gamepad2.dpad_down == true) {
            flap.setPosition(0.35);
        }

        if (gamepad1.b == true) {

            mag.setPosition(0.35);

            sleep(150);
            shootingPos = new Coordinate(position);
            shootingAngle = position.radians();
            mag.setPosition(.5);
        }
        if (gamepad1.left_trigger >= 0.1) {
            tilt.setPosition(0.77);
            flywheel.setPower(-0.90);
            flywheel1.setPower(-0.90);
            flap.setPosition(0.42);
            leftIntakeHolder.setPosition(.23);
            rightIntakeHolder.setPosition(.84);
        } else if (gamepad1.dpad_left==true){
            flywheel.setPower(-0.79);
            flywheel1.setPower(-0.79);
            tilt.setPosition(0.77);
            flap.setPosition(0.42);
            leftIntakeHolder.setPosition(.23);
            rightIntakeHolder.setPosition(.84);
        } else{
            flap.setPosition(0.35);
            tilt.setPosition(0.55);
            flywheel.setPower(0);
            flywheel1.setPower(0);
            leftIntakeHolder.setPosition(0.91);
            rightIntakeHolder.setPosition(0.18);

        }

        if (gamepad1.dpad_up == true) {
            int i = 0;
            while (i < 3) {

                mag.setPosition(0.31);

                sleep(150);
                shootingPos = new Coordinate(position);
                shootingAngle = position.radians();
                mag.setPosition(.5);
                sleep(800);
                i++;
                telemetry.addData("i", i);
                telemetry.update();

            }
        }
    }
}
