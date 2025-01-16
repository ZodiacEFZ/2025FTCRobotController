package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive & Move", group="Robot")
public class AutonomousMode extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private Servo down_clip_head;//down_clip_head控制夹子的旋转
    private Servo down_clip_hand;//down_clip_hand控制夹子的抓放

    private Servo clip;//舵机夹子,顶部的那个,目前还没装好

    private DcMotor front_left;// 四个底盘电机
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;

    private DcMotor lift;//抬升电机

    private IMU imu;// 惯性测量单元
    private double multiplier;

    private final double CPower = 0.5;

    private int steps = 0; /*初始 0
                            前进 1
                            前进结束 2
                            */

    @Override
    public void init() {
        multiplier=1.1;
        // 向遥测发送初始化开始信息
        telemetry.addData("初始化" ,"启动");
        //初始化电机
        {
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
//            down_clip_hand.setPosition(0.2);
//            down_clip_hand.scaleRange();//限制范围，待测试
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            down_clip_head.setPosition(0);
            //舵机夹子
            clip = hardwareMap.get(Servo.class,"Clip");
            clip.setPosition(0);
            //抬升电机
            lift = hardwareMap.get(DcMotor.class,"Lift");
            // 从硬件映射中获取四个底盘电机
            front_left = hardwareMap.get(DcMotor.class, "frontLeft");
            front_right = hardwareMap.get(DcMotor.class, "frontRight");
            rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
            rear_right = hardwareMap.get(DcMotor.class, "rearRight");
            // 设置电机的转动方向
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
            rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
            // 设置电机在功率为零时的行为为制动
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        {
            // 从硬件映射中获取 IMU
            imu = hardwareMap.get(IMU.class, "imu");
            // 调整方向参数以匹配机器人
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // 如果没有这个，REV Hub 的方向将被假定为标志朝上 / USB 朝前
            imu.initialize(parameters);
            // 重置 IMU 的偏航角
            imu.resetYaw();
        }
        // 向遥测发送初始化完成信息
        telemetry.addData("初始化", "完毕");
    }

    @Override
    public void start() {
        runtime.reset();// 开始时重置运行时间
    }
    @Override
    public void init_loop() {
        // 初始化循环，这里为空，可能是后续需要添加的功能预留
    }
    @Override
    public void loop() {
        // 调用场心麦卡纳姆驱动方法
        LoopCode();
        // 向遥测发送运行时间信息
        telemetry.addData("运行时间", runtime);
    }

    private void LoopCode(){
        switch (steps){
            case (0): {
                telemetry.addData("Steps:", String.format("%d (初始)", steps));
                steps++;
                break;
            }
            case (1): {
                telemetry.addData("Steps:", String.format("%d (前进)", steps));
                forward(1);
                steps++;
                break;
            }
            default:{
                telemetry.addData("Steps:", String.format("%d (停止🛑)", steps));
                stopChassis();
            }
        }
    }

    private void forward(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","(F)Forward...");
        telemetry.update();
        setChassisPower(CPower, CPower, CPower, CPower);
        while (timer.seconds() <= seconds) {}
        stopChassis();
    }
    private void backward(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","(B)Backward...");
        telemetry.update();
        setChassisPower(-CPower, -CPower, -CPower, -CPower);
        while (timer.seconds() <= seconds) {}
        stopChassis();
    }
    private void leftward(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","(L)Leftward...");
        telemetry.update();
        setChassisPower(-CPower, CPower, CPower, -CPower);
        while (timer.seconds() <= seconds) {}
        stopChassis();
    }
    private void rightward(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","(R)Rightward...");
        telemetry.update();
        setChassisPower(CPower, -CPower, -CPower, CPower);
        while (timer.seconds() <= seconds) {}
        stopChassis();
    }
    private void stopChassis() {
        setChassisPower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Move:","(S)Stop");
        telemetry.update();
    }

    private void setChassisPower(double front_left_power, double front_right_power, double rear_left_power, double rear_right_power) {
        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        rear_left.setPower(rear_left_power);
        rear_right.setPower(rear_right_power);
    }
}
