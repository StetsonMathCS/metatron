import java.util.Scanner;

public class ArmTest {
	public static void main(String[] args) {
		Scanner in = new Scanner(System.in);
		try {
			Arm arm = new Arm();
			arm.reset();
			/*
			arm.setShoulderPosition(90);
			in.nextLine();
			arm.setShoulderPosition(190);
			in.nextLine();
			arm.setShoulderPosition(180);
			in.nextLine();
			arm.setGripperPosition(5);
			in.nextLine();
			arm.setGripperPosition(1);
			in.nextLine();
			arm.setGripperPosition(10);
			in.nextLine();
			arm.setElbowPosition(90);
			in.nextLine();
			arm.setElbowPosition(220);
			in.nextLine();
			arm.setElbowPosition(180);
			in.nextLine();
			arm.setWristPosition(145);
			in.nextLine();
			arm.setWristPosition(220);
			in.nextLine();
			arm.setWristPosition(180);
			in.nextLine();
			*/
			arm.pickUp();
			in.nextLine();
			arm.setElbowPosition(180);
			in.nextLine();
			arm.setShoulderPosition(180);
			in.nextLine();
			arm.setWristPosition(180);
			in.nextLine();
			arm.putDown();
			in.nextLine();
		} catch(Exception e) {
			e.printStackTrace();
		}

	}
}
