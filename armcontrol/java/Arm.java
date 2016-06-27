import com.fazecast.jSerialComm.*;
import java.util.Scanner;

public class Arm {

	private SerialPort com;
	// gripper has 1-10 range
	int gripperPosition;
	// variables for arm's position (range is 0-360 or so)
	int elbowPosition;
	int wristPosition;
	int shoulderPosition;

	public Arm() throws Exception {
		com = SerialPort.getCommPort("/dev/ttyUSB0");
		com.setBaudRate(9600);
		if(!com.openPort()) {
			throw new Exception("Cannot open serial port.");
		}
		gripperPosition = 0;
		elbowPosition = 0;
		wristPosition = 0;
		shoulderPosition = 0;
	}

	public void reset() throws Exception {
		Scanner in = new Scanner(System.in);
		// set gripper to wide open
		while(true) {
			System.out.print("Is gripper wide open? (y/n): ");
			String answer = in.nextLine();
			if(answer.equals("y")) {
				gripperPosition = 10;
				break;
			} else {
				sendMessage("M4F255D500M4R");	
			}
		}
		// set elbow to horizontal
		while(true) {
			System.out.print("Is elbow straight (180-degrees)? Answer y, above, or below 180 (y/a/b): ");
			String answer = in.nextLine();
			if(answer.equals("y")) {
				elbowPosition = 180;
				break;
			} else if(answer.equals("a")) {
				sendMessage("M1B255D500M1R");	
			} else {
				sendMessage("M1F255D500M1R");
			}
		}
		// set shoulder to horizontal
		while(true) {
			System.out.print("Is shoulder straight (180-degrees)? Answer y, above, or below 180 (y/a/b): ");
			String answer = in.nextLine();
			if(answer.equals("y")) {
				shoulderPosition = 180;
				break;
			} else if(answer.equals("a")) {
				sendMessage("M3F255D500M3R");
			} else {
				sendMessage("M3B255D500M3R");
			}
		}
		// set wrist to horizontal
		while(true) {
			System.out.print("Is wrist straight (180-degrees)? Answer y, above, or below 180 (y/a/b): ");
			String answer = in.nextLine();
			if(answer.equals("y")) {
				wristPosition = 180;
				break;
			} else if(answer.equals("a")) {
				sendMessage("M2F255D500M2R");
			} else {
				sendMessage("M2B255D500M2R");
			}
		}
	}

	public void setGripperPosition(int pos) throws Exception {
		if(pos < 1 || pos > 10) {
			throw new Exception("Invalid position for gripper.");
		}
		if(pos == gripperPosition) return;

		// calculate forward or backward for movement
		String direction;
		if(gripperPosition > pos) {
			direction = "B";
		} else {
			direction = "F";
		}

		// how much to move? (delay)
		int diff = Math.abs(gripperPosition - pos);
		int delay = diff * 125;
		sendMessage("M4" + direction + "255" + "D" + delay + "M4R");
		gripperPosition = pos;
	}

	public void setElbowPosition(int pos) throws Exception {
		if(pos < 0 || pos > 220) {
			throw new Exception("Invalid position for elbow.");
		}
		if(pos == elbowPosition) return;

		// calculate forward or backward for movement
		String direction;
		if(elbowPosition > pos) {
			direction = "B";
		} else {
			direction = "F";
		}

		// how much to move? (delay)
		int diff = Math.abs(elbowPosition - pos);
		int delay = diff * 45;
		sendMessage("M1" + direction + "255" + "D" + delay + "M1R");
		elbowPosition = pos;
	}

	public void setShoulderPosition(int pos) throws Exception {
		if(pos < 0 || pos > 190) {
			throw new Exception("Invalid position for shoulder.");
		}
		if(pos == shoulderPosition) return;

		// calculate forward or backward for movement
		String direction;
		if(shoulderPosition > pos) {
			direction = "F";
		} else {
			direction = "B";
		}

		// how much to move? (delay)
		int diff = Math.abs(shoulderPosition - pos);
		int delay = diff * 50;
		sendMessage("M3" + direction + "255" + "D" + delay + "M3R");
		shoulderPosition = pos;
	}

	public void setWristPosition(int pos) throws Exception {
		if(pos < 145 || pos > 220) {
			throw new Exception("Invalid position for wrist.");
		}
		if(pos == wristPosition) return;

		// calculate forward or backward for movement
		String direction;
		if(wristPosition > pos) {
			direction = "F";
		} else {
			direction = "B";
		}

		// how much to move? (delay)
		int diff = Math.abs(wristPosition - pos);
		int delay = diff * 50;
		sendMessage("M2" + direction + "255" + "D" + delay + "M2R");
		wristPosition = pos;
	}

	public void pickUp() throws Exception {
		setWristPosition(145);
		setElbowPosition(190);
		setShoulderPosition(190);
		setElbowPosition(200);
		setGripperPosition(3);
	}

	public void putDown() throws Exception {
		setWristPosition(145);
		setElbowPosition(190);
		setShoulderPosition(190);
		setElbowPosition(200);
		setGripperPosition(10);
	}
	private void sendMessage(String msg) throws Exception {
		byte[] bytes = msg.getBytes();
		int written = com.writeBytes(bytes, bytes.length);
		if(written != bytes.length) {
			throw new Exception("Could not write message: " + msg);
		}
	}

}
