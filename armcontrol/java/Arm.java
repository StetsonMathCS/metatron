import com.fazecast.jSerialComm.*;

public class Arm {

	private SerialPort com;

	public Arm() throws Exception {
		com = SerialPort.getCommPort("/dev/ttyUSB0");
		com.setBaudRate(9600);
		if(!com.openPort()) {
			throw new Exception("Cannot open serial port.");
		}
	}

	public void moveArm() throws Exception {
		String msg = constructMotorMessage('F', 255, 'F', 255, 'F', 255, 'F', 255);
		msg += constructDelayMessage(1000);
		msg += constructStopMotorsMessage();
		sendMessage(msg);
	}

	private String constructDelayMessage(int delayms) {
		return "D" + delayms;
	}

	private String constructMotorMessage(char m1dir, int m1speed, char m2dir, int m2speed, char m3dir, int m3speed, char m4dir, int m4speed) {
		return "M1" + m1dir + m1speed + "M2" + m2dir + m2speed + "M3" + m3dir + m3speed + "M4" + m4dir + m4speed;
	}

	private String constructStopMotorsMessage() {
		return "M1RM2RM3RM4R";
	}

	private void sendMessage(String msg) throws Exception {
		byte[] bytes = msg.getBytes();
		int written = com.writeBytes(bytes, bytes.length);
		if(written != bytes.length) {
			throw new Exception("Could not write message: " + msg);
		}
	}

}
