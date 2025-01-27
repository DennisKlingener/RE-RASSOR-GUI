import { Robot, WheelOperation, Operation, ServoOperation } from '../../src/enumerations/robot-commands';
import HTTP from '../../src/api/web-commands';

const DEFAULT_EXT = '/';

export default class EZRASSOR {

    constructor(host, route = '') {
        this.host = host;
        this.route = route;
        this.setCoordinate(0, 0);
        this.allStop();
    }

    // Getters and Setters
    get host() {
        return this._host;
    }

    set host(value) {
        this._host = value;
    }

    get route() {
        return this._route;
    }

    set route(value) {
        if (value[0] === '/') {
            this._route = value.substring(1);
            return;
        }

        this._route = value;
    }

    get coordinate() {
        return this._coordinate;
    }

    setCoordinate(combinedCoordinate) {

        var coordinates = (typeof combinedCoordinate === "string") ? combinedCoordinate.trim() : '0';
        var split = coordinates.split(',');

        var x = split[0];
        var y = '0';

        if (split.length == 2 && split[1] != '') {
            y = split[1];
        }

        this._coordinate = {
            x: parseInt(x),
            y: parseInt(y)
        }
    }

    // Build complete apiPath for HTTP requests
    get apiPath() {
        console.log('http://' + this.host + '/' + this.route);
        return this.host + this.route;
    }

    // Return custom twist message
    get twistMsg() {
        return JSON.stringify(this._twistMsg);
    }

    get toggleDetectionMsg() {
        return JSON.stringify(this.toggleDetectionMsg);
    }

    // Update only the instruction needed
    updateTwistMsg(instruction) {
        this._twistMsg = instruction;
    }

    updateWheelTwistMsg(operation) {
        var instruction = null;
        switch (operation) {
            case WheelOperation.FORWARD:
                instruction = { wheel_action: { linear_x: WheelOperation.LINEAR_X_FORWARD, angular_z: 0 } }
                break;
            case WheelOperation.BACKWARD:
                instruction = { wheel_action: { linear_x: WheelOperation.LINEAR_X_BACKWARD, angular_z: 0 } }
                break;
            case WheelOperation.LEFT:
                instruction = { wheel_action: { linear_x: 0, angular_z: WheelOperation.ANGULAR_Z_LEFT } }
                break;
            case WheelOperation.RIGHT:
                instruction = { wheel_action: { linear_x: 0, angular_z: WheelOperation.ANGULAR_Z_RIGHT } }
                break;
            case WheelOperation.STOP:
                instruction = { wheel_action: { linear_x: 0, angular_z: 0 } }
                break;
            default:
                console.log('Invalid wheel part selected');
                return;
        }
        this._twistMsg = instruction;
    }
	updateServoCommandMsg(operation) {
		let instruction = null;
		
		switch (operation) {
			case ServoOperation.CAM_SNAP_RIGHT:
				instruction = { servo_action: {linear_x: -99} };
				console.log('Pressed Servo Snap Right');
				break;
			case ServoOperation.CAM_SNAP_CENTER:
				instruction = { servo_action: {linear_x: 50 } };
				console.log('Pressed Servo Center');
				break;
			case ServoOperation.CAM_PAN_LEFT:
				instruction = { servo_action: { linear_x: 75 } };
				console.log('Pressed Servo Pan Left');
				break;
			case ServoOperation.CAM_SNAP_LEFT:
				instruction = { servo_action: { linear_x: 99 } };
				console.log('Pressed Servo Snap Left');
				break;
			case ServoOperation.CAM_PAN_RIGHT:
				instruction = { servo_action: {linear_x: -75 } };
				console.log('Pressed Servo Pan Right');
				break;
			default:
				console.log('Invalid servo command');
				return;
		}
		this._twistMsg = instruction;
	
	}

    updateAutonomyTwistMsg(instruction) {
        if (instruction == Operation.DRIVE || instruction == Operation.FULLAUTONOMY) {
            this._twistMsg = {
                autonomous_toggles: instruction,
                target_coordinate: this.coordinate
            }
            return;
        }

        this._twistMsg = { autonomous_toggles: instruction };
    }

    // Stop all robot operations
    allStop = () => {
        this._twistMsg = {
            autonomous_toggles: 0,
            target_coordinate: this.coordinate,
            wheel_instruction: "none",
            front_arm_instruction: 0,
            back_arm_instruction: 0,
            front_drum_instruction: 0,
            back_drum_instruction: 0
        }

        HTTP.doPost(this.apiPath, this.twistMsg, DEFAULT_EXT);
    }

    // Execute the corresponding robot command from the enumeration items passed in
    executeRobotCommand(part, operation) {
        // Needed when a stop override needs to occur
        if (part == Robot.ALL && operation == Operation.STOP) {
            this.allStop();
            return;
        }

        switch (part) {
            case Robot.FRONTARM:
                this.updateTwistMsg({ front_arm_action: operation });
                break;
            case Robot.BACKARM:
                this.updateTwistMsg({ back_arm_action: operation });
                break;
            case Robot.FRONTDRUM:
                this.updateTwistMsg({ front_drum_action: operation });
                break;
            case Robot.BACKDRUM:
                this.updateTwistMsg({ back_drum_action: operation });
                break;
            case Robot.WHEELS:
                // this.updateWheelTwistMsg({wheel_action:operation});
                this.updateWheelTwistMsg(operation);
                break;
            case Robot.AUTONOMY:
                this.updateAutonomyTwistMsg(routine_action);
                break;
			case Robot.SERVO:
				this.updateServoCommandMsg(operation);
				break;
            default:
                console.log('Invalid robot part selected');
                return;
        }
        HTTP.doPost(this.apiPath, this.twistMsg, DEFAULT_EXT);
    }
}
