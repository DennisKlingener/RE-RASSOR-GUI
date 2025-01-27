import React from 'react';
import Modal from 'react-native-modal';
import FadeInView from '../../../src/components/app/FadeInView';
import InformationController from './InformationController';
import EZRASSOR from '../../../src/api/ezrassor-service';
import ControllerStyle from '../../../src/styles/controller';
import { Robot, ShoulderOperation, DrumOperation, WheelOperation, Operation } from '../../../src/enumerations/robot-commands';
import {
    Text,
    View,
    TouchableHighlight,
    Pressable,
    Image,
    StatusBar,
    KeyboardAvoidingView,
    TextInput,
} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font from 'expo-font';
import { isIpReachable } from '../../functionality/connection';
import AsyncStorage from '@react-native-async-storage/async-storage';

const CONNECTION_POLLING_INTERVAL = 2000;
const CONNECTION_POLLING_TIMEOUT = 4000;
const CONNECTION_POLLING_VERBOSE = false;

/**
 * React component for the actual rover controller interface.
 */
export default class ControllerScreen extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            autonomyModalVisible: false,
            infoModalVisible: false,
            devModalVisible: false,
            ls: 0,
            rs: 0,
            ipModal: false,
            xyModal: false,
            isLoading: true,
            control: 0,
            xy: '0,0',
            colorValue: { r: 58, g: 61, b: 61 },
            ip: '',
            armInfo: '',
        };
        this.interval = null;

        this.EZRASSOR = new EZRASSOR(this.state.ip);
    }

    async componentDidMount() {
        await Font.loadAsync({ NASA: require('../../../assets/nasa.ttf') });
        await this.getIpFromStorage();

        this.setState({ isLoading: false });

        // Fetch arm info from the Flask server
        try {
            const response = await fetch(`http://${this.state.ip}/arm_info`);
            const armInfo = await response.json();

            // Update state with the received sidekick info
            this.setState({ armInfo: armInfo });

            console.log(this.state.armInfo);
        } catch (error) {
            console.error('Error fetching armInfo info:', error);
        }

        this._unsubscribe = this.props.navigation.addListener('focus', async () => {
            this.getIpFromStorage();
        });

        // Set up the connection polling logic.
        let pollCount = 0;


        const pollInterval = (CONNECTION_POLLING_INTERVAL / 1000.0).toFixed(2);
        const pollTimeout = (CONNECTION_POLLING_TIMEOUT / 1000.0).toFixed(2);
        console.log(`Polling every ${pollInterval}s with a ${pollTimeout}s timeout.`);

        const { passrs, passls } = this.props.route.params
        this.state.ls = passls
        this.state.rs = passrs
    }

    async componentWillUnmount() {
        clearInterval(this.connectionPoller);
        clearInterval(this.interval);
        this._unsubscribe();
    }

    async getIpFromStorage() {
        try {
            const ip = await AsyncStorage.getItem('myIp');

            if (ip != null) {
                this.changeIP(ip);
            }
        } catch (error) {
            // Error retrieving data. Do nothing.
        }
    }

    setAutonomyModalVisible(visible) {
        this.setState({ autonomyModalVisible: visible });
    }

    setInfoModalVisible(visible) {
        this.setState({ infoModalVisible: visible });
    }

    setDevModalVisible(visible) {
        this.setState({ devModalVisible: visible });
    }

    setIPModalVisible(visible) {
        this.setState({ ipModal: visible });
    }

    setXYModalVisible(visible) {
        this.setState({ xyModal: visible });
    }

    changeXY(combined) {
        this.setState({ xy: combined }, () => {
            this.EZRASSOR.setCoordinate(this.state.xy);
        });
    }

    /**
     * Set `this.state.ip` and `this.EZRASSOR.host` to the specified IP + port.
     * 
     * @param {string} ip IP + port.
     */
    changeIP(ip) {
        this.setState({ ip }, () => {
            this.EZRASSOR.host = ip;
        });
    }

    /**
     * Update animation frame before processing click so that opacity can change on click.
     */
    sendOperation(part, operation) {
        requestAnimationFrame(() => {
            this.EZRASSOR.executeRobotCommand(part, operation);
        });
    }


    render() {
        const { ls, rs } = this.state;
        // I.e., don't do full render if font is still loading...
        if (this.state.isLoading) {
            return <View style={{ flex: 1, backgroundColor: '#5D6061' }} />;
        }

        return (
            <View style={ControllerStyle.container}>

                <StatusBar hidden />

                {/* Autonomy popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.autonomyModalVisible}
                    onSwipeComplete={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() =>
                        this.setAutonomyModalVisible(!this.state.autonomyModalVisible)
                    }
                >
                    <Pressable style={{ flex: 1, marginHorizontal: 15, justifyContent: "center" }}>
                        <View>

                            {/* Modal title container. */}
                            <View style={{ flexDirection: "row", marginVertical: 15, justifyContent: "center" }}>
                                <Text style={ControllerStyle.textLarge}>
                                    Activate Autonomous Function(s)
                                </Text>
                            </View>

                            {/* Modal body container. */}
                            <View style={{ flexDirection: "row", justifyContent: 'space-evenly' }}>

                                {/* Button: Drive. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => this.setXYModalVisible(true)}
                                >
                                    <Text style={ControllerStyle.textSmall}>Drive</Text>
                                </Pressable>

                                {/* Button: Dig. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.DIG);
                                    }}
                                >
                                    <Text style={ControllerStyle.textSmall}>Dig</Text>
                                </Pressable>

                                {/* Button: Dump. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.DUMP);
                                    }}
                                >
                                    <Text style={ControllerStyle.textSmall}>Dump</Text>
                                </Pressable>

                                {/* Button: Self-Right. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.SELFRIGHT);
                                    }}
                                >
                                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>
                                        Self - Right
                                    </Text>
                                </Pressable>

                                {/* Button: Full-Auto. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.FULLAUTONOMY);
                                    }}
                                >
                                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>
                                        Auto Mode
                                    </Text>
                                </Pressable>

                            </View>

                        </View>
                    </Pressable>
                </Modal>

                {/* Info popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.infoModalVisible}
                    onSwipeComplete={() => this.setInfoModalVisible(false)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() => this.setInfoModalVisible(false)}
                >
                    <InformationController></InformationController>
                </Modal>

                {/* Drive-autonomy-input modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.xyModal}
                    onSwipeComplete={() => this.setXYModalVisible(false)}
                    swipeDirection="down"
                    onRequestClose={() => {
                        this.setXYModalVisible(false);
                    }}
                >
                    <KeyboardAvoidingView paddingLeft={64} paddingRight={64}>

                        {/* Prompt. */}
                        <Text
                            style={[ControllerStyle.textSmall, ControllerStyle.columnText]}
                        >
                            Enter the X,Y coordinates where the robot will drive to
                        </Text>

                        {/* Input field. */}
                        <TextInput
                            style={ControllerStyle.ipInputBox}
                            onChangeText={(text) => this.changeXY(text)}
                            value={this.state.xy}
                            placeholder="x,y"
                            marginVertical={20}
                        />

                        {/* Done button. */}
                        <Pressable
                            style={{
                                alignItems: 'center',
                                backgroundColor: '#DDDDDD',
                                padding: 10,
                            }}
                            onPress={() => {
                                this.sendOperation(Robot.AUTONOMY, Operation.DRIVE);
                                this.setXYModalVisible(false);
                            }}
                        >
                            <Text>Done</Text>
                        </Pressable>

                    </KeyboardAvoidingView>
                </Modal>

                {/* Controller screen top row controls. */}
                <FadeInView style={ControllerStyle.headerContainer}>
                    <View style={ControllerStyle.evenlySpaceIcons}>
                        {/* Info button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => { this.setInfoModalVisible(true); }}
                        >
                            <FontAwesome name="info-circle" size={50} color="#fff" />
                        </Pressable>

                        {/* Set-IP button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("IPConnect Screen")}
                        >
                            <FontAwesome name="wifi" size={50} color="#fff" />
                        </Pressable>
                    </View>

                    {/* Title. */}
                    <Text style={ControllerStyle.textMedium}>EZ-RASSOR Controller</Text>

                    <View style={ControllerStyle.evenlySpaceIcons}>
                        {/* Stop-rover button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => {
                                this.sendOperation(Robot.ALL, Operation.STOP);
                                this.setState({ ls: 0, rs: 0 });
                            }}
                        >
                            <FontAwesome
                                name="stop-circle-o"
                                size={50}
                                color="#fff"
                            />
                        </Pressable>

                        {/*Paver Arm controls*/}
                        {/*<Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Paver Arm Controller Screen", { currentIp: this.state.ip })}
                        >
                            <MaterialCommunityIcons
                                name="robot-industrial"
                                size={50}
                                color="#fff"
                            />
                        </Pressable>*/}

                        {/* Autonomy button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => { this.setAutonomyModalVisible(true); }}
                        >
                            <MaterialCommunityIcons
                                name="robot"
                                size={50}
                                color="#fff"
                            />
                        </Pressable>

                        {/* TODO make sure this passes the IP address to video screen  */}
                        {/* View-Video-Screen button.*/}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Video Controller Screen", {
                                currentIp: this.state.ip,
                                passls: this.state.ls,
                                passrs: this.state.rs
                            })}
                        >
                            <FontAwesome
                                name="video-camera"
                                size={50}
                                color="#fff" />
                        </Pressable>
                    </View>

                </FadeInView>

                {/* Body container. */}
                <FadeInView style={ControllerStyle.buttonLayoutContainer}>

                    {/* Wheel buttons. */}
                    <View style={ControllerStyle.wheelFunctionContainer}>


                        {/* Forwards button. */}
                        <Pressable style={ControllerStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.FORWARD);
                                if (ls < 3)
                                    this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                if (rs < 3)
                                    this.setState(prevState => ({ rs: prevState.rs + 1 }));
                            }}
                        >
                            <FontAwesome name="chevron-up" size={50} color="#fff" />
                        </Pressable>


                        {/* Left, Stop, and Right buttons. */}
                        <View style={{ flex: 2, flexDirection: 'row' }}>

                            {/* Left button. */}
                            <Pressable style={ControllerStyle.dPadLeft}
                                onPressIn={() => {
                                    this.sendOperation(Robot.WHEELS, WheelOperation.LEFT);
                                    if (ls > -3)
                                        this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                    if (rs < 3)
                                        this.setState(prevState => ({ rs: prevState.rs + 1 }));
                                }}
                            >
                                <FontAwesome name="chevron-left" size={50} color="#fff" />
                                <View style={ControllerStyle.circleOverlayVerticalLeft}>
                                    <View style={[ControllerStyle.circle, ls >= 3 && ControllerStyle.positiveCircle, ls <= -1 && ControllerStyle.negativeCircle]} />
                                    <View style={[ControllerStyle.circle, ls >= 2 && ControllerStyle.positiveCircle, ls <= -2 && ControllerStyle.negativeCircle]} />
                                    <View style={[ControllerStyle.circle, ls >= 1 && ControllerStyle.positiveCircle, ls <= -3 && ControllerStyle.negativeCircle]} />
                                </View>
                            </Pressable>

                            {/* Stop button. */}
                            <Pressable style={[
                                    ControllerStyle.stopButton,
                                    { backgroundColor: `rgb(${this.state.colorValue.r}, ${this.state.colorValue.g}, ${this.state.colorValue.b})` }, // Use the interpolated RGB values
                                ]}
                                onPressIn={() => {
                                        this.sendOperation(Robot.WHEELS, WheelOperation.STOP);
                                        console.log("STOPPING ROVER...");
                                        this.setState({ ls: 0, rs: 0, stopping: false, isPressed: true, colorValue: { r: 128, g: 51, b: 51 } });
                                        clearInterval(this.interval); // Clear any existing fade intervals
                                }}

								onPressOut={() => {
                                    this.setState({isPressed: false});
                                    this.interval = setInterval(() => {
                                        this.setState((prevState) => {
                                            const { r, g, b } = prevState.colorValue;
                                            // Calculate new color values to fade to grey (#3a3d3d or RGB(58, 61, 61))
                                            const newR = r > 58 ? r - 1 : 58;
                                            const newG = g < 61 ? g + 1 : 61;
                                            const newB = b < 61 ? b + 1 : 61;
                            
                                            // Check if we've reached grey
                                            if (newR === 58 && newG === 61 && newB === 61) {
                                                clearInterval(this.interval); // Clear interval when we reach grey
                                            }
                            
                                            return {
                                                colorValue: { r: newR, g: newG, b: newB },
                                            };
                                        });
                                    }, 25); // Change the interval timing to adjust the fade speed
                                     }}
                                >
                                    <FontAwesome name="stop" size={35} color="#fff" />
                            </Pressable>

                            {/* Right button. */}
                            <Pressable style={ControllerStyle.dPadRight}
                                onPressIn={() => {
                                    this.sendOperation(Robot.WHEELS, WheelOperation.RIGHT);
                                    if (ls < 3)
                                        this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                    if (rs > -3)
                                        this.setState(prevState => ({ rs: prevState.rs - 1 }));
                                }}
                            >
                                <FontAwesome name="chevron-right" size={50} color="#fff" />
                                <View style={ControllerStyle.circleOverlayVerticalRight}>
                                    <View style={[ControllerStyle.circle, rs >= 3 && ControllerStyle.positiveCircle, rs <= -1 && ControllerStyle.negativeCircle]} />
                                    <View style={[ControllerStyle.circle, rs >= 2 && ControllerStyle.positiveCircle, rs <= -2 && ControllerStyle.negativeCircle]} />
                                    <View style={[ControllerStyle.circle, rs >= 1 && ControllerStyle.positiveCircle, rs <= -3 && ControllerStyle.negativeCircle]} />
                                </View>
                            </Pressable>
                        </View>




                        {/* Backwards button. */}
                        <Pressable style={ControllerStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.BACKWARD);
                                if (ls > -3)
                                    this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                if (rs > -3)
                                    this.setState(prevState => ({ rs: prevState.rs - 1 }));
                            }}
                        >
                            <FontAwesome name="chevron-down" size={50} color="#fff" />
                        </Pressable>
                    </View>

                    {/* Multi rover info viewer */}
                    <View style={ControllerStyle.roverSectionContainer}>
                        <View style={{flexDirection: 'row'}}>
                            <View style={ControllerStyle.roverSection}>
                                <Text style={ControllerStyle.roverNameText}>Rover 1</Text>
                                <Text style={ControllerStyle.roverInfoText}>Rover Type: Sidekick</Text>
                                <Text style={ControllerStyle.roverInfoText}>IP: {this.state.ip}</Text>
                                <Text style={ControllerStyle.roverInfoText}>Number of pavers: 3</Text>
                            </View>

                            <View style={ControllerStyle.roverSection}>
                            <Text style={ControllerStyle.roverNameText}>Rover 2</Text>
                                <Text style={ControllerStyle.roverInfoText}>Rover Type: {this.state.armInfo.name}</Text>
                                <Text style={ControllerStyle.roverInfoText}>IP: {this.state.armInfo.ip_address}</Text>
                                {/* <Text style={ControllerStyle.roverInfoText}>Currently carrying paver?: false</Text> */}
                            </View>
                        </View>

                            <View style={{flexDirection: 'row'}}>
                            
                                <View style={ControllerStyle.roverSection}>
                                    <Text style={ControllerStyle.roverNameText}>Rover 3</Text>
                                    <Text style={ControllerStyle.roverInfoText}>Not Connected</Text>
                                </View>

                                <View style={ControllerStyle.roverSection}>
                                    <Text style={ControllerStyle.roverNameText}>Rover 4</Text>
                                        <Text style={ControllerStyle.roverInfoText}>Not Connected{'\n\n'}</Text>
                                </View>    
                            </View>
                    </View>


                    {/* Drum/shoulder buttons. */}
                     {/* <View style={ControllerStyle.drumFunctionContainer}>
                        <View style={{ flex: 8, width: '100%' }}>

                            {/* Top row of controls. 
                            <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>

                                <View style={{ flexDirection: 'row', gap: '15px' }}>

                                    // Raise left-shoulder button. 
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.RAISE);
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="arrow-circle-up" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.LOWER);
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="arrow-circle-down" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                </View>

                                <View style={{ flexDirection: 'row', gap: '15px' }}>

                                    
                                    <View style={{}}>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.RAISE);
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="arrow-circle-up" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.LOWER);
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="arrow-circle-down" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                </View>

                            </View>

                            
                            <Image
                                style={ControllerStyle.image}
                                source={require('../../../assets/rassor.png')}
                            />

                            
                            <View style={{ flexDirection: "row", justifyContent: 'space-between' }}>

                                <View style={{ flexDirection: "row", gap: "15px" }}>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(
                                                    Robot.FRONTDRUM,
                                                    DrumOperation.DUMP
                                                );
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="rotate-left" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(
                                                    Robot.FRONTDRUM,
                                                    DrumOperation.DIG
                                                );
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="rotate-right" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                </View>

                                <View style={{ flexDirection: 'row', gap: "15px" }}>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(
                                                    Robot.BACKDRUM,
                                                    DrumOperation.DIG
                                                );
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKDRUM, DrumOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="rotate-left" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                    
                                    <View>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(
                                                    Robot.BACKDRUM,
                                                    DrumOperation.DUMP
                                                );
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKDRUM, DrumOperation.STOP);
                                            }}
                                        >
                                            <FontAwesome name="rotate-right" size={50} color="#fff" />
                                        </Pressable>
                                    </View>

                                </View>

                            </View>

                        </View>
                    </View> */}

                </FadeInView>

            </View>
        );
    }
}
