import { Pressable, StyleSheet, Text, View, Alert, TouchableHighlight } from 'react-native';
import * as React from 'react';
import Modal from 'react-native-modal';
import EZRASSOR from '../../../src/api/ezrassor-service';
import EZARM from '../../../src/api/ezrassor-service-paver-arm'
import { Robot, ShoulderOperation, DrumOperation, WheelOperation, Operation, ServoOperation } from '../../../src/enumerations/robot-commands';
import { isIpReachable } from '../../functionality/connection';
import { FontAwesome, FontAwesome5, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font from 'expo-font';
import ControllerStyle from '../../../src/styles/controller';
import VideoStyle from '../../../src/styles/video';
import { useRoute } from '@react-navigation/native';

const VIDEO_EXT_STANDARD = '/video_feed';
const GREEN_ZONE_CHECK = '/get_target_status';
const DETECTION_POLLING_EXTENSION = '/is_detection';
const DETECTION_POLLING_INTERVAL = 5000;
const GREEN_ZONE_POLLING_INTERVAL = 250;

const CONNECTION_POLLING_INTERVAL = 2000;
const CONNECTION_POLLING_TIMEOUT = 4000;
const CONNECTION_POLLING_VERBOSE = false;

export default class VideoController extends React.Component {

    /**
     * Constructor for VideoController componenet
     * ip variable is part of props passed to this component from ControllerScreen component 
     */
    constructor(props) {
        super(props);

        this.pollForDetection = this.pollForDetection.bind(this);

        this.state = {
            autonomyModalVisible: false,
            infoModalVisible: false,
            devModalVisible: false,
            ls: 0,
            rs: 0,
            ipModal: false,
            xyModal: false,
            isLoading: true,
            isDetecting: false,
            control: 0,
            xy: '0,0',
            ip: '',
            videoIp: '',
            greenZoneIP: '',
            targetMarkerString:'',
            pollingRoute: '',
        };

        this.EZARM = new EZARM(this.state.ip);
        this.EZRASSOR = new EZRASSOR(this.state.ip);

    }

    set detectionReport(report) {

    }

    /**
     * Load assets required for this component and check connection to IP address
     */
    async componentDidMount() {
        await Font.loadAsync({ NASA: require('../../../assets/nasa.ttf') });
        await this.getIpFromStorage();
        this.state.ip = this.props.route.params.currentIp; // setting ip address
        // important line: sets the IP address for current EZRASSOR instance
        this.EZRASSOR.host = this.state.ip;

        // Set IP address to retreive video feed from
        this.state.videoIp = 'http://' + this.state.ip + VIDEO_EXT_STANDARD;
        this.state.greenZoneIP = 'http://' + this.state.ip + GREEN_ZONE_CHECK;

        // Start polling for target marker status
        this.greenZonePolling = setInterval(async () => {
            try {
                const response = await fetch(this.state.greenZoneIP);
                const targetMarkerStatus = await response.text();
                console.log('Target Marker Status:', targetMarkerStatus);

                // Optionally, update state with the received sidekick info
                this.setState({ targetMarkerString: targetMarkerStatus });
            } catch (error) {
                console.error('Error fetching target marker status:', error);
            }
        }, GREEN_ZONE_POLLING_INTERVAL);

        // Set IP address to poll for detection report from 
        this.state.pollingRoute = this.state.ip + DETECTION_POLLING_EXTENSION;

        // this.state.videoIp = this.props.route.params.currentIp + this.videoExtension;
        console.log('Set videoIp to: ' + this.state.videoIp);
        console.log('Set greenZoneIP to: ' + this.state.greenZoneIP);
        console.log('Set polling route to: ' + this.state.pollingRoute);

        this.setState({ isLoading: false });

        this._unsubscribe = this.props.navigation.addListener('focus', async () => {
            this.getIpFromStorage();
        });

        // Set up the connection polling logic.
        // let pollCount = 0;
        // this.connectionPoller = setInterval(async () => {
        //     if (await isIpReachable(this.state.ip, CONNECTION_POLLING_TIMEOUT)) {
        //         if (CONNECTION_POLLING_VERBOSE) {
        //             console.log(`Connection poll attempt ${++pollCount} succeeded...`);
        //         }
        //     } else {
        //         console.log(`Dropped connection from ${this.state.ip}... redirecting to connection screen.`);
        //         this.props.navigation.replace('Connection Status Screen', { screen: 'roverDisconnected' });
        //     }
        // }, CONNECTION_POLLING_INTERVAL);

        const pollInterval = (CONNECTION_POLLING_INTERVAL / 1000.0).toFixed(2);
        const pollTimeout = (CONNECTION_POLLING_TIMEOUT / 1000.0).toFixed(2);
        console.log(`Polling every ${pollInterval}s with a ${pollTimeout}s timeout.`);


        const { passrs, passls } = this.props.route.params
        this.state.ls = passls
        this.state.rs = passrs

    }

    componentWillUnmount() {
        // Stop polling intervals when component is unmounted
        clearInterval(this.greenZonePolling);
        clearInterval(this.connectionPoller);
    }

    setAutonomyModalVisible(invisible) {
        this.setState({ autonomyModalVisible: invisible });
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
        if (passrs !== null && passrs !== null) {
            rs = passrs
            ls = passls
        }
    }

    

    /**
     * Retrieves IP addreess saved previously in AsyncStorage 
     */
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

    /**
     * Set `this.state.ip` and `this.EZRASSOR.host` to the specified IP + port.
     * 
     * @param {string} ip IP + port
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

    /**
     * Update animation frame before processing click so that opacity can change on click.
     */
    // TODO: verify this sends coordiantes of paver to this function and handle it on the arm service
    sendPickUpOperation(part, operation) {
        requestAnimationFrame(() => {
            this.EZARM.executeRobotCommand(part, operation)
        });
    }

    /**
     * Stop and start the paver detection polling service
     * Call function to change video extension, 
     * causing the component to re-render with the new soucre for video feed
     */
    togglePaverDetection() {
        if (this.state.isDetecting) {
            this.state.isDetecting = false;
            this.stopDetectionPolling();
        }
        else {
            this.state.isDetecting = true;
            this.startDetectionPolling();
        }
    }

    /**
      * Poll Controller Server Detection API to confirm a paver is still in view to pick
      */
    async confirmDetection(x, y, z) {
        const response = await fetch('http://' + this.state.pollingRoute)
            .then((response) => response.json())
            .then((json) => console.log(json));

        pickUpOperation = json.x.toString() + json.y.toString() + json.z.toString();

        if (json.hasOwnProperty('x')) {
            // TODO: check that this is the same pave the user wanted to pick up
            // Figure out how to verify that it is the same paver the user said to pick up
            //this.sendPickUpOperation(Arm.AUTONOMY, pickUpOperation);
            console.log("paver to pick up confirmed")
        }
        else {
            console.log("paver to pick up no longer in view")
        }
    }

    /**
     * Handles request to pick up paver
     * 
     */
    handlePickUpRequest(x, y, z) {
        // TODO: add this to send command to arm for picking up paver
        // Need to finish aruco postion calculation first
        // this.EZRASSOR.sendOperation(Robot.WHEELS, WheelOperation.STOP);
        // this.confirmDetection(x, y, z);
    }

    /**
     * Creates and shows dialog box with paver detection info
     */
    createPaverNotification(x, y, z) {
        Alert.alert('Paver Detected at ' + x + ', ' + y + ', ' + z, 'choose arm action', [
            {
                text: 'Ignore',
                onPress: () => this.startDetectionPolling(), // Restart detection polling if user chooses to ingore current paver
                style: 'cancel',
            },
            { text: 'Pick Up (Not implimented yet)', onPress: () => this.handlePickUpRequest(x, y, z) }, // Prepare to pick up paver
        ]);
    }

    /**
     * Starts polling to Controller Server API for detection of Pavers.
     * Sets interval polling function will be called at.
     */
    startDetectionPolling() {
        console.log("Starting Detection Polling");

        this.detectionTask = setInterval(this.pollForDetection, DETECTION_POLLING_INTERVAL);
    }

    /**
     * Stops polling to Controller Server API for detection of Pavers.
     */
    stopDetectionPolling() {
        console.log("Stopping Detection Polling");
        clearInterval(this.detectionTask);
    }

    /**
      * Poll Controller Server API to check for detection of Pavers.
      * Makes API requests at specified interval
      */
    async pollForDetection() {
        console.log("Polling for detection at: " + this.state.pollingRoute);

        let response = await fetch('http://' + this.state.pollingRoute);
        let data = await response.json();

        console.log(data);

        // JSON response contains detection report, stop detection polling and create notification
        if (data.hasOwnProperty('x')) {
            this.stopDetectionPolling();
            this.createPaverNotification(data.x, data.y, data.z);
        }
        else {
            console.log("Detection report did not have a coordinate");
        }
    }

    // Stop detection polling and navigate to standard controller screen
    switchToControllerScreen() {
        if (this.state.isDetecting) {
            this.stopDetectionPolling();
            this.state.isDetecting = false;
        }
        this.props.navigation.replace("Controller Screen", {
            passls: this.state.ls,
            passrs: this.state.rs
        });
    }

    render() {
        const { rs, ls } = this.state

        console.log("Video Web")
        console.log('Video Feed IP Addr: ' + this.state.videoIp);

        // Use WebView to render the webpage hosting the live video streaming from EZRASSOR
        return (
            <View style={styles.container}>

                {/* Video Controller page Info popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.infoModalVisible}
                    onSwipeComplete={() => this.setInfoModalVisible(false)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() => this.setInfoModalVisible(false)}
                >

                    <TouchableHighlight style={{ justifyContent: 'center' }}>
                        <View>
                            <View style={{ flexDirection: 'row', justifyContent: 'center', marginBottom: 25 }}>
                                <Text style={ControllerStyle.textLarge}>Controller Help</Text>
                            </View>

                            <View style={{ flexDirection: 'row', justifyContent: 'center', marginHorizontal: "0.5%" }}>
                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome style={{ textAlign: 'center' }} name="wifi" size={30} color='#fff' />
                                    <Text style={ControllerStyle.textSmall}>
                                        Connects to the server with input string: {'\n'}
                                        IP : PORT
                                    </Text>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <MaterialCommunityIcons style={{ textAlign: 'center' }} name="cube-scan" size={32} color="#fff" />
                                    <Text style={ControllerStyle.textSmall}>
                                        Start/Stop Paver Detection notification
                                    </Text>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome style={{ textAlign: 'center' }} name="stop-circle-o" size={30} color="#fff" />
                                    <Text style={ControllerStyle.textSmall}>
                                        Emergency stop for manual movements
                                    </Text>
                                </View>

                                {/* Vertical separating bar.
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <MaterialCommunityIcons style={{ textAlign: 'center' }} name="robot-industrial" size={32} color="#fff" />
                                    <Text style={ControllerStyle.textSmall}>
                                        Navigate to Paver Arm Controller
                                    </Text>
                                </View>*/}

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome5 style={{ textAlign: 'center' }} name="video-slash" size={32} color="#fff" />
                                    <Text style={ControllerStyle.textSmall}>
                                        Stop Live Stream
                                    </Text>
                                </View>
                            </View>
                        </View>
                    </TouchableHighlight>

                </Modal>

                <View style={styles.webViewContainer}>

                    <img
                        src={this.state.videoIp}
                        style={styles.webView}
                    />

                    {/*Nav Bar*/}
                    <View style={styles.overlayedMenu}>
                        {/* Controller screen top row controls. */}
                        <View style={VideoStyle.headerContainer}>
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

                            {/* Title. */}
                            <Text style={ControllerStyle.textMedium}>EZ-RASSOR Controller</Text>
 
                            {/* Toggle paver detection button. */}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.togglePaverDetection()}
                            >
                                <MaterialCommunityIcons
                                    name="cube-scan"
                                    size={50}
                                    color="#fff"
                                />
                            </Pressable>

                            {/* Back to Controller Screen button.*/}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.switchToControllerScreen()}
                            >
                                <FontAwesome5
                                    name="video-slash"
                                    size={50}
                                    color="#fff" />
                            </Pressable>
                        </View>
                    </View>

                    {/* Wheel, Shoulder, and Drum buttons */}
                    <View style={styles.buttonContainer}>

                    {/*Left button*/}

                    <Pressable style={VideoStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.LEFT);
                                if (ls > -3)
                                    this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                if (rs < 3)
                                    this.setState(prevState => ({ rs: prevState.rs + 1 }));
                            }}

                        >
                            <FontAwesome name="chevron-left" size={40} color="#fff" />

                            <View style={ControllerStyle.circleOverlayDown}>
                                <View style={[ControllerStyle.circle, ls >= 3 && ControllerStyle.positiveCircle, ls <= -1 && ControllerStyle.negativeCircle]} />
                                <View style={[ControllerStyle.circle, ls >= 2 && ControllerStyle.positiveCircle, ls <= -2 && ControllerStyle.negativeCircle]} />
                                <View style={[ControllerStyle.circle, ls >= 1 && ControllerStyle.positiveCircle, ls <= -3 && ControllerStyle.negativeCircle]} />
                            </View>

                        </Pressable>
                        <View style={{flexDirection:"column"}}>
                        {/* Forwards button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.FORWARD);
                                if (ls < 3)
                                    this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                if (rs < 3)
                                    this.setState(prevState => ({ rs: prevState.rs + 1 }));

                            }}

                        >
                            <FontAwesome name="chevron-up" size={40} color="#fff" />
                        </Pressable>

                        {/* Backwards button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.BACKWARD);
                                if (ls > -3)
                                    this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                if (rs > -3)
                                    this.setState(prevState => ({ rs: prevState.rs - 1 }));
                            }}

                        >
                            <FontAwesome name="chevron-down" size={40} color="#fff" />
                        </Pressable>
                         </View>
                        

                        {/* Right button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
                            onPressIn={() => {
                                this.sendOperation(Robot.WHEELS, WheelOperation.RIGHT);
                                if (ls < 3)
                                    this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                if (rs > -3)
                                    this.setState(prevState => ({ rs: prevState.rs - 1 }));

                            }}

                        >
                            <FontAwesome name="chevron-right" size={40} color="#fff" />
                            <View style={ControllerStyle.circleOverlayDown}>
                                <View style={[ControllerStyle.circle, rs >= 3 && ControllerStyle.positiveCircle, rs <= -1 && ControllerStyle.negativeCircle]} />
                                <View style={[ControllerStyle.circle, rs >= 2 && ControllerStyle.positiveCircle, rs <= -2 && ControllerStyle.negativeCircle]} />
                                <View style={[ControllerStyle.circle, rs >= 1 && ControllerStyle.positiveCircle, rs <= -3 && ControllerStyle.negativeCircle]} />
                            </View>
                        </Pressable>


                        {/* Drum/shoulder buttons. */}
                        <View style={VideoStyle.drumFunctionContainer}>
                            <View style={{ flex: 8 }}>

                                {/* Top row of controls. */}
                                <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>
                                    

                                    {/* Stop-rover button. */}
                                    <Pressable
                                        style={ControllerStyle.icons}
                                        onPress={() => {
                                            this.sendOperation(Robot.WHEELS, WheelOperation.STOP);
                                            this.setState({ ls: 0, rs: 0 });
                                        }}
                                    >
                                        <FontAwesome
                                            name="stop-circle-o"
                                            size={75}
                                            color="#fff"
                                        />
                                    </Pressable>
                                    {/* THIS IS FOR THE GREEN ZONE DETECTION STRING ON WEB APP*/}
                                    <Text style={styles.greenZoneText}>
                                        {this.state.targetMarkerString}
                                    </Text>
                                    <View style={{ flexDirection: 'column', marginTop: '1%'}}>
                                        {/*Right Top Buttons*/}



                                        <View style={{ flexDirection: 'row', gap: '5px', alignItems: 'center'}}>
                                            {/* Full to left Servo Button */}
                                            <View style={VideoStyle.upAndDownDPad}> 
                                                <Pressable
                                                    onPressIn={() => {
                                                        this.sendOperation(Robot.SERVO, ServoOperation.CAM_SNAP_RIGHT);
                                                    }}
                                                >
                                                    <FontAwesome name='angle-double-left' size={60} color={"#fff"}/>
                                                </Pressable>
                                            </View>

                                            {/* Slight Left Servo Button */}
                                            <View> 
                                                <Pressable style={VideoStyle.upAndDownDPad}
                                                    onPressIn={() => {
                                                        this.sendOperation(Robot.SERVO, ServoOperation.CAM_PAN_LEFT);
                                                    }}
                                                >
                                                    <FontAwesome name='angle-left' size={60} color={"#fff"}/>
                                                </Pressable>
                                            </View>

                                            {/* Center Servo Button */}
                                            <View> 
                                                <Pressable style={VideoStyle.upAndDownDPad}
                                                    onPressIn={() => {
                                                        this.sendOperation(Robot.SERVO, ServoOperation.CAM_SNAP_CENTER);
                                                    }}
                                                >
                                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" width={50} marginTop={50} fill='#fff'>
                                                        <path d="M213.1 64.8L202.7 96 128 96c-35.3 0-64 28.7-64 64l0 256c0 35.3 28.7 64 64 64l384 0c35.3 0 64-28.7 64-64l0-256c0-35.3-28.7-64-64-64l-74.7 0L426.9 64.8C420.4 45.2 402.1 32 381.4 32L258.6 32c-20.7 0-39 13.2-45.5 32.8zM448 256c0 8.8-7.2 16-16 16l-76.7 0c-6.2 0-11.3-5.1-11.3-11.3c0-3 1.2-5.9 3.3-8L371 229c-13.6-13.4-31.9-21-51-21c-19.2 0-37.7 7.6-51.3 21.3L249 249c-9.4 9.4-24.6 9.4-33.9 0s-9.4-24.6 0-33.9l19.7-19.7C257.4 172.7 288 160 320 160c31.8 0 62.4 12.6 85 35l23.7-23.7c2.1-2.1 5-3.3 8-3.3c6.2 0 11.3 5.1 11.3 11.3l0 76.7zM192 320c0-8.8 7.2-16 16-16l76.7 0c6.2 0 11.3 5.1 11.3 11.3c0 3-1.2 5.9-3.3 8L269 347c13.6 13.4 31.9 21 51 21c19.2 0 37.7-7.6 51.3-21.3L391 327c9.4-9.4 24.6-9.4 33.9 0s9.4 24.6 0 33.9l-19.7 19.7C382.6 403.3 352 416 320 416c-31.8 0-62.4-12.6-85-35l-23.7 23.7c-2.1 2.1-5 3.3-8 3.3c-6.2 0-11.3-5.1-11.3-11.3l0-76.7z"/>
                                                </svg>
                                                </Pressable>
                                            </View>

                                            {/* right Servo Button */}
                                            <View>
                                                <Pressable style={VideoStyle.upAndDownDPad}
                                                    onPressIn={() => {
                                                        this.sendOperation(Robot.SERVO, ServoOperation.CAM_PAN_RIGHT);
                                                    }}
                                                >
                                                    <FontAwesome name='angle-right' size={60} color={"#fff"}/>
                                                </Pressable>
                                            </View>

                                            {/* Full to right Servo Button */}   
                                            <View> 
                                                <Pressable style={VideoStyle.upAndDownDPad}
                                                    onPressIn={() => {
                                                        this.sendOperation(Robot.SERVO, ServoOperation.CAM_SNAP_LEFT);
                                                    }}
                                                >
                                                    <FontAwesome name='angle-double-right' size={60} color={"#fff"}/>
                                                </Pressable>
                                            </View>

                                        </View>

                                    </View>
                                </View>
                                {/* Bottom row of controls. */}
                                <View style={{ flexDirection: "row", justifyContent: 'space-between' }}>


                                </View>
                            </View>
                        </View>



        
                    </View>
                </View>
            </View>
        );
    }
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#5D6061',
        alignItems: 'center',
        justifyContent: 'center',
    },
    webViewContainer: {
        flex: 1,
        alignContent: 'center',
        width: "100%",
        height: '100%',
    },
    webView: {
        objectFit: 'contain',
        width: '100%',
        height: '100%',
        display: 'block',
        position: 'absolute', // top and left adjustments 
        top: '50%',
        left: '50%', // start at horizontal center
        transform: 'translate(-50%, -50%)', // offset the square by half its width and height to truly center it
        margin: 0,
        padding: 0,
    },
    greenZoneText: {
        position: 'absolute', // Absolute positioning for precise control
        bottom: 20, // Adjust this value to move it closer to the stop button
        left: '50%', // Horizontally center it
        transform: [{ translateX: '-50%' }], // Keep it centered
        backgroundColor: 'white',
        borderColor: 'black',
        borderWidth: 1,
        borderRadius: 5,
        padding: 10,
        zIndex: 20,
        textAlign: 'center',
        fontWeight: 'bold',
        fontSize: 16,
        maxWidth: 300,
        flexWrap: 'wrap',
    },
    buttonContainer: {
        position: 'absolute',
        bottom: 16,
        flexDirection: 'row',
        justifyContent: 'space-evenly',
        alignContent: 'center',
        zIndex: 10,
        flex: 1,
        elevation: 3,
        opacity: 0.8,
        width: '100%',
        paddingHorizontal: 70,
        shadowColor: 'transparent',
    },
    buttonText: {
        color: 'white',
        fontWeight: 'bold',
    },
    overlayedMenu: {
        flex: 1,
        position: 'absolute',
        top: 5,
        flexDirection: 'row',
        width: '100%',
        height: '10%',
        justifyContent: 'center',
        alignContent: 'center',
        zIndex: 10,
    },
});
