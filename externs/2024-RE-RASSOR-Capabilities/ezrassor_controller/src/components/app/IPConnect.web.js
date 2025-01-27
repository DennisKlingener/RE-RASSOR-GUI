import React from 'react';
import {
    Text,
    View,
    Image,
    StatusBar,
    TextInput,
    Pressable
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import ControllerStyle from '../../../src/styles/controller';
import * as Font from 'expo-font';
import logo from '../../../assets/fsiLogo.png';
import arrowright from '../../../assets/arrowri.png';
import { KeyboardAwareScrollView } from 'react-native-keyboard-aware-scroll-view';
import { isIpReachable } from '../../functionality/connection';
import { normalize } from '../../functionality/display';
import LoadingDots from './LoadingDots';

const DEFAULT_IP = '192.168.1.2:8080';

/**
 * React component for the connect-to-ip screen.
 */
export default class IPConnect extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            isLoading: true,
            ip: null
        };

        this.animation = React.createRef(null);
    }

    async componentDidMount() {
        await Font.loadAsync({ NASA: require('../../../assets/nasa.ttf') });

        this.animation = React.createRef(null);
        this.animation.current?.reset();

        this.setState({ isLoading: false });
        this.getIpFromStorage();

        this._unsubscribe = this.props.navigation.addListener('focus', () => {
            this.getIpFromStorage();
            this.animation.current?.reset();
        });

        this._blur = this.props.navigation.addListener('blur', () => {
            this.animation.current?.reset();
        });
    }

    async componentWillUnmount() {
        this.animation = React.createRef(null);
        this.animation.current?.reset();

        this._unsubscribe();
        this._blur();
    }

    /**
     * Set `this.state.ip` to the specified IP + port.
     * 
     * @param {string} ip IP + port.
     */
    changeIP(ip) {
        this.setState({ ip });
    }

    /**
     * Set `myIp` in storage to `this.state.ip`.
     */
    async setIpInStorage() {
        try {
            await AsyncStorage.setItem('myIp', this.state.ip);
        } catch (error) {
            // Error saving data. Log error, but do nothing otherwise.
            console.log(error);
        }
    }

    /**
     * Load `myIp` from storage into `this.state.ip`.
     */
    async getIpFromStorage() {
        try {
            const ip = await AsyncStorage.getItem('myIp');

            if (this.state.ip == null) {
                this.setState({
                    ip: (ip == null) ? DEFAULT_IP : ip
                });
            }
        } catch (error) {
            // Error retrieving data. Do nothing.
        }
    }

    /**
     * Check if we can connect to the ip address at `this.state.ip`. Then redirect to either
     * a "can connect" or "cannot connect" screen.
     */
    async redirectBasedOnReachability() {
        const timeoutTime = 5000;

        console.log('In redirectBasedOnReachability with IP: ' + this.state.ip);

        if (await isIpReachable(this.state.ip, timeoutTime)) {
            this.setIpInStorage();
            this.props.navigation.navigate('Connection Status Screen', { screen: 'roverConnected' });
        } else {
            this.props.navigation.navigate('Connection Status Screen', { screen: 'roverDisconnected' });
        }
    }

    render() {
        console.log("WEB")
        // I.e., don't do full render if font is still loading...
        if (this.state.isLoading) {
            return <View style={{ flex: 1, backgroundColor: '#5D6061' }} />;
        }

        return (
            <KeyboardAwareScrollView contentContainerStyle={[ControllerStyle.keyboardAwareScrollView]}>
                <View style={[ControllerStyle.screenLayout, { overflow: 'hidden' }]}>

                    <StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

                    {/* Title container. */}
                    <View style={[ControllerStyle.title, { marginTop: '15%' }]}>
                        <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.titleText, { fontSize: '3.5vw', justifyContent: 'center' }]}>
                            RE-RASSOR Connect
                        </Text>
                    </View>

                    {/* Body container. */}
                    <View style={{ flexDirection: 'column', justifyContent: 'space-between', marginTop: '2%', width: '70%' }}>

                        {/* Inner-body container. */}
                        <View style={[ControllerStyle.containerTwo, { backgroundColor: '#4a4d4e', width: '70%', height: '85%', padding: '5%' }]}>

                            {/* Message to user. */}
                            <Text
                                adjustsFontSizeToFit={true}
                                //numberOfLines={1}
                                style={{
                                    display: 'flex',
                                    alignSelf: 'center',
                                    textAlign: 'center',
                                    fontFamily: 'NASA',
                                    margin: 10,
                                    fontSize: '2vw',
                                    color: '#fff',
                                    //paddingTop: 10, // Add padding to the top
                                    //paddingBottom: 10, // Add padding to the bottom
                                }}
                            >
                                Please enter the IP address of the RE-RASSOR cart:
                            </Text>

                            {/* Loading dots animation. */}
                            <View
                                style={[{ justifyContent: 'center', alignItems: 'center', position: 'absolute' }]}>
                                <LoadingDots ref={this.animation} autoPlay={false} />
                            </View>

                            {/* Text input for IP + port. */}
                            <TextInput
                                //fontSize={normalize(32, 1.8)}
                                style={[ControllerStyle.ipInputBox, { fontSize: '2vw' }]}
                                onChangeText={(text) => this.changeIP(text)}
                                value={this.state.ip}
                                marginVertical={8}
                                disableFullscreenUI={true}
                                selectionColor={'white'}
                            />

                            {/* Connect button. */}
                            <Pressable
                                activeOpacity={0.95}
                                backgroundColor="#FFFFFF"
                                style={[ControllerStyle.connectButton, { marginTop: '3%', padding: '2%', height: 'auto' }]}
                                onPress={() => {
                                    this.animation.current?.play();
                                    this.redirectBasedOnReachability();
                                }}
                            >
                                <Text style={[ControllerStyle.connectButtonText, { fontSize: '1.5vw' }]}>
                                    CONNECT
                                </Text>
                                <Image source={arrowright} style={ControllerStyle.arrowRight} />
                            </Pressable>

                        </View>


                        {/* Help button. */}

                    </View>
                    <View style={[{ flexDirection: 'row', justifyContent: 'space-between', width: '100%', height: '40%', marginTop: '2%' }]}>

                        {/* FSI logo. */}
                        <Image
                            source={logo}
                            style={[ControllerStyle.fsiLogo, { alignSelf: 'auto', height: '21%' }]}
                        />

                        <Pressable
                            activeOpacity={0.95}
                            style={[ControllerStyle.buttonContainer, { height: '16%', alignSelf: 'auto', marginRight: '5%', padding: '1.5%' }]}
                            onPress={() => {
                                this.props.navigation.navigate('Connection Help Screen');
                            }}
                        >
                            <Text style={[ControllerStyle.buttonText, { fontSize: '2vw' }]}>
                                Help
                            </Text>
                        </Pressable>

                    </View>

                </View>
            </KeyboardAwareScrollView>
        );
    }
}
