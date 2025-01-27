import { NativeStackView } from '@react-navigation/native-stack';
import { StyleSheet } from 'react-native';
import { normalize, width } from '../functionality/display';
import { Platform } from 'react-native';

const splashScreen = {
    splashFsiLogo: {
        height: '55%',
        width: '55%',
        resizeMode: 'contain',
        backgroundColor: '#2E3030',
        marginTop: 70

    },

    splashScreen: {
        display: 'flex',
        flex: 1,
        backgroundColor: '#2E3030',
        justifyContent: 'center',
        alignItems: 'center',
        alignContent: 'center'
    },
}


const ControllerStyle = StyleSheet.create({

    keyboardAwareScrollView: {
        flexGrow: 1,
        ...Platform.select({
            ios: {
                flex: 1
            }
        })
    },

    fsiLogo: {
        height: '20%',
        width: '16%',
        alignSelf: 'flex-end',
        position: 'relative',
        right: 20,
        bottom: 10,
        overflow: 'visible',
        resizeMode: 'contain',
        backgroundColor: '#2E3030',
        ...Platform.select({
            ios: {
                // bottom: 10 // as same as height
            },
            android: {
                // bottom: 10
            }
        })
    },

    //splash screen
    ...splashScreen,


    splashLoading: {
        width: '100%',
        height: '100%',
        alignSelf: 'center'
    },

    buttonContainer: {
        backgroundColor: '#FFFFFF',
        fontFamily: 'NASA',
        borderRadius: 8,
        height: '10%',
        width: '12%',
        textAlign: 'center',
        alignSelf: 'flex-end',
        // position: 'absolute',
        left: 10,
        bottom: 10,
        ...Platform.select({
            ios: { // as same as height
                // marginRight: 20
            },
            android: {
            },
            default: {
                width: 'fit-content',
                height: 'fit-content',
                padding: '10px',
                justifyContent: 'center'
            }
        })
    },

    buttonModalContainer: {
        backgroundColor: '#FFFFFF',
        fontFamily: 'NASA',
        borderRadius: 8,
        height: '10%',
        width: '12%',
        textAlign: 'center',
        alignSelf: 'flex-end',
        position: "absolute",
        right: 10,
        bottom: 20,
    },

    buttonText: {
        fontFamily: 'NASA',
        textAlign: 'center',
        fontSize: 50,
        ...Platform.select({
            ios: {
                // marginTop: 5, // as same as height
            }
        })
    },



    screenLayout: {
        width: '100%',
        height: '100%',
        backgroundColor: '#2E3030',
        justifyContent: 'center', //Centered vertically
        alignItems: 'center',
        flexDirection: 'column',
        overflow: 'visible'
    },

    title: {
        overflow: 'hidden',
        justifyContent: 'center',
        marginHorizontal: 10,
        color: '#fff',
        textAlign: 'center',
        backgroundColor: '#4a4d4e',
        fontFamily: 'NASA',
        flexWrap: 'nowrap',
        height: '20%',
        width: '60%',
        borderRadius: 15,

        ...Platform.select({
            ios: {
                // lineHeight: -50, // as same as height
                // marginTop: 30,
            },
            android: {},
            default: {
            }
        })
    },

    titleText: {
        display: 'flex',
        padding: 0,
        textAlign: 'center',
        fontFamily: 'NASA',
        fontSize: 70,
        color: '#fff',
        // textAlignVertical: 'center',
        ...Platform.select({
            ios: {
                margin: 15
            },
            android: {
            },
            default: {
                fontSize: normalize(40),
                justifyContent: 'center',
            }
        })
    },


    containerTwo: {
        justifyContent: 'center',
        alignItems: 'center',
        padding: 10,
        borderRadius: 15,
        alignSelf: 'center',
        height: '80%',
        flexDirection: 'column',
        ...Platform.select({
            ios: {
            },
            android: {
            },
            default: {
                overflow: 'visible',
                height: '100%'
            }
        })
    },


    connectButton: {
        flexDirection: 'row',
        backgroundColor: '#FFFFFF',
        display: 'flex',
        overflow: 'hidden',
        justifyContent: 'space-evenly',
        fontFamily: 'NASA',
        borderRadius: 8,
        textAlign: 'center',
        textAlignVertical: 'center',
        // height: '20%',
        height: (2 * normalize(45, 1.5)),
        lineHeight: (2 * normalize(45, 1.8)),
        width: '30%',
        marginTop: 5,
        marginBottom: 15
    },

    statusButton: {
        backgroundColor: '#3F4142',
        display: 'flex',
        overflow: 'hidden',
        justifyContent: 'center',
        fontFamily: 'NASA',
        borderRadius: 8,
        justifyContent: 'center',
        textAlign: 'center',
        textAlignVertical: 'center',
        height: '20%',
        width: '30%',
        marginTop: 5,
        marginBottom: 15,
        ...Platform.select({
            ios: {
            },
            android: {
            },
            default: {
                height: '30%',
                width: 'fit-content',
                padding: '10px',
                justifyContent: 'center'
            }
        })
    },

    arrowRight: {
        resizeMode: 'contain',
        height: '80%',
        width: '10%',
        alignSelf: 'center',

    },

    connectButtonText: {
        fontSize: (normalize(45, 1.2)),
        alignSelf: 'center',
        fontFamily: 'NASA',
        justifyContent: 'center',
        textAlign: 'center',
        textAlignVertical: 'center',

        ...Platform.select({
            android: {
                marginBottom: 10
            },
        })
    },


    statusButtonText: {
        justifyContent: 'center',
        textAlign: 'center',
        fontFamily: 'NASA',
        color: 'white',
        textAlignVertical: 'center',
        fontSize: 50,
        ...Platform.select({
            default: {
                fontSize: '2em',
                overflow: 'visible'
            }
        })

    },

    buttonLayoutContainer: {
        flex: 8,
        flexDirection: 'row',
        marginVertical: 10,
        ...Platform.select({
            web: {
                width: '90%',
                alignContent: 'center'
            }
        })
    },

    container: {
        flex: 1,
        backgroundColor: '#5D6061',
        alignItems: 'center',
        justifyContent: 'center',
    },
    dPadLeft: {
        flex: 1,
        backgroundColor: '#3a3d3d',
        borderRadius: 10,
        marginHorizontal: 10,
        elevation: 5,
        justifyContent: 'center',
        alignItems: 'center'
    },

    dPadRight: {
        flex: 1,
        backgroundColor: '#3a3d3d',
        borderRadius: 10,
        marginHorizontal: 10,
        elevation: 5,
        justifyContent: 'center',
        alignItems: 'center'
    },

    /********************************/
    /**********CONTROLLER************/
    /********************************/

    icons: {
        alignItems: 'center',
        alignSelf: 'center',
        justifyContent: 'center',
        ...Platform.select({
            web: {
                padding: 4,
                height: '50%',
                flex: 1,
            }
        })
    },

    evenlySpaceIcons: {
        flexDirection: 'row',
        gap: 30,
        ...Platform.select({
            web: {
                display: 'flex',
                gap: 15
            }
        })
    },

    stopButton: {
        flex: 1,
        backgroundColor: '#3a3d3d', // Same background color
        borderRadius: 10,
        elevation: 5,
        justifyContent: 'center',
        alignItems: 'center'
    },

    circle: {
        width: 5,
        height: 5,
        borderRadius: 5,
        backgroundColor: '#5b5b5b', // Default circle color
        margin: 5
    },

    positiveCircle: {
        backgroundColor: '#490' // Active circle color
    },

    negativeCircle: {
        backgroundColor: '#922'
    },

    circleOverlayDown: {
        position: 'absolute',
        top: 0,
        flexDirection: 'row',
        justifyContent: 'center',
    },
    circleOverlayUp: {
        position: 'absolute',
        bottom: 0,
        flexDirection: 'row',
        justifyContent: 'center',
    },
    circleOverlayVerticalRight: {
        position: 'absolute',
        flexDirection: 'column',
        justifyContent: 'center',
        left: 0,
    },
    circleOverlayVerticalLeft: {
        position: 'absolute',
        flexDirection: 'column',
        justifyContent: 'center',
        right: 0,
    },
    drumFunctionContainer: {
        flex: 6,
        justifyContent: 'center',
        marginHorizontal: 10,
        padding: 10,
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        ...Platform.select({
            web: {
                flex: 8,
                alignItems: 'center'
            }
        })
    },

    roverSectionContainer: {
        flex: 6,
        flexDirection: 'column',
        marginHorizontal: 10,
        padding: 10,
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        justifyContent: 'center',
    },

    roverSection: {
        flex: 2,
        backgroundColor:'#3a3d3d',
        borderRadius: 10,
        elevation: 3,
        padding: 25,
        margin: 10,
    },

    roverNameText: {
        fontFamily: 'NASA',
        fontSize: '2.75vw',
        lineHeight: '200%',
        color: '#fff',
        textAlign: 'center',
        textAlignVertical: 'center',
    },

    roverInfoText: {
        fontFamily: 'Comic Sans',
        lineHeight:'200%',
        fontSize: '1.25vw',
        color: '#fff',
        textAlign: 'center',
        textAlignVertical: 'center',
    },

    wheelFunctionContainer: {
        flex: 3,
        marginLeft: 10,
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
    },

    headerContainer: {
        flex: 1,
        flexDirection: 'row',
        marginTop: 10,
        marginHorizontal: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        borderRadius: 10,
        padding: 10,
        justifyContent: 'center',
        alignItems: 'center',
        ...Platform.select({
            web: {
                width: '95%',
                justifyContent: 'space-between'
            }
        })
    },

    image: {
        flex: 1,
        resizeMode: 'contain',
        paddingVertical: 20,
        width: '100%'
    },

    textLarge: {
        fontFamily: 'NASA',
        fontSize: 30,
        color: '#fff',
        ...Platform.select({
            web: {
                fontSize: 50,
                alignSelf: 'center',
                textAlign: 'center',
            }
        })
    },

    textMedium: {
        fontFamily: 'NASA',
        flex: 4,
        fontSize: 25,
        color: '#fff',
        textAlign: 'center',
        textAlignVertical: 'center',
        ...Platform.select({
            web: {
                flex: 'unset',
                fontSize: "3vw"
            }
        })
    },

    textHeader: {
        fontFamily: 'NASA',
        fontSize: 22,
        color: '#fff'
    },


    textSmall: {
        position: "absolute",
        top: 5,
        fontFamily: 'NASA',
        fontSize: 22,
        color: '#fff',
        ...Platform.select({
            web: {
                top: 'unset',
                position: 'unset',
                textAlign: 'center'
            }
        })
    },

    textTiny: {
        fontFamily: 'NASA',
        fontSize: 12,
        color: '#fff'
    },

    columnText: {
        color: '#fff',
        textAlign: 'center',
        marginVertical: '1%',
    },

    columnHyperlink: {
        color: 'cornflowerblue',
        textAlign: 'center'
    },

    columnHeader: {
        flex: 20,
        marginHorizontal: 15,
        justifyContent: 'center',
        alignItems: 'center'
    },

    modalViewContainer: {
        borderRadius: 25,
        backgroundColor: '#5D6061',
    },

    modalButton: {
        flex: 1,
        backgroundColor: '#767676',
        borderRadius: 100,
        marginHorizontal: 15,
        justifyContent: 'center',
        alignItems: 'center',
        height: 100,
        elevation: 5,
    },

    upAndDownDPad: {
        flex: 1,
        backgroundColor: '#3a3d3d',
        borderRadius: 10,
        margin: 10,
        elevation: 5,
        justifyContent: 'center',
        alignItems: 'center'
    },

    ipInputBox: {
        display: 'flex',
        height: '30%',
        width: '70%',
        backgroundColor: '#BFBFBF',
        borderColor: 'gray',
        borderWidth: 1,
        color: 'black',
        borderRadius: 8,
        textAlign: 'center',
        fontFamily: 'NASA',
        ...Platform.select({
            ios: {
            },
            android: {
                paddingBottom: 5, paddingTop: 0,
                textAlignVertical: 'center',
                alignItems: 'center'
            }
        })
    },

    ipInputBoxMedium: {
        backgroundColor: '#2e3030',
        borderColor: 'gray',
        color: '#fff',
        fontSize: 15,
        textAlign: 'center',
        fontFamily: 'NASA',

    },

    rightSideRow: {
        flexDirection: 'row',
        position: 'absolute',
        right: 0
    },

    /***************************/
    /*********PAVER ARM*********/
    /***************************/

    ArmContainer: {
        flex: 1,
        backgroundColor: '#5D6061'
    },

    PaverArmBaseButton: {
        flex: 3,
        marginLeft: 15,
        borderRadius: 10,
        backgroundColor: '#2e3030',
    },

    PaverArmBackground: {
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        marginBottom: 10,
        marginLeft: 10,
        marginRight: 10,
        width: 180,
    },

    buttonTextCenter: {
        fontFamily: 'NASA',
        fontSize: 22,
        color: '#fff',
        textAlign: 'center',
        paddingTop: 7
    },

    buttonBackground: {
        width: '40%',
        height: '70%',
        backgroundColor: '#3a3d3d',
        justifyContent: 'center',
        alignItems: 'center',
        borderRadius: 16,
    },

    mainButtonTextVertical: {
        fontFamily: 'NASA',
        fontSize: 15,
        color: '#fff',
        textAlign: 'center',
        bottom: 5,
        paddingTop: 5
    },

    buttonImage: {
        fontFamily: 'NASA',
        color: '#fff',
        textAlign: 'center',
    },
});

export default ControllerStyle;
