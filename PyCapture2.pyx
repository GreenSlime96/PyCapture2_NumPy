from _FlyCapture2_C cimport *

cimport numpy as np

from cpython cimport PyObject, Py_INCREF, Py_DECREF
from libc.stdlib cimport malloc, free

cdef extern from "numpy/arrayobject.h":
    object PyArray_NewFromDescr(object subtype, np.dtype descr,
                                int nd, np.npy_intp* dims,
                                np.npy_intp* strides,
                                void* data, int flags, object obj)

cdef extern from "Python.h":
    void PyEval_InitThreads()

np.import_array()


cdef class IMAGE_FILE_FORMAT:
    """File formats to be used for saving images to disk.

        RAW: Raw data.
    """
    FROM_FILE_EXT = FC2_FROM_FILE_EXT
    PGM = FC2_PGM
    PPM = FC2_PPM
    BMP = FC2_BMP
    JPEG = FC2_JPEG
    JPEG2000 = FC2_JPEG2000
    TIFF = FC2_TIFF
    PNG = FC2_PNG
    RAW = FC2_RAW

cdef class PIXEL_FORMAT:
    """Pixel formats available for Format7 modes.

        UNSPECIFIED_PIXEL_FORMAT: Unspecified pixel format.
    """
    MONO8 = FC2_PIXEL_FORMAT_MONO8
    YUV8_411 = FC2_PIXEL_FORMAT_411YUV8
    YUV8_422 = FC2_PIXEL_FORMAT_422YUV8
    YUV8_444 = FC2_PIXEL_FORMAT_444YUV8
    RGB8 = FC2_PIXEL_FORMAT_RGB8
    MONO16 = FC2_PIXEL_FORMAT_MONO16
    RGB16 = FC2_PIXEL_FORMAT_RGB16
    S_MONO16 = FC2_PIXEL_FORMAT_S_MONO16
    S_RGB16 = FC2_PIXEL_FORMAT_S_RGB16
    RAW8 = FC2_PIXEL_FORMAT_RAW8
    RAW16 = FC2_PIXEL_FORMAT_RAW16
    MONO12 = FC2_PIXEL_FORMAT_MONO12
    RAW12 = FC2_PIXEL_FORMAT_RAW12
    BGR = FC2_PIXEL_FORMAT_BGR
    BGRU = FC2_PIXEL_FORMAT_BGRU
    RGB = FC2_PIXEL_FORMAT_RGB
    RGBU = FC2_PIXEL_FORMAT_RGBU
    BGR16 = FC2_PIXEL_FORMAT_BGR16
    BGRU16 = FC2_PIXEL_FORMAT_BGRU16
    YUV8_JPEG_422 = FC2_PIXEL_FORMAT_422YUV8_JPEG
    NUM_PIXEL_FORMATS = FC2_NUM_PIXEL_FORMATS
    UNSPECIFIED_PIXEL_FORMAT = FC2_UNSPECIFIED_PIXEL_FORMAT

cdef class INTERFACE_TYPE:
    """Interfaces that a camera may use to communicate with a host.

        UNKNOWN: Unknown interface.
    """
    IEEE1394 = FC2_INTERFACE_IEEE1394
    USB_2 = FC2_INTERFACE_USB_2
    USB_3 = FC2_INTERFACE_USB_3
    GIGE = FC2_INTERFACE_GIGE
    UNKNOWN = FC2_INTERFACE_UNKNOWN

cdef class DRIVER_TYPE:
    """Types of low level drivers that FlyCapture uses.

        UNKNOWN: Unknown driver type.
    """
    CAM_1394 = FC2_DRIVER_1394_CAM
    PRO_1394 = FC2_DRIVER_1394_PRO
    JUJU_1394 = FC2_DRIVER_1394_JUJU
    VIDEO1394 = FC2_DRIVER_1394_VIDEO1394
    RAW1394 = FC2_DRIVER_1394_RAW1394
    USB_NONE = FC2_DRIVER_USB_NONE
    USB_CAM = FC2_DRIVER_USB_CAM
    USB3_PRO = FC2_DRIVER_USB3_PRO
    GIGE_NONE = FC2_DRIVER_GIGE_NONE
    GIGE_FILTER = FC2_DRIVER_GIGE_FILTER
    GIGE_PRO = FC2_DRIVER_GIGE_PRO
    GIGE_LWF = FC2_DRIVER_GIGE_LWF
    UNKNOWN = FC2_DRIVER_UNKNOWN

cdef class BANDWIDTH_ALLOCATION:
    """Bandwidth allocation options for 1394 devices.

        UNSPECIFIED: Not specified. This leaves the current setting unchanged.
    """
    OFF = FC2_BANDWIDTH_ALLOCATION_OFF
    ON = FC2_BANDWIDTH_ALLOCATION_ON
    UNSUPPORTED = FC2_BANDWIDTH_ALLOCATION_UNSUPPORTED
    UNSPECIFIED = FC2_BANDWIDTH_ALLOCATION_UNSPECIFIED

cdef class GRAB_MODE:
    """The grab strategy employed during image transfer.

        UNSPECIFIED_GRAB_MODE Unspecified grab mode.
    """
    DROP_FRAMES = FC2_DROP_FRAMES
    BUFFER_FRAMES = FC2_BUFFER_FRAMES
    UNSPECIFIED_GRAB_MODE = FC2_UNSPECIFIED_GRAB_MODE

cdef class BUS_SPEED:
    """Bus speeds.

        SPEED_UNKNOWN: Unknown bus speed.
    """
    S100 = FC2_BUSSPEED_S100
    S200 = FC2_BUSSPEED_S200
    S400 = FC2_BUSSPEED_S400
    S480 = FC2_BUSSPEED_S480
    S800 = FC2_BUSSPEED_S800
    S1600 = FC2_BUSSPEED_S1600
    S3200 = FC2_BUSSPEED_S3200
    S5000 = FC2_BUSSPEED_S5000
    BASE_T_10 = FC2_BUSSPEED_10BASE_T
    BASE_T_100 = FC2_BUSSPEED_100BASE_T
    BASE_T_1000 = FC2_BUSSPEED_1000BASE_T
    BASE_T_10000 = FC2_BUSSPEED_10000BASE_T
    S_FASTEST = FC2_BUSSPEED_S_FASTEST
    ANY = FC2_BUSSPEED_ANY
    SPEED_UNKNOWN = FC2_BUSSPEED_SPEED_UNKNOWN

cdef class PROPERTY_TYPE:
    """Camera properties.

        UNSPECIFIED_PROPERTY_TYPE
    """
    BRIGHTNESS = FC2_BRIGHTNESS
    AUTO_EXPOSURE = FC2_AUTO_EXPOSURE
    SHARPNESS = FC2_SHARPNESS
    WHITE_BALANCE = FC2_WHITE_BALANCE
    HUE = FC2_HUE
    SATURATION = FC2_SATURATION
    GAMMA = FC2_GAMMA
    IRIS = FC2_IRIS
    FOCUS = FC2_FOCUS
    ZOOM = FC2_ZOOM
    PAN = FC2_PAN
    TILT = FC2_TILT
    SHUTTER = FC2_SHUTTER
    GAIN = FC2_GAIN
    TRIGGER_MODE = FC2_TRIGGER_MODE
    TRIGGER_DELAY = FC2_TRIGGER_DELAY
    FRAME_RATE = FC2_FRAME_RATE
    TEMPERATURE = FC2_TEMPERATURE
    UNSPECIFIED_PROPERTY_TYPE = FC2_UNSPECIFIED_PROPERTY_TYPE

cdef class VIDEO_MODE:
    """DCAM video modes.

        NUM_VIDEOMODES: Number of possible video modes
    """
    VM_160x120YUV444 = FC2_VIDEOMODE_160x120YUV444
    VM_320x240YUV422 = FC2_VIDEOMODE_320x240YUV422
    VM_640x480YUV411 = FC2_VIDEOMODE_640x480YUV411
    VM_640x480YUV422 = FC2_VIDEOMODE_640x480YUV422
    VM_640x480RGB = FC2_VIDEOMODE_640x480RGB
    VM_640x480Y8 = FC2_VIDEOMODE_640x480Y8
    VM_640x480Y16 = FC2_VIDEOMODE_640x480Y16
    VM_800x600YUV422 = FC2_VIDEOMODE_800x600YUV422
    VM_800x600RGB = FC2_VIDEOMODE_800x600RGB
    VM_800x600Y8 = FC2_VIDEOMODE_800x600Y8
    VM_800x600Y16 = FC2_VIDEOMODE_800x600Y16
    VM_1024x768YUV422 = FC2_VIDEOMODE_1024x768YUV422
    VM_1024x768RGB = FC2_VIDEOMODE_1024x768RGB
    VM_1024x768Y8 = FC2_VIDEOMODE_1024x768Y8
    VM_1024x768Y16 = FC2_VIDEOMODE_1024x768Y16
    VM_1280x960YUV422 = FC2_VIDEOMODE_1280x960YUV422
    VM_1280x960RGB = FC2_VIDEOMODE_1280x960RGB
    VM_1280x960Y8 = FC2_VIDEOMODE_1280x960Y8
    VM_1280x960Y16 = FC2_VIDEOMODE_1280x960Y16
    VM_1600x1200YUV422 = FC2_VIDEOMODE_1600x1200YUV422
    VM_1600x1200RGB = FC2_VIDEOMODE_1600x1200RGB
    VM_1600x1200Y8 = FC2_VIDEOMODE_1600x1200Y8
    VM_1600x1200Y16 = FC2_VIDEOMODE_1600x1200Y16
    FORMAT7 = FC2_VIDEOMODE_FORMAT7
    NUM_VIDEOMODES = FC2_NUM_VIDEOMODES

cdef class FRAMERATE:
    """Frame rates in frames per second.

        FORMAT7: Custom frame rate for Format7 functionality.
    """
    FR_1_875 = FC2_FRAMERATE_1_875
    FR_3_75 = FC2_FRAMERATE_3_75
    FR_7_5 = FC2_FRAMERATE_7_5
    FR_15 = FC2_FRAMERATE_15
    FR_30 = FC2_FRAMERATE_30
    FR_60 = FC2_FRAMERATE_60
    FR_120 = FC2_FRAMERATE_120
    FR_240 = FC2_FRAMERATE_240
    FORMAT7 = FC2_FRAMERATE_FORMAT7

cdef class MODE:
    """Camera modes for DCAM formats as well as Format7.

        FC2_NUM_MODES: Number of modes.
    """
    MODE_0 = FC2_MODE_0
    MODE_1 = FC2_MODE_1
    MODE_2 = FC2_MODE_2
    MODE_3 = FC2_MODE_3
    MODE_4 = FC2_MODE_4
    MODE_5 = FC2_MODE_5
    MODE_6 = FC2_MODE_6
    MODE_7 = FC2_MODE_7
    MODE_8 = FC2_MODE_8
    MODE_9 = FC2_MODE_9
    MODE_10 = FC2_MODE_10
    MODE_11 = FC2_MODE_11
    MODE_12 = FC2_MODE_12
    MODE_13 = FC2_MODE_13
    MODE_14 = FC2_MODE_14
    MODE_15 = FC2_MODE_15
    MODE_16 = FC2_MODE_16
    MODE_17 = FC2_MODE_17
    MODE_18 = FC2_MODE_18
    MODE_19 = FC2_MODE_19
    MODE_20 = FC2_MODE_20
    MODE_21 = FC2_MODE_21
    MODE_22 = FC2_MODE_22
    MODE_23 = FC2_MODE_23
    MODE_24 = FC2_MODE_24
    MODE_25 = FC2_MODE_25
    MODE_26 = FC2_MODE_26
    MODE_27 = FC2_MODE_27
    MODE_28 = FC2_MODE_28
    MODE_29 = FC2_MODE_29
    MODE_30 = FC2_MODE_30
    MODE_31 = FC2_MODE_31
    NUM_MODES = FC2_NUM_MODES

cdef class GIGE_PROPERTY_TYPE:
    """Possible properties that can be queried from the camera.

        GIGE_PACKET_DELAY
    """
    GIGE_HEARTBEAT = FC2_HEARTBEAT
    GIGE_HEARTBEAT_TIMEOUT = FC2_HEARTBEAT_TIMEOUT
    GIGE_PACKET_SIZE = PACKET_SIZE
    GIGE_PACKET_DELAY = PACKET_DELAY

cdef class PCIE_BUS_SPEED:
    """Speed of PCIE busses.

        FC2_PCIE_BUSSPEED_UNKNOWN: Speed is unknown
    """
    PCIE_2_5 = FC2_PCIE_BUSSPEED_2_5
    PCIE_5_0 = FC2_PCIE_BUSSPEED_5_0
    UNKNOWN = FC2_PCIE_BUSSPEED_UNKNOWN

cdef class BAYER_FORMAT:
    """Bayer tile formats.

        BGGR; Blue-Green-Green-Red.
    """
    NONE = FC2_BT_NONE
    RGGB = FC2_BT_RGGB
    GRBG = FC2_BT_GRBG
    GBRG = FC2_BT_GBRG
    BGGR = FC2_BT_BGGR

cdef class COLOR_PROCESSING:
    """Color processing algorithms.

        DIRECTIONAL: Best quality but much faster than rigorous
    """
    DEFAULT = FC2_DEFAULT
    NO_COLOR_PROCESSING = FC2_NO_COLOR_PROCESSING
    NEAREST_NEIGHBOR_FAST = FC2_NEAREST_NEIGHBOR_FAST
    EDGE_SENSING = FC2_EDGE_SENSING
    HQ_LINEAR = FC2_HQ_LINEAR
    RIGOROUS = FC2_RIGOROUS
    IPP = FC2_IPP
    DIRECTIONAL = FC2_DIRECTIONAL

cdef class STATISTICS_CHANNEL:
    """Channels that allow statistics to be calculated.

        LIGHTNESS
    """
    GREY = FC2_STATISTICS_GREY
    RED = FC2_STATISTICS_RED
    GREEN = FC2_STATISTICS_GREEN
    BLUE = FC2_STATISTICS_BLUE
    HUE = FC2_STATISTICS_HUE
    SATURATION = FC2_STATISTICS_SATURATION
    LIGHTNESS = FC2_STATISTICS_LIGHTNESS

cdef class TIFF_COMPRESSION:
    """TIFF compression method.

        JPEG: Save using JPEG compression. This is only valid for 8-bit
        greyscale and 24-bit only. Default to LZW for other bit depths.
    """
    NONE = FC2_TIFF_NONE
    PACKBITS = FC2_TIFF_PACKBITS
    DEFLATE = FC2_TIFF_DEFLATE
    ADOBE_DEFLATE = FC2_TIFF_ADOBE_DEFLATE
    CCITTFAX3 = FC2_TIFF_CCITTFAX3
    CCITTFAX4 = FC2_TIFF_CCITTFAX4
    JPEG = FC2_TIFF_JPEG

cdef class OS_TYPE:
    """Possible operating systems.

        UNKNOWN_OS: Unknown operating system
    """
    WINDOWS_X86 = FC2_WINDOWS_X86
    WINDOWS_X64 = FC2_WINDOWS_X64
    LINUX_X86 = FC2_LINUX_X86
    LINUX_X64 = FC2_LINUX_X64
    MAC = FC2_MAC
    UNKNOWN_OS = FC2_UNKNOWN_OS

cdef class BYTE_ORDER:
    """Possible byte order.

        BIG_ENDIAN
    """
    LITTLE_ENDIAN = FC2_BYTE_ORDER_LITTLE_ENDIAN
    BIG_ENDIAN = FC2_BYTE_ORDER_BIG_ENDIAN

cdef class NODE_TYPE:
    """Type of node.

        NODE_NODE: Unknown node type.
    """
    NODE_COMPUTER = COMPUTER
    NODE_BUS = BUS
    NODE_CAMERA = CAMERA
    NODE_NODE = NODE

cdef class PORT_TYPE:
    """Possible states of a port on a node.

        PORT_CONNECTED_TO_CHILD
    """
    PORT_NOT_CONNECTED = NOT_CONNECTED
    PORT_CONNECTED_TO_PARENT = CONNECTED_TO_PARENT
    PORT_CONNECTED_TO_CHILD = CONNECTED_TO_CHILD

cdef class BUS_CALLBACK_TYPE:
    """The type of bus callback to register a callback function for.

        REMOVAL: Register for removals only.
    """
    BUS_RESET = FC2_BUS_RESET
    ARRIVAL = FC2_ARRIVAL
    REMOVAL = FC2_REMOVAL


# classes
class Fc2error(Exception):
    """An error raised by the PyCapture2 library."""
    def __init__(self, errorNum):
        """Initialize the error with the index of the error."""
        self.errorNum = errorNum

    def __str__(self):
        """Get the name of the error from the error index."""
        errorStr = fc2ErrorToDescription(self.errorNum)
        return repr(errorStr)


class ConfigROM:
    """camera configuration ROM class.

        pszKeyword (str): keyword.
    """
    def __init__(self, nodeVendorId=0, chipIdHi=0, chipIdLo=0, unitSpecId=0,
                 unitSWVer=0, unitSubSWVer=0, vendorUniqueInfo_0=0,
                 vendorUniqueInfo_1=0, vendorUniqueInfo_2=0,
                 vendorUniqueInfo_3=0, pszKeyword="", ):
        self.nodeVendorId = nodeVendorId
        self.chipIdHi = chipIdHi
        self.chipIdLo = chipIdLo
        self.unitSpecId = unitSpecId
        self.unitSWVer = unitSWVer
        self.unitSubSWVer = unitSubSWVer
        self.vendorUniqueInfo_0 = vendorUniqueInfo_0
        self.vendorUniqueInfo_1 = vendorUniqueInfo_1
        self.vendorUniqueInfo_2 = vendorUniqueInfo_2
        self.vendorUniqueInfo_3 = vendorUniqueInfo_3
        self.pszKeyword = pszKeyword


class CameraInfo:
    """camera information class.


    """
    def __init__(self, serialNumber=0, interfaceType=0, driverType=0,
                 isColorCamera=0, modelName="", vendorName="", sensorInfo="",
                 sensorResolution="", driverName="", firmwareVersion="",
                 firmwareBuildTime="", maximumBusSpeed=0, pcieBusSpeed=0,
                 bayerTileFormat=0, busNumber=0, nodeNumber=0, iidcVer=0,
                 configROM=ConfigROM(), gigEMajorVersion=0, gigEMinorVersion=0,
                 userDefinedName="", xmlURL1="", xmlURL2="",
                 macAddress=(0, 0, 0, 0, 0, 0), ipAddress=(0, 0, 0, 0),
                 subnetMask=(0, 0, 0, 0), defaultGateway=(0, 0, 0, 0),
                 ccpStatus=0, applicationIPAddress=0, applicationPort=0):
        self.serialNumber = serialNumber
        self.interfaceType = interfaceType
        self.driverType = driverType
        self.isColorCamera = isColorCamera
        self.modelName = modelName
        self.vendorName = vendorName
        self.sensorInfo = sensorInfo
        self.sensorResolution = sensorResolution
        self.driverName = driverName
        self.firmwareVersion = firmwareVersion
        self.firmwareBuildTime = firmwareBuildTime
        self.maximumBusSpeed = maximumBusSpeed
        self.pcieBusSpeed = pcieBusSpeed
        self.bayerTileFormat = bayerTileFormat
        self.busNumber = busNumber
        self.nodeNumber = nodeNumber
        self.iidcVer = iidcVer
        self.configROM = configROM
        self.gigEMajorVersion = gigEMajorVersion
        self.gigEMinorVersion = gigEMinorVersion
        self.userDefinedName = userDefinedName
        self.xmlURL1 = xmlURL1
        self.xmlURL2 = xmlURL2
        self.macAddress = macAddress
        self.ipAddress = ipAddress
        self.subnetMask = subnetMask
        self.defaultGateway = defaultGateway
        self.ccpStatus = ccpStatus
        self.applicationIPAddress = applicationIPAddress
        self.applicationPort = applicationPort


class AvailableImageInfo:
    """Structure containing the availability of image properties, for use in
    EmbeddedImageInfo class.

        ROIPosition (bool): Whether specifying the region of interest is
        specified by the camera.
    """
    def __init__(self, timestamp=False, gain=False, shutter=False,
                 brightness=False, exposure=False, whiteBalance=False,
                 frameCounter=False, strobePattern=False, ROIPosition=False):
        self.timestamp = timestamp
        self.gain = gain
        self.shutter = shutter
        self.brightness = brightness
        self.exposure = exposure
        self.whiteBalance = whiteBalance
        self.frameCounter = frameCounter
        self.strobePattern = strobePattern
        self.ROIPosition = ROIPosition


class EmbeddedImageInfo:
    """Structure containing the current status and availability (via the
    "available" attribute) of image properties.

        available (PyCapture2.availableImageInfo): Whether these properties are
        available for modification. See AvailableImageInfo for reference.
    """
    def __init__(self, timestamp=False, gain=False, shutter=False,
                 brightness=False, exposure=False, whiteBalance=False,
                 frameCounter=False, strobePattern=False, GPIOPinState=False,
                 ROIPosition=False, available=AvailableImageInfo()):
        self.timestamp = timestamp
        self.gain = gain
        self.shutter = shutter
        self.brightness = brightness
        self.exposure = exposure
        self.whiteBalance = whiteBalance
        self.frameCounter = frameCounter
        self.strobePattern = strobePattern
        self.GPIOPinState = GPIOPinState
        self.ROIPosition = ROIPosition
        self.available = available


class Config:
    """Structure containing the configuration for a camera.

        registerTimeout (int): Register read/write timeout value (in
        microseconds).
    """
    def __init__(self, numBuffers=0, numImageNotifications=0,
                 minNumImageNotifications=0, grabTimeout=0, grabMode=0,
                 isochBusSpeed=0, asyncBusSpeed=0, bandwidthAllocation=0,
                 registerTimeoutRetries=0, registerTimeout=0, ):
        self.numBuffers = numBuffers
        self.numImageNotifications = numImageNotifications
        self.minNumImageNotifications = minNumImageNotifications
        self.grabTimeout = grabTimeout
        self.grabMode = grabMode
        self.isochBusSpeed = isochBusSpeed
        self.asyncBusSpeed = asyncBusSpeed
        self.bandwidthAllocation = bandwidthAllocation
        self.registerTimeoutRetries = registerTimeoutRetries
        self.registerTimeout = registerTimeout


class TriggerMode:
    """Properties of a camera trigger.

        parameter (int): The parameter value.
    """
    def __init__(self, onOff=False, polarity=0, source=0, mode=0, parameter=0,
                 ):
        self.onOff = onOff
        self.polarity = polarity
        self.source = source
        self.mode = mode
        self.parameter = parameter


class PropertyInfo:
    """Information about a specific camera property.

        unitAbbr (str): Abbreviated textual description of units.
    """
    def __init__(self, type=0, present=False, autoSupported=False,
                 manualSupported=False, onOffSupported=False,
                 onePushSupported=False, absValSupported=False,
                 readOutSupported=False, min=0, max=0, absMin=0.0, absMax=0.0,
                 units="", unitAbbr="", ):
        self.type = type
        self.present = present
        self.autoSupported = autoSupported
        self.manualSupported = manualSupported
        self.onOffSupported = onOffSupported
        self.onePushSupported = onePushSupported
        self.absValSupported = absValSupported
        self.readOutSupported = readOutSupported
        self.min = min
        self.max = max
        self.absMin = absMin
        self.absMax = absMax
        self.units = units
        self.unitAbbr = unitAbbr


class Property:
    """Data from a specific camera property.

        absValue (float): Floating point value.
    """
    def __init__(self, propType=0, present=False, absControl=False,
                 onePush=False, onOff=False, autoManualMode=False, valueA=0,
                 valueB=0, absValue=0.0, ):
        self.type = propType
        self.present = present
        self.absControl = absControl
        self.onePush = onePush
        self.onOff = onOff
        self.autoManualMode = autoManualMode
        self.valueA = valueA
        self.valueB = valueB
        self.absValue = absValue


class StrobeInfo:
    """Information about the camera's strobe settings and capabilities.

        maxValue (float): Maximum value.
    """
    def __init__(self, source=0, present=False, readOutSupported=False,
                 onOffSupported=False, polaritySupported=False, minValue=0.0,
                 maxValue=0.0, ):
        self.source = source
        self.present = present
        self.readOutSupported = readOutSupported
        self.onOffSupported = onOffSupported
        self.polaritySupported = polaritySupported
        self.minValue = minValue
        self.maxValue = maxValue


class StrobeControl:
    """Data about the camera strobe.

        duration (float): Signal duration (in ms)
    """
    def __init__(self, source=0, onOff=False, polarity=0, delay=0.0,
                 duration=0.0, ):
        self.source = source
        self.onOff = onOff
        self.polarity = polarity
        self.delay = delay
        self.duration = duration


class LUTData:
    """Information about the camera's lookup table.

        numEntries (int): The number of entries in the LUT.
    """
    def __init__(self, supported=False, enabled=False, numBanks=0,
                 numChannels=0, inputBitDepth=0, outputBitDepth=0,
                 numEntries=0, ):
        self.supported = supported
        self.enabled = enabled
        self.numBanks = numBanks
        self.numChannels = numChannels
        self.inputBitDepth = inputBitDepth
        self.outputBitDepth = outputBitDepth
        self.numEntries = numEntries


class TimeStamp:
    """Information detailing the time an image was taken.

        cycleOffset (int): 1394 cycle time offset.
    """
    def __init__(self, seconds=0, microSeconds=0, cycleSeconds=0,
                 cycleCount=0, cycleOffset=0, ):
        self.seconds = seconds
        self.microSeconds = microSeconds
        self.cycleSeconds = cycleSeconds
        self.cycleCount = cycleCount
        self.cycleOffset = cycleOffset


class CameraStats:
    """Diagnostic information about the camera.


    """
    def __init__(self, imageDropped=0, imageCorrupt=0, imageXmitFailed=0,
                 imageDriverDropped=0, regReadFailed=0, regWriteFailed=0,
                 portErrors=0, cameraPowerUp=False, cameraVoltages=(0.0,),
                 cameraCurrents=(0.0,), temperature=0,
                 timeSinceInitialization=0, timeSinceBusReset=0, timeStamp=0,
                 numResendPacketsRequested=0, numResendPacketsReceived=0, ):
        self.imageDropped = imageDropped
        self.imageCorrupt = imageCorrupt
        self.imageXmitFailed = imageXmitFailed
        self.imageDriverDropped = imageDriverDropped
        self.regReadFailed = regReadFailed
        self.regWriteFailed = regWriteFailed
        self.portErrors = portErrors
        self.cameraPowerUp = cameraPowerUp
        self.cameraVoltages = cameraVoltages
        self.cameraCurrents = cameraCurrents
        self.temperature = temperature
        self.timeSinceInitialization = timeSinceInitialization
        self.timeSinceBusReset = timeSinceBusReset
        self.timeStamp = timeStamp
        self.numResendPacketsRequested = numResendPacketsRequested
        self.numResendPacketsReceived = numResendPacketsReceived


class Format7Info:
    """Format 7 information for a single mode.

        percentage (float): Current packet size as a percentage of maximum.
    """
    def __init__(self, mode=0, maxWidth=0, maxHeight=0, offsetHStepSize=0,
                 offsetVStepSize=0, imageHStepSize=0, imageVStepSize=0,
                 pixelFormatBitField=0, vendorPixelFormatBitField=0,
                 packetSize=0, minPacketSize=0, maxPacketSize=0,
                 percentage=0.0, ):
        self.mode = mode
        self.maxWidth = maxWidth
        self.maxHeight = maxHeight
        self.offsetHStepSize = offsetHStepSize
        self.offsetVStepSize = offsetVStepSize
        self.imageHStepSize = imageHStepSize
        self.imageVStepSize = imageVStepSize
        self.pixelFormatBitField = pixelFormatBitField
        self.vendorPixelFormatBitField = vendorPixelFormatBitField
        self.packetSize = packetSize
        self.minPacketSize = minPacketSize
        self.maxPacketSize = maxPacketSize
        self.percentage = percentage


class Format7ImageSettings:
    """Format 7 image settings.

        PixelFormat (int): Pixel format of image. Use PIXEL_FORMAT to set
        correctly.
    """
    def __init__(self, mode=0, offsetX=0, offsetY=0, width=0, height=0,
                 pixelFormat=0):
        self.mode = mode
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.width = width
        self.height = height
        self.pixelFormat = pixelFormat


class Format7PacketInfo:
    """Format 7 packet information.

        minBytesPerPacket (int): Minimum bytes per packet.
    """
    def __init__(self, recommendedBytesPerPacket=0, maxBytesPerPacket=0,
                 unitBytesPerPacket=0, ):
        self.recommendedBytesPerPacket = recommendedBytesPerPacket
        self.maxBytesPerPacket = maxBytesPerPacket
        self.unitBytesPerPacket = unitBytesPerPacket


class GigEProperty:
    """A property specific to GigE cameras.

        value (int): Current value of the property.
    """
    def __init__(self, propType=0, isReadable=False, isWritable=False, min=0,
                 max=0, value=0, ):
        self.propType = propType
        self.isReadable = isReadable
        self.isWritable = isWritable
        self.min = min
        self.max = max
        self.value = value


class GigEImageSettingsInfo:
    """Format 7 information for a single mode.

        vendorPixelFormatBitField (int): Vendor unique pixel formats in a bit
        field.
    """
    def __init__(self, maxWidth=0, maxHeight=0, offsetHStepSize=0,
                 offsetVStepSize=0, imageHStepSize=0, imageVStepSize=0,
                 pixelFormatBitField=0, vendorPixelFormatBitField=0, ):
        self.maxWidth = maxWidth
        self.maxHeight = maxHeight
        self.offsetHStepSize = offsetHStepSize
        self.offsetVStepSize = offsetVStepSize
        self.imageHStepSize = imageHStepSize
        self.imageVStepSize = imageVStepSize
        self.pixelFormatBitField = pixelFormatBitField
        self.vendorPixelFormatBitField = vendorPixelFormatBitField


class GigEImageSettings:
    """Image settings for a GigE camera.

        pixelFormat (int): Pixel format of image. Use PyCapture2.PIXEL_FORMAT
        to set correctly.
    """
    def __init__(self, offsetX=0, offsetY=0, width=0, height=0, pixelFormat=0,
                 ):
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.width = width
        self.height = height
        self.pixelFormat = pixelFormat


class GigEStreamChannel:
    """Information about a single GigE stream channel.

        sourcePort (int): Source UDP port of the stream channel.
    """
    def __init__(self, networkInterfaceIndex=0, hostPort=0,
                 doNotFragment=False, packetSize=0, interPacketDelay=0,
                 destinationIpAddress=(0, 0, 0, 0), sourcePort=0, ):
        self.networkInterfaceIndex = networkInterfaceIndex
        self.hostPort = hostPort
        self.doNotFragment = doNotFragment
        self.packetSize = packetSize
        self.interPacketDelay = interPacketDelay
        self.destinationIpAddress = destinationIpAddress
        self.sourcePort = sourcePort


class GigEConfig:
    """Configuration for a GigE camera.

        registerTimeout (int): Register read/write timeout value (in
        microseconds).
    """
    def __init__(self, enablePacketResend=False, registerTimeoutRetries=0,
                 registerTimeout=0, ):
        self.enablePacketResend = enablePacketResend
        self.registerTimeoutRetries = registerTimeoutRetries
        self.registerTimeout = registerTimeout


class SystemInfo:
    """Description of the system connected to the camera.

        screenHeight (int): Screen resolution height, in pixels.
    """
    def __init__(self, osType=0, osDescription="", byteOrder=0, sysMemSize=0,
                 cpuDescription="", numCpuCores=0, driverList="",
                 libraryList="", gpuDescription="", screenWidth=0,
                 screenHeight=0, ):
        self.osType = osType
        self.osDescription = osDescription
        self.byteOrder = byteOrder
        self.sysMemSize = sysMemSize
        self.cpuDescription = cpuDescription
        self.numCpuCores = numCpuCores
        self.driverList = driverList
        self.libraryList = libraryList
        self.gpuDescription = gpuDescription
        self.screenWidth = screenWidth
        self.screenHeight = screenHeight


class CallbackData:
    """Data returned from an event callback.

        eventTimestamp (long): Time as reported by the camera at which the
        exposure operation completed. Please note this is NOT a
        PyCapture2.TimeStamp object!
    """
    def __init__(self, eventName="", eventID=0, eventTimestamp=0, ):
        self.eventName = eventName
        self.eventID = eventID
        self.eventTimestamp = eventTimestamp


class PNGOption:
    """Structure containing options for saving PNG images.

        compressionLevel (int): Level of compression for the image, on the
        range (0-9)
        """
    def __init__(self, interlaced=False, compressionLevel=0, ):
        self.interlaced = interlaced
        self.compressionLevel = compressionLevel


class PPMOption:
    """Structure containing options for saving PPM images.

        binaryFile (bool): Whether to save the PPM as a binary file.
    """
    def __init__(self, binaryFile=False, ):
        self.binaryFile = binaryFile


class PGMOption:
    """Structure containing options for saving PGM images.

        binaryFile (bool): Whether to save the PGM as a binary file.
    """
    def __init__(self, binaryFile=False, ):
        self.binaryFile = binaryFile


class TIFFOption:
    """Structure containing options for saving TIFF images.

        compression (int): Compression method to use for encoding. Use
        TIFF_COMPRESSION to set correctly.
    """
    def __init__(self, compression=0, ):
        self.compression = compression


class JPEGOption:
    """Structure containing options for saving JPEG images.

        quality (int): JPEG image quality in range (0-100)
    """
    def __init__(self, progressive=False, quality=0, ):
        self.progressive = progressive
        self.quality = quality


class JPG2Option:
    """Structure containing options for saving JPEG2000 images.

        quality (int): JPEG saving quality in range (1-512)
    """
    def __init__(self, quality=0, ):
        self.quality = quality


class BMPOption:
    """Structure containing options for saving Bitmap images.

        indexedColor_8bit (bool): WHether to save with 8bit indexed color.
    """
    def __init__(self, indexedColor_8bit=False, ):
        self.indexedColor_8bit = indexedColor_8bit


cdef class BaseCamera:
    """Base camera class. This class exists only for camera and GigECamera
    classes to inherit from it."""
    cdef fc2Context c
    cdef public bint isConnected
    cdef fc2EventOptions evntOpts
    cdef tuple callbackArgs
    cdef dict eventArgs

    def __init__(self):
        """camera() -> Camera object
        Initialize and return a Camera object.
        This function takes no arguments.
        """
        self.eventArgs = {}
        self.callbackArgs = ()

    def __dealloc__(self):
        fc2DestroyContext(self.c)

    def connect(self, tuple IDTup):
        """camera.connect(guid) -> None

        """
        cdef fc2PGRGuid camID
        self.isConnected = True
        camID.value[0], camID.value[1], camID.value[2], camID.value[3] = IDTup
        checkError(fc2Connect(self.c, &camID))

    def disconnect(self):
        """camera.disconnect() -> None

        Disconnect the camera object from a physical camera.
        """
        checkError(fc2Disconnect(self.c))
        self.isConnected = False

    def startCapture(self, func=None, *extraArgs):
        """camera.startCapture(function = None, *arguments) -> None

            arguments: The rest of the arguments are the arguments for the
            callback function.
        """
        if func:
            self.callbackArgs = (func, extraArgs)
            checkError(fc2StartCaptureCallback(self.c,
                                               <fc2ImageEventCallback>&imageCallbackBridge,
                                               <void*>self.callbackArgs))
        else:
            checkError(fc2StartCapture(self.c))

    def stopCapture(self):
        """camera.stopCapture() -> None

        Stop capturing data from camera.
        """
        checkError(fc2StopCapture(self.c))

    def retrieveBuffer(self):
        """camera.retrieveBuffer() -> image

            image (PyCapture2.Image): The image object crabbed from the camera.
        """
        img = Image()
        checkError(fc2RetrieveBuffer(self.c, &img.i))
        return img

    def getCameraInfo(self):
        """camera.getCameraInfo() -> cameraInfo

        """
        cdef fc2CameraInfo camInfo
        checkError(fc2GetCameraInfo(self.c, &camInfo))
        returnVal = CameraInfo()
        returnVal.__dict__ = {
            "serialNumber": camInfo.serialNumber,
            "interfaceType": camInfo.interfaceType,
            "driverType": camInfo.driverType,
            "isColorCamera": camInfo.isColorCamera,
            "modelName": camInfo.modelName,
            "vendorName": camInfo.vendorName,
            "sensorInfo": camInfo.sensorInfo,
            "sensorResolution": camInfo.sensorResolution,
            "driverName": camInfo.driverName,
            "firmwareVersion": camInfo.firmwareVersion,
            "firmwareBuildTime": camInfo.firmwareBuildTime,
            "maximumBusSpeed": camInfo.maximumBusSpeed,
            "pcieBusSpeed": camInfo.pcieBusSpeed,
            "bayerTileFormat": camInfo.bayerTileFormat,
            "busNumber": camInfo.busNumber,
            "nodeNumber": camInfo.nodeNumber,
            "iidcVer": camInfo.iidcVer,
            "configROM": {
                "nodeVendorId": camInfo.configROM.nodeVendorId,
                "chipIdHi": camInfo.configROM.chipIdHi,
                "chipIdLo": camInfo.configROM.chipIdLo,
                "unitSpecId": camInfo.configROM.unitSpecId,
                "unitSWVer": camInfo.configROM.unitSWVer,
                "unitSubSWVer": camInfo.configROM.unitSubSWVer,
                "vendorUniqueInfo_0": camInfo.configROM.vendorUniqueInfo_0,
                "vendorUniqueInfo_1": camInfo.configROM.vendorUniqueInfo_1,
                "vendorUniqueInfo_2": camInfo.configROM.vendorUniqueInfo_2,
                "vendorUniqueInfo_3": camInfo.configROM.vendorUniqueInfo_3,
                "pszKeyword": camInfo.configROM.pszKeyword
            },
            "gigEMajorVersion": camInfo.gigEMajorVersion,
            "gigEMinorVersion": camInfo.gigEMinorVersion,
            "userDefinedName": camInfo.userDefinedName,
            "xmlURL1": camInfo.xmlURL1,
            "xmlURL2": camInfo.xmlURL2,
            "macAddress": (camInfo.macAddress.octets[0],
                           camInfo.macAddress.octets[1],
                           camInfo.macAddress.octets[2],
                           camInfo.macAddress.octets[3],
                           camInfo.macAddress.octets[4],
                           camInfo.macAddress.octets[5]),
            "ipAddress": (camInfo.ipAddress.octets[0],
                          camInfo.ipAddress.octets[1],
                          camInfo.ipAddress.octets[2],
                          camInfo.ipAddress.octets[3]),
            "subnetMask": (camInfo.subnetMask.octets[0],
                           camInfo.subnetMask.octets[1],
                           camInfo.subnetMask.octets[2],
                           camInfo.subnetMask.octets[3]),
            "defaultGateway": (camInfo.defaultGateway.octets[0],
                               camInfo.defaultGateway.octets[1],
                               camInfo.defaultGateway.octets[2],
                               camInfo.defaultGateway.octets[3]),
            "ccpStatus": camInfo.ccpStatus,
            "applicationIPAddress": camInfo.applicationIPAddress,
            "applicationPort": camInfo.applicationPort
        }
        return returnVal

    def getEmbeddedImageInfo(self):
        """camera.getEmbeddedImageInfo -> embeddedImageInfo

        """
        cdef fc2EmbeddedImageInfo imgInfo
        checkError(fc2GetEmbeddedImageInfo(self.c, &imgInfo))
        returnVal = EmbeddedImageInfo()
        returnVal.__dict__ = {
            "timestamp": imgInfo.timestamp.onOff,
            "gain":  imgInfo.gain.onOff,
            "shutter": imgInfo.shutter.onOff,
            "brightness": imgInfo.brightness.onOff,
            "exposure": imgInfo.exposure.onOff,
            "whiteBalance": imgInfo.whiteBalance.onOff,
            "frameCounter": imgInfo.frameCounter.onOff,
            "strobePattern": imgInfo.strobePattern.onOff,
            "GPIOPinState": imgInfo.GPIOPinState.onOff,
            "ROIPosition": imgInfo.ROIPosition.onOff,
            "available": type("availableImageInfo", (object,), {
                "timestamp": imgInfo.timestamp.available,
                "gain": imgInfo.gain.available,
                "shutter": imgInfo.shutter.available,
                "brightness": imgInfo.brightness.available,
                "exposure": imgInfo.exposure.available,
                "whiteBalance": imgInfo.whiteBalance.available,
                "frameCounter": imgInfo.frameCounter.available,
                "strobePattern": imgInfo.strobePattern.available,
                "GPIOPinState": imgInfo.GPIOPinState.available,
                "ROIPosition": imgInfo.ROIPosition.available
            })()
        }
        return returnVal

    def setEmbeddedImageInfo(self, *arg, **kwargs):
        """camera.setEmbeddedImageInfo(embeddedImageInfo = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__

        cdef fc2EmbeddedImageInfo imgInfoTmp, imgInfoFinal
        checkError(fc2GetEmbeddedImageInfo(self.c, &imgInfoTmp))
        imgInfoFinal.timestamp.onOff = kwargs.get("timestamp", imgInfoTmp.timestamp.onOff)
        imgInfoFinal.gain.onOff = kwargs.get("gainonOff", imgInfoTmp.gain.onOff)
        imgInfoFinal.shutter.onOff = kwargs.get("shutter", imgInfoTmp.shutter.onOff)
        imgInfoFinal.brightness.onOff = kwargs.get("brightness", imgInfoTmp.brightness.onOff)
        imgInfoFinal.exposure.onOff = kwargs.get("exposure", imgInfoTmp.exposure.onOff)
        imgInfoFinal.whiteBalance.onOff = kwargs.get("whiteBalance", imgInfoTmp.whiteBalance.onOff)
        imgInfoFinal.frameCounter.onOff = kwargs.get("frameCounter", imgInfoTmp.frameCounter.onOff)
        imgInfoFinal.strobePattern.onOff = kwargs.get("strobePattern", imgInfoTmp.strobePattern.onOff)
        imgInfoFinal.GPIOPinState.onOff = kwargs.get("GPIOPinState", imgInfoTmp.GPIOPinState.onOff)
        imgInfoFinal.ROIPosition.onOff = kwargs.get("ROIPosition", imgInfoTmp.ROIPosition.onOff)
        checkError(fc2SetEmbeddedImageInfo(self.c, &imgInfoFinal))

    def getConfiguration(self):
        """camera.getConfiguration() -> config

        """
        cdef fc2Config cnfg
        checkError(fc2GetConfiguration(self.c, &cnfg))
        returnVal = Config()
        returnVal.__dict__ = {
            "numBuffers ": cnfg.numBuffers,
            "numImageNotifications ": cnfg.numImageNotifications,
            "minNumImageNotifications ": cnfg.minNumImageNotifications,
            "grabTimeout ": cnfg.grabTimeout,
            "grabMode ": cnfg.grabMode,
            "isochBusSpeed ": cnfg.isochBusSpeed,
            "asyncBusSpeed ": cnfg.asyncBusSpeed,
            "bandwidthAllocation ": cnfg.bandwidthAllocation,
            "registerTimeoutRetries ": cnfg.registerTimeoutRetries,
            "registerTimeout": cnfg.registerTimeout
        }
        return returnVal

    def setConfiguration(self, *arg, **kwargs):
        """camera.setConfiguration(config = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__

        cdef fc2Config configTmp, configFinal
        checkError(fc2GetConfiguration(self.c, &configTmp))
        configFinal.numBuffers = kwargs.get("numBuffers", configTmp.numBuffers)
        configFinal.numImageNotifications = kwargs.get("numImageNotifications", configTmp.numImageNotifications)
        configFinal.minNumImageNotifications = kwargs.get("minNumImageNotifications", configTmp.minNumImageNotifications)
        configFinal.grabTimeout = kwargs.get("grabTimeout", configTmp.grabTimeout)
        configFinal.grabMode = kwargs.get("grabMode", configTmp.grabMode)
        configFinal.isochBusSpeed = kwargs.get("isochBusSpeed", configTmp.isochBusSpeed)
        configFinal.asyncBusSpeed = kwargs.get("asyncBusSpeed", configTmp.asyncBusSpeed)
        configFinal.bandwidthAllocation = kwargs.get("bandwidthAllocation", configTmp.bandwidthAllocation)
        configFinal.registerTimeoutRetries = kwargs.get("registerTimeoutRetries", configTmp.registerTimeoutRetries)
        configFinal.registerTimeout = kwargs.get("registerTimeout", configTmp.registerTimeout)
        checkError(fc2SetConfiguration(self.c, &configFinal))

    def getGPIOPinDirection(self, unsigned int pin):
        """camera.getGPIOPinDirection(pin) -> direction

        """
        cdef unsigned int pDirection
        checkError(fc2GetGPIOPinDirection(self.c, pin, &pDirection))
        return pDirection

    def setGPIOPinDirection(self, unsigned int pin, unsigned int direction):
        """camera.setGPIOPinDirection(pin, direction) -> None

            direction (int): The direction to set.
        """
        checkError(fc2SetGPIOPinDirection(self.c, pin, direction))

    def getTriggerMode(self):
        """camera.getTriggerMode() -> triggerMode

        """
        cdef fc2TriggerMode tMode
        checkError(fc2GetTriggerMode(self.c, &tMode))
        returnVal = TriggerMode()
        returnVal.__dict__ = {
            "onOff": tMode.onOff,
            "polarity": tMode.polarity,
            "source": tMode.source,
            "mode": tMode.mode,
            "parameter": tMode.parameter
        }
        return returnVal

    def setTriggerMode(self, *arg, **kwargs):
        """camera.setTriggerMode(TriggerMode = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__

        cdef fc2TriggerMode tModeFinal, tModeTemp
        checkError(fc2GetTriggerMode(self.c, &tModeTemp))

        tModeFinal.onOff = kwargs.get("onOff", tModeTemp.onOff)
        tModeFinal.polarity = kwargs.get("polarity", tModeTemp.polarity)
        tModeFinal.source = kwargs.get("source", tModeTemp.source)
        tModeFinal.mode = kwargs.get("mode", tModeTemp.mode)
        tModeFinal.parameter = kwargs.get("parameter", tModeTemp.parameter)

        checkError(fc2SetTriggerMode(self.c, &tModeFinal))

    def fireSoftwareTrigger(self):
        """camera.fireSoftwareTrigger() -> None

        Fire the software trigger, according to the DCAM specifications.
        """
        checkError(fc2FireSoftwareTrigger(self.c))

    def getTriggerDelayInfo(self):
        """camera.getTriggerDelayInfo() -> PropertyInfo

        """
        cdef fc2TriggerDelayInfo tdInfo
        checkError(fc2GetTriggerDelayInfo(self.c, &tdInfo))
        returnVal =  PropertyInfo()
        returnVal.__dict__ = {
            "type": tdInfo.type,
            "present": tdInfo.present,
            "autoSupported": tdInfo.autoSupported,
            "manualSupported": tdInfo.manualSupported,
            "onOffSupported": tdInfo.onOffSupported,
            "onePushSupported": tdInfo.onePushSupported,
            "absValSupported": tdInfo.absValSupported,
            "readOutSupported": tdInfo.readOutSupported,
            "min": tdInfo.min,
            "max": tdInfo.max,
            "absMin": tdInfo.absMin,
            "absMax": tdInfo.absMax,
            "units": tdInfo.pUnits,
            "unitAbbr": tdInfo.pUnitAbbr
        }
        return returnVal

    def getTriggerDelay(self):
        """camera.getTriggerDelay() -> property

        """
        cdef fc2TriggerDelay tDelay
        checkError(fc2GetTriggerDelay(self.c, &tDelay))
        returnVal = Property()
        returnVal.__dict__ = {
            "type": tDelay.type,
            "present": tDelay.present,
            "absControl": tDelay.absControl,
            "onePush": tDelay.onePush,
            "onOff": tDelay.onOff,
            "autoManualMode": tDelay.autoManualMode,
            "valueA": tDelay.valueA,
            "valueB": tDelay.valueB,
            "absValue": tDelay.absValue
        }
        return returnVal

    def setTriggerDelay(self, *arg, **kwargs):
        """camera.setTriggerDelay(property = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__

        cdef fc2TriggerDelay tDelayTmp, tDelayFinal
        checkError(fc2GetTriggerDelay(self.c, &tDelayTmp))
        tDelayFinal.type = kwargs.get("type", tDelayTmp.type)
        tDelayFinal.present = kwargs.get("present", tDelayTmp.present)
        tDelayFinal.absControl = kwargs.get("absControl", tDelayTmp.absControl)
        tDelayFinal.onePush = kwargs.get("onePush", tDelayTmp.onePush)
        tDelayFinal.onOff = kwargs.get("onOff", tDelayTmp.onOff)
        tDelayFinal.autoManualMode = kwargs.get("autoManualMode", tDelayTmp.autoManualMode)
        tDelayFinal.valueA = kwargs.get("valueA", tDelayTmp.valueA)
        tDelayFinal.valueB = kwargs.get("valueB", tDelayTmp.valueB)
        tDelayFinal.absValue = kwargs.get("absValue", tDelayTmp.absValue)
        checkError(fc2SetTriggerDelay(self.c, &tDelayFinal))

    def getStrobeInfo(self):
        """camera.getStrobeInfo() -> strobeInfo

        """
        cdef fc2StrobeInfo sInfo
        checkError(fc2GetStrobeInfo(self.c, &sInfo))
        returnVal = StrobeInfo()
        returnVal.__dict__ = {
            "source": sInfo.source,
            "present": sInfo.present,
            "readOutSupported": sInfo.readOutSupported,
            "onOffSupported": sInfo.onOffSupported,
            "polaritySupported": sInfo.polaritySupported,
            "minValue": sInfo.minValue,
            "maxValue": sInfo.maxValue
        }
        return returnVal

    def getStrobe(self):
        """camera.getStrobe() -> strobeControl

        """
        cdef fc2StrobeControl sCtrl
        checkError(fc2GetStrobe(self.c, &sCtrl))
        returnVal = StrobeControl()
        returnVal.__dict__ = {
            "source": sCtrl.source,
            "onOff": sCtrl.onOff,
            "polarity": sCtrl.polarity,
            "delay": sCtrl.delay,
            "duration": sCtrl.duration
        }
        return returnVal

    def setStrobe(self, *arg, **kwargs):
        """camera.setStrobe(strobeControl = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__

        cdef fc2StrobeControl sCtrlTmp, sCtrlFinal
        checkError(fc2GetStrobe(self.c, &sCtrlTmp))
        sCtrlFinal.source = kwargs.get("source", sCtrlTmp.source)
        sCtrlFinal.onOff = kwargs.get("onOff", sCtrlTmp.onOff)
        sCtrlFinal.polarity = kwargs.get("polarity", sCtrlTmp.polarity)
        sCtrlFinal.delay = kwargs.get("delay", sCtrlTmp.delay)
        sCtrlFinal.duration = kwargs.get("duration", sCtrlTmp.duration)
        checkError(fc2SetStrobe(self.c, &sCtrlFinal))

    def getLUTInfo(self):
        """camera.getLUTInfo() -> LUTData

        """
        cdef fc2LUTData lutData
        checkError(fc2GetLUTInfo(self.c, &lutData))
        returnVal = LUTData()
        returnVal.__dict__ = {
            "supported": lutData.supported,
            "enabled": lutData.enabled,
            "numBanks": lutData.numBanks,
            "numChannels": lutData.numChannels,
            "inputBitDepth": lutData.inputBitDepth,
            "outputBitDepth": lutData.outputBitDepth,
            "numEntries": lutData.numEntries
        }
        return returnVal

    def getLUTBankInfo(self, unsigned int bank):
        """camera.getLUTBankInfo(bank) -> readSupport, writeSupport

        """
        cdef bint readSupport, writeSupport
        checkError(fc2GetLUTBankInfo(self.c, bank, &readSupport, &writeSupport))
        return readSupport, writeSupport

    def getActiveLUTBank(self):
        """camera.getActiveLUTBank() -> activeBank

        """
        cdef unsigned int activeBank
        checkError(fc2GetActiveLUTBank(self.c, &activeBank))
        return activeBank

    def setActiveLUTBank(self, unsigned int activeBank):
        """camera.setActiveLUTBank(activeBank) -> None

            activeBank (int): an integer representing the LUT bank to set.
        """
        checkError(fc2SetActiveLUTBank(self.c, activeBank))

    def enableLUT(self, bint onOff):
        """camera.enableLUT(enable) -> None

            enable (bool): Whether to activate or deactivate the LUT.
        """
        checkError(fc2EnableLUT(self.c, onOff))

    def getLUTChannel(self, unsigned int bank, unsigned int channel, unsigned int numEntries):
        """camera.getLUTChannel(bank, channel, numEntries) -> entries


        """
        cdef unsigned int* pEntries = <unsigned int*>malloc(sizeof(unsigned int) * numEntries)
        cdef unsigned int i
        returnVal = []
        checkError(fc2GetLUTChannel(self.c, bank, channel, numEntries, pEntries))
        for i in range(numEntries):
            returnVal.append(pEntries[i])
        free(pEntries)
        return returnVal

    def setLUTChannel(self, unsigned int bank, unsigned int channel, list data):
        """camera.setLUTChannel(bank, channel, data) -> None

            data ( [int, int, ...] ): a list containing the integers to write to the LUT channel.
        """

        cdef unsigned int numEntries = <unsigned int>len(data)
        cdef unsigned int* pEntries = NULL
        cdef unsigned int i
        pEntries = <unsigned int*>malloc(numEntries*sizeof(unsigned int))
        for i in range(numEntries):
            pEntries[i] = data[i]
        checkError(fc2SetLUTChannel(self.c, bank, channel, numEntries, pEntries))
        free(pEntries)

    def getMemoryChannel(self):
        """camera.getMemoryChannel() -> currentChannel

        """
        cdef unsigned int currentChan
        checkError(fc2GetMemoryChannel(self.c, &currentChan))
        return currentChan

    def saveToMemoryChannel(self, unsigned int channel):
        """camera.saveToMemoryChannel(channel) -> None

            channel (int): The memory channel to save the settings to.
        """
        checkError(fc2SaveToMemoryChannel(self.c, channel))

    def restoreFromMemoryChannel(self, unsigned int channel):
        """camera.restoreFromMemoryChannel(channel) -> None

            channel (int): memory channel to restore from
        """
        checkError(fc2RestoreFromMemoryChannel(self.c, channel))

    def getMemoryChannelInfo(self):
        """camera.getMemoryChannelInfo() -> numChannels

        """
        cdef unsigned int numChannels
        checkError(fc2GetMemoryChannelInfo(self.c, &numChannels))
        return numChannels

    def writeRegister(self, unsigned int address, unsigned int value):
        """camera.writeRegister(address, value) -> None

            value (int): The value to write to the register.
        """
        checkError(fc2WriteRegister(self.c, address, value))

    def readRegister(self, unsigned int address):
        """camera.readRegister(address) -> value

        """
        cdef unsigned int value
        checkError(fc2ReadRegister(self.c, address, &value))
        return value

    def writeRegisterBlock(self, unsigned long long address, values):
        """camera.writeRegisterBlock(address, values) -> None

            values ( [int, int, ...] ): List containing data to be written.
        """
        cdef unsigned int addressLow, length = <unsigned int>len(values)
        cdef unsigned short addressHigh
        cdef unsigned int* pBuffer = NULL
        cdef unsigned int i
        addressLow = <unsigned int>(address & 0xffffffff)    #0xffffffff is max size for unsigned 32bit int
        addressHigh = <unsigned short>((address >> 32) & 0xffff)    #0xffff is max size for unsigned 16bit short
        pBuffer = <unsigned int*>malloc(length*sizeof(unsigned int))
        for i in range(length):
            pBuffer[i] = values[i]
        checkError(fc2WriteRegisterBlock(self.c, addressHigh, addressLow, pBuffer, length))
        free(pBuffer)

    def readRegisterBlock(self, unsigned long long address, unsigned int length):
        """camera.readRegisterBlock(address, length) -> values

            length (int): Amount of data to read.
        """
        values = []
        cdef unsigned int* pBuffer = <unsigned int*>malloc(length*sizeof(unsigned int))
        cdef unsigned int i, addressLow
        cdef unsigned short addressHigh
        addressLow = <unsigned int>(address & 4294967295)    #4294967295 is max size for unsigned 32bit int
        addressHigh = <unsigned short>((address >> 32) & 65535)    #65535 is max size for unsigned 16bit short
        checkError(fc2ReadRegisterBlock(self.c, addressHigh, addressLow, pBuffer, length))
        for i in range(length):
            values.append(pBuffer[i])
        free(pBuffer)
        return values

    def getCycleTime(self):
        """camera.getCycleTime() -> timeStamp

        """
        cdef fc2TimeStamp timestamp
        checkError(fc2GetCycleTime(self.c, &timestamp))
        returnVal = TimeStamp()
        returnVal.__dict__ = {
            "seconds": timestamp.seconds,
            "microSeconds": timestamp.microSeconds,
            "cycleSeconds": timestamp.cycleSeconds,
            "cycleCount": timestamp.cycleCount,
            "cycleOffset": timestamp.cycleOffset
        }
        return returnVal

    def setCallback(self, func, *extraArgs):
        """camera.setCallback(function, *args) -> None

            arguments: The rest of the arguments are the arguments for the callback function.
        """
        self.callbackArgs = (func, extraArgs)
        checkError(fc2SetCallback(self.c, <fc2ImageEventCallback>&imageCallbackBridge, <void*>self.callbackArgs))

    def unsetCallback(self):
        """camera.unsetCallback() -> None

        If there is currently no stored callback data, this function does nothing.
        """
        checkError(fc2SetCallback(self.c, NULL, NULL))
        self.callbackArgs = ()

    def getPropertyInfo(self, propertyType):
        """camera.getPropertyInfo(propertyType) ->propertyInfo

        """
        cdef fc2PropertyInfo fc2propInfo
        fc2propInfo.type = propertyType
        checkError(fc2GetPropertyInfo(self.c, &fc2propInfo))
        returnVal = PropertyInfo()
        returnVal.__dict__ = {
            "type": fc2propInfo.type,
            "present": fc2propInfo.present,
            "autoSupported": fc2propInfo.autoSupported,
            "manualSupported": fc2propInfo.manualSupported,
            "onOffSupported": fc2propInfo.onOffSupported,
            "onePushSupported": fc2propInfo.onePushSupported,
            "absValSupported": fc2propInfo.absValSupported,
            "readOutSupported": fc2propInfo.readOutSupported,
            "min": fc2propInfo.min,
            "max": fc2propInfo.max,
            "absMin": fc2propInfo.absMin,
            "absMax": fc2propInfo.absMax,
            "units": fc2propInfo.pUnits,
            "unitAbbr": fc2propInfo.pUnitAbbr
        }
        return returnVal

    def getProperty(self, propertyType):
        """camera.getProperty(propertyType) -> property

        """
        cdef fc2Property fc2prop
        fc2prop.type = propertyType
        checkError(fc2GetProperty(self.c, &fc2prop))
        returnVal = Property()
        returnVal.__dict__ = {
            "type": fc2prop.type,
            "present": fc2prop.present,
            "absControl": fc2prop.absControl,
            "onePush": fc2prop.onePush,
            "onOff": fc2prop.onOff,
            "autoManualMode": fc2prop.autoManualMode,
            "valueA": fc2prop.valueA,
            "valueB": fc2prop.valueB,
            "absValue": fc2prop.absValue
        }
        return returnVal

    def setProperty(self, *arg, **kwargs):
        """camera.setProperty(property = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        if not kwargs["type"]:
            raise ValueError("'type' must be defined.")
        cdef fc2Property fc2propTmp, fc2propFinal
        fc2propTmp.type = kwargs["type"]
        checkError(fc2GetProperty(self.c, &fc2propTmp))
        fc2propFinal.type = kwargs.get("type", fc2propTmp.type)
        fc2propFinal.present = kwargs.get("present", fc2propTmp.present)
        fc2propFinal.absControl = kwargs.get("absControl", fc2propTmp.absControl)
        fc2propFinal.onePush = kwargs.get("onePush", fc2propTmp.onePush)
        fc2propFinal.onOff = kwargs.get("onOff", fc2propTmp.onOff)
        fc2propFinal.autoManualMode = kwargs.get("autoManualMode", fc2propTmp.autoManualMode)
        fc2propFinal.valueA = kwargs.get("valueA", fc2propTmp.valueA)
        fc2propFinal.valueB = kwargs.get("valueB", fc2propTmp.valueB)
        fc2propFinal.absValue = kwargs.get("absValue", fc2propTmp.absValue)
        checkError(fc2SetProperty(self.c, &fc2propFinal))

    def getStats(self):
        """camera.getStats() -> cameraStats

        """
        cdef fc2CameraStats camStats
        checkError(fc2GetStats(self.c, &camStats))
        returnVal = CameraStats()
        returnVal.__dict__ = {
            "imageDropped": camStats.imageDropped,
            "imageCorrupt": camStats.imageCorrupt,
            "imageXmitFailed": camStats.imageXmitFailed,
            "imageDriverDropped": camStats.imageDriverDropped,
            "regReadFailed": camStats.regReadFailed,
            "regWriteFailed": camStats.regWriteFailed,
            "portErrors": camStats.portErrors,
            "cameraPowerUp": camStats.cameraPowerUp,
            "cameraVoltages": [camStats.cameraVoltages[i] for i in range(camStats.numVoltages)],
            "cameraCurrents": [camStats.cameraCurrents[i] for i in range(camStats.numCurrents)],
            "temperature": camStats.temperature,
            "timeSinceInitialization": camStats.timeSinceInitialization,
            "timeSinceBusReset": camStats.timeSinceBusReset,
            "timeStamp": camStats.timeStamp,
            "numResendPacketsRequested": camStats.numResendPacketsRequested,
            "numResendPacketsReceived": camStats.numResendPacketsReceived
        }
        return returnVal

    def registerEvent(self, const char* eventName, cbFunc, *args):
        """camera.registerEvent(eventName, function, *arguments) -> None

            *arguments: The rest of the arguments are given to the callback function when called.
        """
        cbData = (cbFunc, args)
        self.evntOpts.EventCallbackFcn = &eventBridge
        self.evntOpts.EventName = eventName
        self.evntOpts.EventUserData = <void*>cbData
        self.evntOpts.EventUserDataSize = sizeof(cbData)
        self.eventArgs[eventName] = cbData
        checkError(fc2RegisterEvent(self.c, &(self.evntOpts)))

    def registerAllEvents(self, cbFunc, *args):
        """camera.registerAllEvents(function, arguments) -> None

        """

        cbData = (cbFunc, args)
        self.evntOpts.EventCallbackFcn = &eventBridge
        self.evntOpts.EventUserData = <void*>cbData
        self.evntOpts.EventUserDataSize = sizeof(cbData)
        self.eventArgs["ALL"] = cbData
        checkError(fc2RegisterAllEvents(self.c, &(self.evntOpts)))

    def deregisterEvent(self, eventName):
        """camera.deregisterEvent(eventName) -> None

        This function currently only works with EventExposureEnd events.
        """
        if eventName == "EventExposureEnd":
            checkError(fc2DeregisterEvent(self.c, &(self.evntOpts)))
        self.eventArgs[eventName] = None

    def deregisterAllEvents(self):
        """camera.deregisterAllEvents() -> None

        De-register all registered events.
        """
        checkError(fc2DeregisterAllEvents(self.c))
        self.eventArgs = {}

    def waitForBufferEvent(self, unsigned int eventNumber):
        """camera.waitForBufferEvent(eventNumber) -> image

            image (PyCapture2.Image): Image object with the latest image buffer.
        """
        img = Image()
        checkError(fc2WaitForBufferEvent(self.c, &img.i, eventNumber))
        return img


cdef class Camera(BaseCamera):
    """A camera object from FlyCapture2. This is specifically for USB cameras."""
    def __cinit__(self):
        self.isConnected = False
        checkError(fc2CreateContext(&(self.c)))

    def getVideoModeAndFrameRateInfo(self, vMode, fRate):
        """camera.getVideoModeAndFrameRateInfo(videoMode, frameRate) -> supported

        """
        cdef bint supported
        checkError(fc2GetVideoModeAndFrameRateInfo(self.c, vMode, fRate, &supported))
        return supported

    def getVideoModeAndFrameRate(self):
        cdef fc2VideoMode vMode
        cdef fc2FrameRate fRate
        checkError(fc2GetVideoModeAndFrameRate(self.c, &vMode, &fRate))
        return vMode, fRate

    def setVideoModeAndFrameRate(self, vMode, fRate):
        """camera.setVideoModeAndFrameRate(videoMode, frameRate) -> None

            frameRate (int): Frame rate to set. Use PyCapture2.FRAME_RATE to set correctly.
        """
        checkError(fc2SetVideoModeAndFrameRate(self.c, vMode, fRate))

    def getFormat7Info(self, fmt7Mode):
        cdef fc2Format7Info fmt7Info
        cdef bint supported
        fmt7Info.mode = fmt7Mode
        checkError(fc2GetFormat7Info(self.c, &fmt7Info, &supported))
        returnVal = Format7Info()
        returnVal.__dict__ = {
            "mode": fmt7Info.mode,
            "maxWidth": fmt7Info.maxWidth,
            "maxHeight": fmt7Info.maxHeight,
            "offsetHStepSize": fmt7Info.offsetHStepSize,
            "offsetVStepSize": fmt7Info.offsetVStepSize,
            "imageHStepSize": fmt7Info.imageHStepSize,
            "imageVStepSize": fmt7Info.imageVStepSize,
            "pixelFormatBitField": fmt7Info.pixelFormatBitField,
            "vendorPixelFormatBitField": fmt7Info.vendorPixelFormatBitField,
            "packetSize": fmt7Info.packetSize,
            "minPacketSize": fmt7Info.minPacketSize,
            "maxPacketSize": fmt7Info.maxPacketSize,
            "percentage": fmt7Info.percentage
        }
        return returnVal, supported

    def getFormat7Configuration(self):
        cdef fc2Format7ImageSettings fmt7Set
        cdef unsigned int packetSize
        cdef float percentage
        checkError(fc2GetFormat7Configuration(self.c, &fmt7Set, &packetSize, &percentage))
        returnVal = Format7ImageSettings()
        returnVal.__dict__ = {
            "mode": fmt7Set.mode,
            "offsetX": fmt7Set.offsetX,
            "offsetY": fmt7Set.offsetY,
            "width": fmt7Set.width,
            "height": fmt7Set.height,
            "pixelFormat": fmt7Set.pixelFormat,
        }
        return returnVal, packetSize, percentage

    def validateFormat7Settings(self, *arg, **kwargs):
        """camera.validateFormat7Settings(imageSettings = None, **kwargs) ->
        packetInfo, isValid

                isValid (bool): Whether the settings are valid.
        """
        if arg:
            kwargs = arg[0].__dict__
        cdef fc2Format7PacketInfo fmt7PInfo
        cdef fc2Format7ImageSettings fmt7SetTmp, fmt7SetFinal
        cdef bint isValid
        cdef unsigned int packetSizeFinal
        cdef float percentageFinal
        fmt7SetFinal.mode = kwargs.get("mode", fmt7SetTmp.mode)
        fmt7SetFinal.offsetX = kwargs.get("offsetX", fmt7SetTmp.offsetX)
        fmt7SetFinal.offsetY = kwargs.get("offsetY", fmt7SetTmp.offsetY)
        fmt7SetFinal.width = kwargs.get("width", fmt7SetTmp.width)
        fmt7SetFinal.height = kwargs.get("height", fmt7SetTmp.height)
        fmt7SetFinal.pixelFormat = kwargs.get("pixelFormat", fmt7SetTmp.pixelFormat)
        checkError(fc2ValidateFormat7Settings(self.c, &fmt7SetFinal, &isValid, &fmt7PInfo))

        returnVal = Format7PacketInfo()
        returnVal.__dict__ = {
            "recommendedBytesPerPacket": fmt7PInfo.recommendedBytesPerPacket,
            "maxBytesPerPacket": fmt7PInfo.maxBytesPerPacket,
            "unitBytesPerPacket": fmt7PInfo.unitBytesPerPacket
        }
        return returnVal, isValid

    def setFormat7Configuration(self, float percentSpeed, *arg, **kwargs):
        """camera.setFormat7Configuration(percentSpeed, imageSettings = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        cdef fc2Format7ImageSettings fmt7SetTmp, fmt7SetFinal
        cdef bint isValid
        cdef fc2Format7PacketInfo fmt7PInfo
        fmt7SetFinal.mode = kwargs.get("mode", fmt7SetTmp.mode)
        fmt7SetFinal.offsetX = kwargs.get("offsetX", fmt7SetTmp.offsetX)
        fmt7SetFinal.offsetY = kwargs.get("offsetY", fmt7SetTmp.offsetY)
        fmt7SetFinal.width = kwargs.get("width", fmt7SetTmp.width)
        fmt7SetFinal.height = kwargs.get("height", fmt7SetTmp.height)
        fmt7SetFinal.pixelFormat = kwargs.get("pixelFormat", fmt7SetTmp.pixelFormat)
        checkError(fc2SetFormat7Configuration(self.c, &fmt7SetFinal, percentSpeed))

    def setFormat7ConfigurationPacket(self, unsigned int packetSize, *arg, **kwargs):
        """camera.setFormat7Configuration(packetSize, imageSettings = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        cdef fc2Format7ImageSettings fmt7SetTmp, fmt7SetFinal
        cdef bint isValid
        cdef fc2Format7PacketInfo fmt7PInfo
        cdef unsigned int packetSizeFinal
        fmt7SetFinal.mode = kwargs.get("mode", fmt7SetTmp.mode)
        fmt7SetFinal.offsetX = kwargs.get("offsetX", fmt7SetTmp.offsetX)
        fmt7SetFinal.offsetY = kwargs.get("offsetY", fmt7SetTmp.offsetY)
        fmt7SetFinal.width = kwargs.get("width", fmt7SetTmp.width)
        fmt7SetFinal.height = kwargs.get("height", fmt7SetTmp.height)
        fmt7SetFinal.pixelFormat = kwargs.get("pixelFormat", fmt7SetTmp.pixelFormat)
        packetSizeFinal = packetSize
        checkError(fc2SetFormat7ConfigurationPacket(self.c, &fmt7SetFinal, packetSizeFinal))


cdef class GigECamera(BaseCamera):
    """A camera object from FlyCapture2. This is specifically for GigE and ethernet cameras."""
    def __cinit__(self):
        self.isConnected = False
        checkError(fc2CreateGigEContext(&(self.c)))

    def writeGVCPRegister(self, unsigned int address, unsigned int value):
        """camera.writeGVCPRegister(address, value) -> None

            value (int): The value to be written.
        """
        checkError(fc2WriteGVCPRegister(self.c, address, value))

    def readGVCPRegister(self, unsigned int address):
        """camera.readGVCPRegister(address) -> value

        """
        cdef unsigned int value
        checkError(fc2ReadGVCPRegister(self.c, address, &value))
        return value

    def writeGVCPRegisterBlock(self, unsigned int address, list values):
        """camera.writeGVCPRegisterBlock(address, values) -> None

            values ( [int, int, ...] ): The list of integer values to write to register.
        """
        cdef unsigned int length = <unsigned int>len(values)
        cdef unsigned int* pBuffer = NULL
        cdef unsigned int i
        pBuffer = <unsigned int*>malloc(length*sizeof(unsigned int))
        for i in range(length):
            pBuffer[i] = values[i]
        checkError(fc2WriteGVCPRegisterBlock(self.c, address, pBuffer, length))
        free(pBuffer)

    def readGVCPRegisterBlock(self, unsigned int address, unsigned int length):
        """camera.readGVCPRegisterBlock(address, length) -> values

            values ( [int, int, ...] ): The list of values read from the registers.
        """
        values = []
        cdef unsigned int* pBuffer = <unsigned int*>malloc(length*sizeof(unsigned int))
        cdef unsigned int i
        checkError(fc2ReadGVCPRegisterBlock(self.c, address, pBuffer, length))
        for i in range(length):
            values.append(pBuffer[i])
        free(pBuffer)
        return values

    def writeGVCPMemory(self, unsigned int address, list values):
        """camera.writeGVCPMemory(address, values) -> None

            values ( [int, int, ...] ): The list of integer values to write to the memory.
        """
        cdef unsigned int length = <unsigned int>len(values)
        cdef unsigned char* pBuffer = <unsigned char*>malloc(length*sizeof(unsigned char))
        cdef unsigned int i
        for i in range(length):
            pBuffer[i] = values[i]
        checkError(fc2WriteGVCPMemory(self.c, address, pBuffer, length))
        free(pBuffer)

    def readGVCPMemory(self, unsigned int address, unsigned int length):
        """camera.readGVCPMemory(address, length) -> values

            values ( [int, int, ...] ): The list of integers containing read data.
        """
        values = []
        cdef unsigned char* pBuffer = <unsigned char*>malloc(length*sizeof(unsigned char))
        cdef unsigned int i
        checkError(fc2ReadGVCPMemory(self.c, address, pBuffer, length))
        for i in range(length):
            values.append(pBuffer[i])
        return values

    def getGigEProperty(self, propertyType):
        """camera.getGigEProperty(propertyType) -> gigEProperty

        """
        cdef fc2GigEProperty gigEProp
        gigEProp.propType = propertyType
        checkError(fc2GetGigEProperty(self.c, &gigEProp))
        returnVal = GigEProperty()
        returnVal.__dict__ = {
            "propType": gigEProp.propType,
            "isReadable": gigEProp.isReadable,
            "isWritable": gigEProp.isWritable,
            "min": gigEProp.min,
            "max": gigEProp.max,
            "value": gigEProp.value
        }
        return returnVal

    def setGigEProperty(self, *arg, **kwargs):
        """camera.setGigEProperty(gigEProperty = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        if not kwargs["propType"]:
            raise ValueError("'propType' must be defined.")
        cdef fc2GigEProperty gigEPropTmp, gigEPropFinal
        gigEPropTmp.propType = kwargs["propType"]
        checkError(fc2GetGigEProperty(self.c, &gigEPropTmp))
        gigEPropFinal.propType = kwargs.get("propType", gigEPropTmp.propType)
        gigEPropFinal.isReadable = kwargs.get("isReadable", gigEPropTmp.isReadable)
        gigEPropFinal.isWritable = kwargs.get("isWritable", gigEPropTmp.isWritable)
        gigEPropFinal.min = kwargs.get("min", gigEPropTmp.min)
        gigEPropFinal.max = kwargs.get("max", gigEPropTmp.max)
        gigEPropFinal.value = kwargs.get("value", gigEPropTmp.value)
        checkError(fc2SetGigEProperty(self.c, &gigEPropFinal))

    def discoverGigEPacketSize(self):
        """camera.discoverGigEPacketSize() -> packetSize

        """
        cdef unsigned int packetSize
        checkError(fc2DiscoverGigEPacketSize(self.c, &packetSize))
        return packetSize

    def queryGigEImagingMode(self, mode):
        """camera.queryGigEImagingMode(mode) -> isSupported

        """
        cdef bint isSupported
        checkError(fc2QueryGigEImagingMode(self.c, mode, &isSupported))
        return isSupported

    def getGigEImagingMode(self):
        """camera.getGigEImagingMode() -> mode

        """
        cdef fc2Mode mode
        checkError(fc2GetGigEImagingMode(self.c, &mode))
        return mode

    def setGigEImagingMode(self, mode):
        """camera.setGigEImagingMode(mode) -> None

            mode (int): Mode to set the camera to. Use PyCapture2.MODE to set correctly.
        """
        checkError(fc2SetGigEImagingMode(self.c, mode))

    def getGigEImageSettingsInfo(self):
        """camera.getGigEImageSettingsInfo() -> gigEImageSettingsInfo

        """
        cdef fc2GigEImageSettingsInfo gigESetInfo
        checkError(fc2GetGigEImageSettingsInfo(self.c, &gigESetInfo))
        returnVal = GigEImageSettingsInfo()
        returnVal.__dict__ = {
            "maxWidth": gigESetInfo.maxWidth,
            "maxHeight": gigESetInfo.maxHeight,
            "offsetHStepSize": gigESetInfo.offsetHStepSize,
            "offsetVStepSize": gigESetInfo.offsetVStepSize,
            "imageHStepSize": gigESetInfo.imageHStepSize,
            "imageVStepSize": gigESetInfo.imageVStepSize,
            "pixelFormatBitField": gigESetInfo.pixelFormatBitField,
            "vendorPixelFormatBitField": gigESetInfo.vendorPixelFormatBitField
        }
        return returnVal

    def getGigEImageSettings(self):
        """camera.getGigEImageSettings() -> gigEImageSettings

        """
        cdef fc2GigEImageSettings gigESet
        checkError(fc2GetGigEImageSettings(self.c, &gigESet))
        returnVal = GigEImageSettings()
        returnVal.__dict__ = {
            "offsetX": gigESet.offsetX,
            "offsetY": gigESet.offsetY,
            "width": gigESet.width,
            "height": gigESet.height,
            "pixelFormat": gigESet.pixelFormat,
        }
        return returnVal

    def setGigEImageSettings(self, *arg, **kwargs):
        """camera.setGigEImageSettings(gigEImageSettings = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        cdef fc2GigEImageSettings gigESetTmp, gigESetFinal
        checkError(fc2GetGigEImageSettings(self.c, &gigESetTmp))
        gigESetFinal.offsetX = kwargs.get("offsetX", gigESetTmp.offsetX)
        gigESetFinal.offsetY = kwargs.get("offsetY", gigESetTmp.offsetY)
        gigESetFinal.width = kwargs.get("width", gigESetTmp.width)
        gigESetFinal.height = kwargs.get("height", gigESetTmp.height)
        gigESetFinal.pixelFormat = kwargs.get("pixelFormat", gigESetTmp.pixelFormat)

        checkError(fc2SetGigEImageSettings(self.c, &gigESetFinal))

    def getGigEImageBinningSettings(self):
        """camera.getGigEImageBinningSettings() -> (horzValue, vertValue)

        """
        cdef unsigned int horzValue, vertValue
        checkError(fc2GetGigEImageBinningSettings(self.c, &horzValue, &vertValue))
        return horzValue, vertValue

    def setGigEImageBinningSettings(self, tuple settings):
        """camera.setGigEImageBinningSettings( (horzValue, vertValue) ) -> None

            vertValue (int): Vertical binning value to set.
        """
        checkError(fc2SetGigEImageBinningSettings(self.c, settings[0], settings[1]))

    def getNumStreamChannels(self):
        """camera.getNumStreamChannels() -> numChannels

        """
        cdef unsigned int numChannels
        checkError(fc2GetNumStreamChannels(self.c, &numChannels))
        return numChannels

    def getGigEStreamChannelInfo(self, unsigned int channel):
        """camera.getGigEStreamChannelInfo(channel) -> gigEStreamChannel

        """
        cdef fc2GigEStreamChannel channelInfo
        checkError(fc2GetGigEStreamChannelInfo(self.c, channel, &channelInfo))
        returnVal = GigEStreamChannel()
        returnVal.__dict__ = {
            "networkInterfaceIndex": channelInfo.networkInterfaceIndex,
            "hostPort": channelInfo.hostPort,
            "doNotFragment": channelInfo.doNotFragment,
            "packetSize": channelInfo.packetSize,
            "interPacketDelay": channelInfo.interPacketDelay,
            "destinationIpAddress": (channelInfo.destinationIpAddress.octets[0],
                                     channelInfo.destinationIpAddress.octets[1],
                                     channelInfo.destinationIpAddress.octets[2],
                                     channelInfo.destinationIpAddress.octets[3]),
            "sourcePort": channelInfo.sourcePort
        }
        return returnVal

    def getGigEConfig(self):
        """camera.getGigEConfig() -> gigEConfig

        """
        cdef fc2GigEConfig config
        checkError(fc2GetGigEConfig(self.c, &config))
        returnVal = GigEConfig()
        returnVal.__dict__ = {
            "enablePacketResend": config.enablePacketResend,
            "registerTimeoutRetries": config.registerTimeoutRetries,
            "registerTimeout": config.registerTimeout
        }
        return returnVal

    def setGigEConfig(self, *arg, **kwargs):
        """camera.setGigEConfig(gigEConfig = None, **kwargs) -> None

            kwargs ( {...} ): The second method of calling: specify the properties to change as keywords.
        """
        if arg:
            kwargs = arg[0].__dict__
        cdef fc2GigEConfig configTmp, configFinal
        checkError(fc2GetGigEConfig(self.c, &configTmp))
        configFinal.enablePacketResend = kwargs.get("enablePacketResend", configTmp.enablePacketResend)
        configFinal.registerTimeoutRetries = kwargs.get("registerTimeoutRetries", configTmp.registerTimeoutRetries)
        configFinal.registerTimeout = kwargs.get("registerTimeout", configTmp.registerTimeout)
        checkError(fc2SetGigEConfig(self.c, &configFinal))


cdef class Image:
    """An image from the FlyCapture 2 library."""
    cdef fc2Image i
    def __cinit__(self):
        checkError(fc2CreateImage(&(self.i)))
    def __dealloc__(self):
        fc2DestroyImage(&(self.i))
    def save(self, const char* filename, format, option = None):
        """Image.save(fileName, format, option = None) -> None

                    BMPOption
        """
        cdef fc2Image fc2img = getImage(self)
        cdef void* optPtr
        if not option:
            checkError(fc2SaveImage(&fc2img, filename, format))
            return

        elif type(option) == "PNGOption":
            optPtr = malloc(sizeof(fc2PNGOption))
            (<fc2PNGOption*>optPtr)[0] = {option.interlaced, option.compressionLevel}
        elif type(option) == "PPMOption":
            optPtr = malloc(sizeof(fc2PPMOption))
            (<fc2PPMOption*>optPtr)[0] = {option.binaryFile}
        elif type(option) == "PGMOption":
            optPtr = malloc(sizeof(fc2PGMOption))
            (<fc2PGMOption*>optPtr)[0] = {option.binaryFile}
        elif type(option) == "TIFFOption":
            optPtr = malloc(sizeof(fc2TIFFOption))
            (<fc2TIFFOption*>optPtr)[0] = {option.compression}
        elif type(option) == "JPEGOption":
            optPtr = malloc(sizeof(fc2JPEGOption))
            (<fc2JPEGOption*>optPtr)[0] = {option.progressive, option.quality}
        elif type(option) == "JPG2Option":
            optPtr = malloc(sizeof(fc2JPG2Option))
            (<fc2JPG2Option*>optPtr)[0] = {option.quality}
        elif type(option) == "BMPOption":
            optPtr = malloc(sizeof(fc2BMPOption))
            (<fc2BMPOption*>optPtr)[0] = {option.indexedColor_8bit}
        checkError(fc2SaveImageWithOption(&(self.i), filename, format, optPtr))
        free(optPtr)

    def getTimeStamp(self):
        """Image.getTimeStamp() -> timeStamp

            timeStamp (PyCapture2.TimeStamp): TimeStamp object containing image capture time.
        """
        cdef fc2Image img = getImage(self)
        cdef fc2TimeStamp timestamp
        timestamp = fc2GetImageTimeStamp(&img)
        returnVal = TimeStamp()
        returnVal.__dict__ = {
            "seconds": timestamp.seconds,
            "microSeconds": timestamp.microSeconds,
            "cycleSeconds": timestamp.cycleSeconds,
            "cycleCount": timestamp.cycleCount,
            "cycleOffset": timestamp.cycleOffset
        }
        return returnVal

    def convert(self, format):
        """Image.convert(format) -> image

            image (PyCapture2.Image): An image object in the specified format.
        """
        img2 = Image()
        cdef fc2Image fc2self, fc2img2
        fc2self = getImage(self)
        fc2img2 = getImage(img2)
        checkError(fc2ConvertImageTo(format, &(self.i), &fc2img2))
        return img2

    def getPixelFormat(self):
        """Image.getPixelFormat() -> format

            format (int): Pixel format of the image. Use PyCapture2.PIXEL_FORMAT to read correctly.
        """
        return self.i.format

    def getBayerTileFormat(self):
        """Image.getBayerTileFormat() -> bayerFormat

            bayerFormat (int): Bayer tile format of the image. Use BAYER_FORMAT to read correctly.
        """
        return self.i.bayerFormat

    def getRows(self):
        """Image.getRows() -> numRows

            numRows (int): The number of rows in the image.
        """
        return self.i.rows

    def getCols(self):
        """Image.getCols() -> numCols

            numCols (int): The number of cols in the image.
        """
        return self.i.cols

    def getStride(self):
        """Image.getStride() -> stride

            stride (int): The stride of the image.
        """
        return self.i.stride

    def getDataSize(self):
        """Image.getDataSize() -> dataSize

            dataSize (int): The maximum possible length of image data.
        """
        return self.i.dataSize

    def getReceivedDataSize(self):
        """Image.getReceivedDataSize() -> dataSize

            dataSize (int): The length of the image data.
        """
        return self.i.receivedDataSize

    def getData(self):
        """Image.getData() -> imageData

        """
        cdef unsigned int i
        returnVal = []
        for i in range(self.i.dataSize):
            returnVal.append(self.i.pData[i])
        return returnVal

    def __bytes__(self):
        return <bytes> self.i.pData[:self.i.dataSize]

    def __array__(self):
        cdef np.ndarray r
        cdef np.npy_intp shape[3]
        cdef np.npy_intp stride[3]
        cdef np.dtype dtype

        fmt = self.i.format
        ndim = 2

        if fmt == PIXEL_FORMAT.MONO8 or fmt == PIXEL_FORMAT.RAW8:
            dtype = np.dtype("uint8")
            stride[1] = 1
        elif fmt == PIXEL_FORMAT.MONO16 or fmt == PIXEL_FORMAT.RAW16:
            dtype = np.dtype("uint16")
            stride[1] = 2
        elif fmt == PIXEL_FORMAT.RGB8 or fmt == PIXEL_FORMAT.YUV8_444:
            dtype = np.dtype("uint8")
            ndim = 3
            stride[1] = 3
            stride[2] = 1
            shape[2] = 3
        elif fmt == PIXEL_FORMAT.YUV8_422:
            dtype = np.dtype("uint8")
            ndim = 3
            stride[1] = 2
            stride[2] = 1
            shape[2] = 2
        else:
            dtype = np.dtype("uint8")
            stride[1] = self.i.stride / self.i.cols

        Py_INCREF(dtype)
        shape[0] = self.i.rows
        shape[1] = self.i.cols
        stride[0] = self.i.stride

        #assert stride[0] == stride[1]*shape[1]
        #assert shape[0]*shape[1]*stride[1] == self.i.dataSize

        r = PyArray_NewFromDescr(np.ndarray, dtype,
                ndim, shape, stride,
                self.i.pData, np.NPY_DEFAULT, None)

        r.base = <PyObject *>self

        Py_INCREF(self)
        return r

cdef class AVIRecorder:
    """A class from FlyCapture 2 used to create animated AVI and MP4 files."""
    cdef fc2AVIContext c

    def __cinit__(self):
        checkError(fc2CreateAVI(&(self.c)))

    def __dealloc__(self):
        fc2DestroyAVI(self.c)

    def AVIOpen(self, const char* filename, float framerate):
        """AVIRecorder.AVIOpen(filename, framerate) -> None

        """
        cdef fc2AVIOption AVIOpt
        AVIOpt.frameRate = framerate
        checkError(fc2AVIOpen(self.c, filename, &AVIOpt))

    def MJPGOpen(self, const char* filename, float framerate, unsigned int quality):
        """AVIRecorder.MJPGOpen(filename, framerate, quality) -> None

        """
        cdef fc2MJPGOption MJPGOpt
        MJPGOpt.frameRate = framerate
        MJPGOpt.quality = quality
        checkError(fc2MJPGOpen(self.c, filename, &MJPGOpt))

    def H264Open(self, const char* filename, float framerate, unsigned int width, unsigned int height, unsigned int bitrate):
        """AVIRecorder.H264Open(filename, framerate, width, height, bitrate)

        """
        cdef fc2H264Option H264Opt
        H264Opt.frameRate = framerate
        H264Opt.width = width
        H264Opt.height = height
        H264Opt.bitrate = bitrate
        checkError(fc2H264Open(self.c, filename, &H264Opt))

    def close(self):
        """AVIRecorder.close() -> None

        Close the AVI/MP4 file and save it to disk.
        """
        checkError(fc2AVIClose(self.c))

    def append(self, img):
        """AVIRecorder.append(image) -> None

        """
        cdef fc2Image fc2img
        fc2img = getImage(img)
        checkError(fc2AVIAppend(self.c, &fc2img))


cdef class ImageStatistics:
    cdef fc2ImageStatisticsContext c

    def __cinit__(self):
        checkError(fc2CreateImageStatistics(&(self.c)))

    def __dealloc__(self):
        checkError(fc2DestroyImageStatistics(self.c))

    def enableAllChannels(self):
        """ImageStatistics.enableAllChannels() -> None

        Enable all channels for image statistics analysis.
        """
        checkError(fc2ImageStatisticsEnableAll(self.c))

    def disableAllChannels(self):
        """ImageStatistics.disableAllChannels() -> None

        Disable all channels for image statistics analysis.
        """
        checkError(fc2ImageStatisticsDisableAll(self.c))

    def enableGreyChannel(self):
        """ImageStatistics.enableGreyChannel() -> None

        Enable only the grey channel for image statistics analysis.
        """
        checkError(fc2ImageStatisticsEnableGreyOnly(self.c))

    def enableRGBChannel(self):
        """ImageStatistics.enableRGBChannel() -> None

        Enable only the red, green, and blue channels for image statistics analysis.
        """
        checkError(fc2ImageStatisticsEnableRGBOnly(self.c))

    def enableHSLChannel(self):
        """ImageStatistics.enableHSLChannel() -> None

        Enable only the hue, saturation, and lightness channels for image statistics analysis.
        """
        checkError(fc2ImageStatisticsEnableHSLOnly(self.c))

    def getChannelStatus(self, channel):
        """ImageStatistics.getChannelStatus(channel) -> enabled

        """
        cdef bint enabled
        checkError(fc2GetChannelStatus(self.c, channel, &enabled))
        return enabled

    def setChannelStatus(self, channel, bint onOff):
        """ImageStatistics.setChannelStatus(channel, enabled) -> None

            enabled (bool): Whether the channel will be enabled or disabled.
        """
        checkError(fc2SetChannelStatus(self.c, channel, onOff))

    def getRange(self, channel):
        """ImageStatistics.getRange(channel) -> (min, max)

        """
        cdef unsigned int min, max
        checkError(fc2GetChannelRange(self.c, channel, &min, &max))
        return min, max

    def getPixelValueRange(self, channel):
        """ImageStatistics.getPixelValueRange(channel) -> (min, max)

        """
        cdef unsigned int min, max
        checkError(fc2GetChannelPixelValueRange(self.c, channel, &min, &max))
        return min, max

    def getNumPixelValues(self, channel):
        """ImageStatistics.getNumPixelValues(channel) -> numValues

        """
        cdef unsigned int numValues
        checkError(fc2GetChannelNumPixelValues(self.c, channel, &numValues))
        return numValues

    def getMean(self, channel):
        """ImageStatistics.getMean(channel) -> mean

        """
        cdef float meanValue
        checkError(fc2GetChannelMean(self.c, channel, &meanValue))
        return meanValue

    def getHistogram(self, channel):
        cdef int* histo
        cdef unsigned int i, min, max
        listo = []
        checkError(fc2GetChannelRange(self.c, channel, &min, &max))
        checkError(fc2GetChannelHistogram(self.c, channel, &histo))
        for i in range(max-min):
            listo.append(histo[i])
        return listo

    def getStatistics(self, channel):
        cdef unsigned int rangeMin
        cdef unsigned int rangeMax
        cdef unsigned int pixValMin
        cdef unsigned int pixValMax
        cdef unsigned int numPixVals
        cdef float pixValMean
        cdef int* histo
        cdef unsigned int i
        checkError(fc2GetImageStatistics(self.c, channel, &rangeMin, &rangeMax,
                                        &pixValMin, &pixValMax, &numPixVals,
                                        &pixValMean, &histo))
        listo = []
        for i in range(rangeMax-rangeMin):
            listo.append(histo[i])
        return (rangeMin, rangeMax), (pixValMin, pixValMax), numPixVals, pixValMean, listo

    def calculateStatistics(self, img):
        """ImageStatistics.calculateStatistics(image) -> None

            image (PyCapture2.Image): The image to calculate statistics from.
        """
        cdef fc2Image fc2img = getImage(img)
        checkError(fc2CalculateImageStatistics(&fc2img, &(self.c)))


cdef class BusManager:
    cdef fc2Context c
    cdef fc2CallbackHandle eventArrival
    cdef fc2CallbackHandle eventRemoval
    cdef fc2CallbackHandle allEvents
    cdef dict callbackArgs

    def __cinit__(self):
        checkError(fc2CreateContext(&(self.c)))

    def __init__(self):
        """BusManager() -> BusManager object

        Initialize a bus manager class.
        """
        self.callbackArgs = {}

    def __dealloc__(self):
        fc2DestroyContext(self.c)

    def fireBusReset(self, tuple id):
        """BusManager.fireBusReset(guid) -> None

        """
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2FireBusReset(self.c, &uID))

    def getNumOfCameras(self):
        """BusManager.getNumOfCameras() -> count

        """
        cdef unsigned int count
        checkError(fc2GetNumOfCameras(self.c, &count))
        return count

    def getCameraFromIPAddress(self, tuple ip):
        cdef fc2PGRGuid uID
        cdef fc2IPAddress ipAddr
        ipAddr.octets[0], ipAddr.octets[1], ipAddr.octets[2], ipAddr.octets[3] = ip
        checkError(fc2GetCameraFromIPAddress(self.c, ipAddr, &uID))
        return uID.value[0], uID.value[1], uID.value[2], uID.value[3]

    def getCameraFromIndex(self, unsigned int index):
        """BusManager.getCameraFromIndex(index) -> guid

        """
        cdef fc2PGRGuid camID
        checkError(fc2GetCameraFromIndex(self.c, index, &camID))
        return (camID.value[0], camID.value[1], camID.value[2], camID.value[3])

    def getInterfaceTypeFromGuid(self, tuple id):
        cdef fc2InterfaceType iType
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2GetInterfaceTypeFromGuid(self.c, &uID, &iType))
        return iType

    def getCameraSerialNumberFromIndex(self, unsigned int index):
        """BusManager.getCameraSerialNumberFromIndex(index) -> serialNumber

        """
        cdef unsigned int serialNum
        checkError(fc2GetCameraSerialNumberFromIndex(self.c, index, &serialNum))
        return serialNum

    def getCameraFromSerialNumber(self, unsigned int serialNum):
        """BusManager.getCameraFromIndex(serial number) -> guid

        """
        cdef fc2PGRGuid uID
        checkError(fc2GetCameraFromSerialNumber(self.c, serialNum, &uID))
        return (uID.value[0], uID.value[1], uID.value[2], uID.value[3])

    def getNumOfDevices(self):
        """BusManager.getNumOfDevices() -> numDevices

        """
        cdef unsigned int numDevices
        checkError(fc2GetNumOfDevices(self.c, &numDevices))
        return numDevices

    def getDeviceFromIndex(self, unsigned int index):
        """BusManager.getDeviceFromIndex(index) -> guid

        """
        cdef fc2PGRGuid uID
        checkError(fc2GetDeviceFromIndex(self.c, index, &uID))
        return (uID.value[0], uID.value[1], uID.value[2], uID.value[3])

    def readPhyRegister(self, tuple id, unsigned int page, unsigned int port, unsigned int address):
        cdef unsigned int value
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2ReadPhyRegister(self.c, uID, page, port, address, &value))
        return value

    def writePhyRegister(self, tuple id, unsigned int page, unsigned int port, unsigned int address, unsigned int value):
        """BusManager.writePhyRegister(guid, page, port, address, value) -> None

        """
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2WritePhyRegister(self.c, uID, page, port, address, value))
    def getUsbLinkInfo(self, id):
        cdef unsigned int value
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2GetUsbLinkInfo(self.c, uID, &value))
        return value

    def getUsbPortStatus(self, id):
        cdef unsigned int status
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2GetUsbPortStatus(self.c, uID, &status))
        return status

    def rescanBus(self):
        """BusManager.rescanBus() -> None

        This does not trigger a bus reset. However, any current connections to a Camera object will be invalidated.
        """
        checkError(fc2RescanBus(self.c))

    def isCameraControllable(self, id):
        cdef bint isControllable
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
        checkError(fc2IsCameraControlable(self.c, &uID, &isControllable))
        return isControllable

    def forceIPAddressToCamera(self, tuple mac, tuple ip, tuple subnet, tuple gateway):
        """BusManager.forceIPAddressToCamera(macAddress, ipAddress, subnetMask, gateway) -> None

            gateway (int, int, int, int): The default gateway to force to the camera.
        """
        cdef fc2IPAddress ipAddr = {0,0,0,0}, subnetMask = {0,0,0,0}, defaultGateway = {0,0,0,0}
        cdef fc2MACAddress macAddr = {0,0,0,0,0,0}
        macAddr.value[0], macAddr.value[1], macAddr.value[2], macAddr.value[3], macAddr.value[4], macAddr.value[5] = mac
        ipAddr.value[0], ipAddr.value[1], ipAddr.value[2], ipAddr.value[3] = ip
        subnetMask.value[0], subnetMask.value[1], subnetMask.value[2], subnetMask.value[3] = subnet
        defaultGateway.value[0], defaultGateway.value[1], defaultGateway.value[2], defaultGateway.value[3] = gateway
        checkError(fc2ForceIPAddressToCamera(self.c, macAddr, ipAddr, subnetMask, defaultGateway))

    def forceAllIPAddressesAutomatically(self):
        """BusManager.forceAllIPAddressesAutomatically() -> None

        This is useful in situations where GigE Vision cameras are using Persistent IP addresses and the application's subnet is different from the devices.
        """
        checkError(fc2ForceAllIPAddressesAutomatically())

    def discoverGigECameras(self, unsigned int numCams = 10):
        """BusManager.discoverGigECameras(numCams = 10) -> cameraInfos

        """
        cdef fc2CameraInfo* gigECams
        gigECams = <fc2CameraInfo*> malloc(numCams * sizeof(fc2CameraInfo))
        cdef unsigned int i
        checkError(fc2DiscoverGigECameras(self.c, gigECams, &numCams))
        returnVal = [CameraInfo() for i in range(numCams)]
        for i in range(numCams):
            returnVal[i].__dict__ = {
                "serialNumber": gigECams[i].serialNumber,
                "interfaceType": gigECams[i].interfaceType,
                "driverType": gigECams[i].driverType,
                "isColorCamera": gigECams[i].isColorCamera,
                "modelName": gigECams[i].modelName,
                "vendorName": gigECams[i].vendorName,
                "sensorInfo": gigECams[i].sensorInfo,
                "sensorResolution": gigECams[i].sensorResolution,
                "driverName": gigECams[i].driverName,
                "firmwareVersion": gigECams[i].firmwareVersion,
                "firmwareBuildTime": gigECams[i].firmwareBuildTime,
                "maximumBusSpeed": gigECams[i].maximumBusSpeed,
                "pcieBusSpeed": gigECams[i].pcieBusSpeed,
                "bayerTileFormat": gigECams[i].bayerTileFormat,
                "busNumber": gigECams[i].busNumber,
                "nodeNumber": gigECams[i].nodeNumber,
                "iidcVer": gigECams[i].iidcVer,
                "configROM": {
                    "nodeVendorId": gigECams[i].configROM.nodeVendorId,
                    "chipIdHi": gigECams[i].configROM.chipIdHi,
                    "chipIdLo": gigECams[i].configROM.chipIdLo,
                    "unitSpecId": gigECams[i].configROM.unitSpecId,
                    "unitSWVer": gigECams[i].configROM.unitSWVer,
                    "unitSubSWVer": gigECams[i].configROM.unitSubSWVer,
                    "vendorUniqueInfo_0": gigECams[i].configROM.vendorUniqueInfo_0,
                    "vendorUniqueInfo_1": gigECams[i].configROM.vendorUniqueInfo_1,
                    "vendorUniqueInfo_2": gigECams[i].configROM.vendorUniqueInfo_2,
                    "vendorUniqueInfo_3": gigECams[i].configROM.vendorUniqueInfo_3,
                    "pszKeyword": gigECams[i].configROM.pszKeyword
                },
                "gigEMajorVersion": gigECams[i].gigEMajorVersion,
                "gigEMinorVersion": gigECams[i].gigEMinorVersion,
                "userDefinedName": gigECams[i].userDefinedName,
                "xmlURL1": gigECams[i].xmlURL1,
                "xmlURL2": gigECams[i].xmlURL2,
                "macAddress": (gigECams[i].macAddress.octets[0], gigECams[i].macAddress.octets[1], gigECams[i].macAddress.octets[2], gigECams[i].macAddress.octets[3], gigECams[i].macAddress.octets[4], gigECams[i].macAddress.octets[5]),
                "ipAddress": (gigECams[i].ipAddress.octets[0], gigECams[i].ipAddress.octets[1], gigECams[i].ipAddress.octets[2], gigECams[i].ipAddress.octets[3]),
                "subnetMask": (gigECams[i].subnetMask.octets[0], gigECams[i].subnetMask.octets[1], gigECams[i].subnetMask.octets[2], gigECams[i].subnetMask.octets[3]),
                "defaultGateway": (gigECams[i].defaultGateway.octets[0], gigECams[i].defaultGateway.octets[1], gigECams[i].defaultGateway.octets[2], gigECams[i].defaultGateway.octets[3]),
                "ccpStatus": gigECams[i].ccpStatus,
                "applicationIPAddress": gigECams[i].applicationIPAddress,
                "applicationPort": gigECams[i].applicationPort
            }
        free(gigECams)
        return returnVal

    def registerCallback(self, callbackType, function, *funcArgs):
        """BusManager.registerCallback(callbackType, function, *arguments) -> None

        """
        cdef fc2CallbackHandle* cbHandle
        if callbackType == FC2_BUS_RESET:
            cbHandle = &self.allEvents
        elif callbackType == FC2_ARRIVAL:
            cbHandle = &self.eventArrival
        elif callbackType == FC2_REMOVAL:
            cbHandle = &self.eventRemoval
        cbData = (function, funcArgs)
        checkError(fc2RegisterCallback(self.c, busCallbackBridge, callbackType, <void*>cbData, cbHandle))
        self.callbackArgs[callbackType] = cbData

    def unregisterCallback(self, callbackType):
        """BusManager.unregisterCallback(callbackType) -> None

        """
        cdef fc2CallbackHandle* cbHandle
        if callbackType == FC2_BUS_RESET:
            cbHandle = &self.allEvents
        elif callbackType == FC2_ARRIVAL:
            cbHandle = &self.eventArrival
        elif callbackType == FC2_REMOVAL:
            cbHandle = &self.eventRemoval
        checkError(fc2UnregisterCallback(self.c, cbHandle[0]))
        self.callbackArgs[callbackType] = None

    def getTopology(self):
        """BusManager.getTopology() -> topologyNode

            topologyNode (PyCapture2.topologyNode): PyCapture2.TopologyNode object that contains the topology information.
        """
        top = TopologyNode()
        cdef fc2TopologyNodeContext fc2top = getTopNode(top)
        checkError(fc2GetTopology(self.c, &fc2top))
        setTopNode(top, fc2top)
        return top


cdef class TopologyNode:
    cdef fc2TopologyNodeContext c

    def __cinit__(self):
        checkError(fc2CreateTopologyNode(&(self.c)))

    def __dealloc__(self):
        fc2DestroyTopologyNode(self.c)

    def getGuid(self):
        """TopologyNode.getGuid() -> guid

        """
        cdef fc2PGRGuid uID
        checkError(fc2TopologyNodeGetGuid(self.c, &uID))
        return uID.value[0], uID.value[1], uID.value[2], uID.value[3]

    def getDeviceID(self):
        """TopologyNode.getDeviceID() -> deviceID

        """
        cdef int devID
        checkError(fc2TopologyNodeGetDeviceId(self.c, &devID))
        return devID

    def getNodeType(self):
        """TopologyNode.getNodeType() -> nodeType

        """
        cdef fc2NodeType nodeType
        checkError(fc2TopologyNodeGetNodeType(self.c, &nodeType))
        return nodeType

    def getInterfaceType(self):
        """TopologyNode.getInterfaceType() -> interfaceType

        """
        cdef fc2InterfaceType interType
        checkError(fc2TopologyNodeGetInterfaceType(self.c, &interType))
        return interType

    def getNumChildren(self):
        """TopologyNode.getNumChildren() -> children

        """
        cdef unsigned int numChild
        checkError(fc2TopologyNodeGetNumChildren(self.c, &numChild))
        return numChild

    def getChild(self, unsigned int pos):
        """TopologyNode.getChild(index) -> childNode

        """
        cdef fc2TopologyNodeContext fc2node
        child = TopologyNode()
        checkError(fc2TopologyNodeGetChild(self.c, pos, &fc2node))
        return setTopNode(child, fc2node)

    def addChild(self, TopologyNode child):
        """TopologyNode.addChild(child) -> None

        """
        cdef fc2TopologyNodeContext fc2cont
        fc2cont = getTopNode(child)
        checkError(fc2TopologyNodeAddChild(self.c, fc2cont))

    def getNumPorts(self):
        """TopologyNode.getNumPorts() -> numPorts

        """
        cdef unsigned int numPorts
        checkError(fc2TopologyNodeGetNumPorts(self.c, &numPorts))
        return numPorts

    def getPortType(self, unsigned int pos):
        """TopologyNode.getPortType(position) -> portType

        """
        cdef fc2PortType portType
        checkError(fc2TopologyNodeGetPortType(self.c, pos, &portType))
        return portType

    def addPortType(self, portType):
        """TopologyNode.addPortType(type) -> None

            type (int): The type of port to add. Use PyCapture2.PORT_TYPE to
            read correctly.
        """
        checkError(fc2TopologyNodeAddPortType(self.c, portType))

    def assignGuid(self, tuple guid, int devID, nodeType=None):
        """TopologyNode.assignGuid(guid, deviceID, nodeType = None) -> None

        """
        cdef fc2PGRGuid uID
        uID.value[0], uID.value[1], uID.value[2], uID.value[3] = guid
        if not nodeType:
            checkError(fc2TopologyNodeAssignGuidToNode(self.c, uID, devID))
        else:
            checkError(fc2TopologyNodeAssignGuidToNodeEx(self.c, uID, devID, nodeType))

# cfunctions
cdef inline fc2Context getContext(BaseCamera cont):
    return cont.c

cdef inline fc2Image getImage(Image img):
    return img.i

cdef inline fc2TopologyNodeContext getTopNode(TopologyNode node):
    return node.c

cdef inline setTopNode(TopologyNode node, fc2TopologyNodeContext fc2node):
    node.c = fc2node
    return node

cdef inline fc2ImageStatisticsContext getStats(ImageStatistics stats):
    return stats.c

cdef void imageCallbackBridge(fc2Image* fc2img, void* pCallbackData) with gil:        #with gil so the function can be called from a separate thread and still manipulate python variables.
    cdef fc2Image gcImg
    img = Image()
    img.i = fc2img[0]
    userData = <object>pCallbackData
    userData[0](img, *userData[1])
    img.i = gcImg            #without this, fc2img is garbage collected (because img.i has borrowed it) and on next run through, the function segfaults.

cdef void eventBridge(void* pCallbackData) with gil:
    dataStruct = <fc2EventCallbackData*>pCallbackData
    userData = <object>dataStruct.EventUserData
    cbData = CallbackData()
    cbData.eventName = dataStruct.EventName
    cbData.eventID = dataStruct.EventID
    cbData.eventTimestamp = dataStruct.EventTimestamp
    userData[0](cbData, *userData[1])

cdef void busCallbackBridge(void* pParameter, unsigned int serialNumber) with gil:
    dataStruct = <object>pParameter        #dataStruct is the given pyObject
    dataStruct[0](serialNumber, *dataStruct[1])

cdef void commandCallbackBridge(fc2Error retError, void* pUserData) with gil:
    checkError(retError)
    dataStruct = <object>pUserData
    dataStruct[0](*dataStruct[1])
    Py_DECREF(dataStruct)


# error-handling function
cdef inline bint checkError(fc2Error errorVal) except True:
    cdef bint failed = (errorVal != FC2_ERROR_OK)
    if failed:
        raise Fc2error(errorVal)
    return failed


# callable functions
def getLibraryVersion():
    """getLibraryVersion() -> version

    """
    cdef fc2Version Ver
    checkError(fc2GetLibraryVersion(&Ver))
    return (Ver.major, Ver.minor, Ver.type, Ver.build)


def startSyncCapture(cameras):
    """startSyncCapture(cameras) -> None

        cameras ( [PyCapture2.Camera, PyCapture2.Camera, ...] ): A list of
        cameras to start isochronous capture.
    """
    cdef unsigned int numConts = <unsigned int>len(cameras)
    cdef fc2Context* fc2Conts
    fc2Conts = <fc2Context*>malloc(numConts * sizeof(fc2Context))
    for i in range(len(cameras)):
        fc2Conts[i] = getContext(cameras[i])
    checkError(fc2StartSyncCapture(numConts, fc2Conts))
    free(fc2Conts)


def getRegisterString(unsigned int registerVal):
    """getRegisterString(registerValue) -> registerString

        registerString (str): The textual representation of the register value.
    """
    return fc2GetRegisterString(registerVal)


def setDefaultColorProcessing(defaultMethod):
    """setDefaultColorProcessing(defaultMethod) -> None

        defaultMethod (int): The method to set as default. Use
        PyCapture2.COLOR_PROCESSING to set correctly.
    """
    checkError(fc2SetDefaultColorProcessing(defaultMethod))


def getDefaultColorProcessing():
    """getDefaultColorProcessing() -> defaultMethod

    """
    cdef fc2ColorProcessingAlgorithm defaultMethod
    checkError(fc2GetDefaultColorProcessing(&defaultMethod))
    return defaultMethod


def getDefaultOutputFormat():
    """getDefaultOutputFormat() -> format

    """
    cdef fc2PixelFormat format
    checkError(fc2GetDefaultOutputFormat(&format))
    return format


def setDefaultOutputFormat(format):
    """setDefaultOutputFormat(format) -> None

        format (int): The pixel format to set as default. Use
        PyCapture2.PIXEL_FORMAT to set correctly.
    """
    checkError(fc2SetDefaultOutputFormat(format))


def determineBitsPerPixel(format):
    """determineBitsPerPixel(format) -> bitsPerPixel

    """
    cdef unsigned int bPP
    checkError(fc2DetermineBitsPerPixel(format, &bPP))
    return bPP


def checkDriver(tuple id):
    """checkDriver(guid) -> None

    """
    cdef fc2PGRGuid uID
    uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
    checkError(fc2CheckDriver(&uID))


def getDriverDeviceName(tuple id):
    """getDriverDeviceName(guid) -> name

        name (str): The name of the device.
    """
    cdef char* deviceName = NULL
    cdef size_t nameLen, i
    cdef fc2PGRGuid uID
    uID.value[0], uID.value[1], uID.value[2], uID.value[3] = id
    checkError(fc2GetDriverDeviceName(&uID, deviceName, &nameLen))
    returnVal = ""
    for i in range(nameLen):
        returnVal += nameLen
    return returnVal


def getSystemInfo():
    """getSystemInfo() -> systemInfo

    """
    cdef fc2SystemInfo sysInfo
    checkError(fc2GetSystemInfo(&sysInfo))
    returnVal = SystemInfo()
    returnVal.__dict__ = {
        "osType": sysInfo.osType,
        "osDescription": sysInfo.osDescription,
        "byteOrder": sysInfo.byteOrder,
        "sysMemSize": sysInfo.sysMemSize,
        "cpuDescription": sysInfo.cpuDescription,
        "numCpuCores": sysInfo.numCpuCores,
        "driverList": sysInfo.driverList,
        "libraryList": sysInfo.libraryList,
        "gpuDescription": sysInfo.gpuDescription,
        "screenWidth": sysInfo.screenWidth,
        "screenHeight": sysInfo.screenHeight
    }
    return returnVal


def launchBrowser(const char* url):
    """launchBrowser(url) -> None

        url (str): The URL to open.
    """
    checkError(fc2LaunchBrowser(url))


def launchHelp(const char* fileName):
    """launchHelp(fileName) -> None

        fileName (str): Filename of the CHM file to open.
    """
    checkError(fc2LaunchHelp(fileName))


def launchCommand(const char* command):
    """launchCommand(command) -> None

        command (str): The command to execute.
    """
    checkError(fc2LaunchCommand(command))


def launchCommandAsync(const char* command, func, *funcArgs):
    """launchCommandAsync(command, function, *arguments) -> None

        arguments: Any additional arguments will be passed to the callback function when it is called.
    """
    cbData = (func, funcArgs)
    Py_INCREF(cbData)
    checkError(fc2LaunchCommandAsync(command, commandCallbackBridge, <void*>cbData))


PyEval_InitThreads()    #allows cython to use the GIL, making async callbacks and events possible.
