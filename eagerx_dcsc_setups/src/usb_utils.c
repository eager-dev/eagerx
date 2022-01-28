#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <eagerx_dcsc_setups/usb_utils.h>

#define USB4LINUX

/*****************************************************************************
;*
;*		scaling constants
;*
;****************************************************************************/

#define WATCHDOGRES     (250E-6)
#define RELTIMERES      (1E-6)
#define CLOCK           (48E6)
#define FLANKSPERPULSE  (4)
#define PULSESPERCIRCLE (500)
#define PWMCLOCKDIVIDER (15)

/*****************************************************************************
;*
;*		other constants
;*
;****************************************************************************/

#define MAXFUGINAME            20
#define MAXFUGIES              8
#define MAXCOMMANDLEN          20
#define MAXVALUENAMELEN        20
#define MAXERRORLEN            (40+MAXVALUENAMELEN)
#define PI                     3.14159265358979323846264
#define INVALID_USB_DEVICE     ((libusb_device_handle *) -1)

/*****************************************************************************
;*
;*		typedefs
;*
;****************************************************************************/

typedef struct FUGIstruct *saxo;

typedef FpStatus (*rcommand)(saxo port, const int argc, float *argv);
typedef FpStatus (*wcommand)(saxo port, const int argc, const float *argv);

typedef struct {
    char          Name[MAXFUGINAME];
    char          HwCode[MAXFUGINAME];
    rcommand      ReadFn;
    wcommand      WriteFn;
} FUGImodel;

typedef struct FUGIstruct {
	char          Name[MAXFUGINAME];
    char          HwCode[MAXFUGINAME];
	int32_t       Device;

    rcommand      ReadFn;
    wcommand      WriteFn;

	uint32_t      ReadError;
	uint32_t      WriteError;

    libusb_device_handle  *Handle;
    struct libusb_context *Context;
} FUGIdevice;


/*****************************************************************************
;*
;*		prototypes
;*
;****************************************************************************/

static FpStatus ReadMOPS         (saxo port, const int dstn, float *dstp);
static FpStatus WriteMOPS        (saxo port, const int srcn, const float *srcp);
static FpStatus ReadBALLPLATE    (saxo port, const int dstn, float *dstp);
static FpStatus WriteBALLPLATE   (saxo port, const int srcn, const float *srcp);
static FpStatus ReadHUBOAT       (saxo port, const int dstn, float *dstp);
static FpStatus WriteHUBOAT      (saxo port, const int srcn, const float *srcp);
static FpStatus ReadPENDULUM     (saxo port, const int dstn, float *dstp);
static FpStatus WritePENDULUM    (saxo port, const int srcn, const float *srcp);
static FpStatus ReadENCODERPWM   (saxo port, const int dstn, float *dstp);
static FpStatus WriteENCODERPWM  (saxo port, const int srcn, const float *srcp);
static inline uint8_t ConvertENCODERPWM(float frequency, float ratio, uint32_t *count_on, uint32_t *count_off);

/*****************************************************************************
;*
;*		global variables
;*
;****************************************************************************/

static FUGImodel FUGImodels[] = {
    "MOPS", "(0.0)", ReadMOPS, WriteMOPS,
    "BALLPLATE", "(0.1)", ReadBALLPLATE, WriteBALLPLATE,
    "HUBOAT", "(0.2)", ReadHUBOAT, WriteHUBOAT,
    "PENDULUM", "(0.3)", ReadPENDULUM, WritePENDULUM,
    "ENCODERPWM", "(0.4)", ReadENCODERPWM, WriteENCODERPWM,
};

static int FUGIcount = 0;
static FUGIdevice Port[MAXFUGIES];



/*****************************************************************************
******************************************************************************

support routines

******************************************************************************
*****************************************************************************/


/*****************************************************************************
;*
;*		ALLOCCHECK
;*		check allocated memory for validity
;*
;*		Input:	pointer to check
;*		Output:	checked pointer
;*
;****************************************************************************/

// static mxArray *AllocCheck(mxArray *p){
// 	if (p==NULL) mexErrMsgTxt("Out of memory.");
// 	return(p);
// }

/*****************************************************************************
;*
;*		SETPORTPARAMS
;*		fill FUGIdevice structure from input
;*
;*		Input:	port = FUGIdevice pointer
;*				obj = FUGI object
;*		Output:	none
;*
;****************************************************************************/

// static void SetPortParams(saxo port, const mxArray *obj){
// 	FUGIdevice temp;

// 	/* fill a temporary structure, update just when success */
// 	temp = *port;

// 	/* last read error, always reset to zero */
// 	temp.ReadError = 0;

// 	/* last write error, always reset to zero */
// 	temp.WriteError = 0;

// 	/* everything OK, so copy the result */
// 	*port = temp;
// }

/*****************************************************************************
;*
;*		GETSTRINGPORT
;*		get port pointer by string or FUGIdevice object
;*
;*		Input:	obj = string or possible FUGIdevice object
;*		Output:	FUGIdevice pointer
;*
;****************************************************************************/

static saxo GetStringPort(const char *name) {
    int i, devnum;

	/* Try to find the name in the list of active FUGIdevices */
	for (i = 0; i < FUGIcount; i++) {
		if (strcasecmp(Port[i].Name, name) == 0) {
			return(&Port[i]);
		}
	}

	/* Not an active board, see if there is room for another one */
	if (FUGIcount >= MAXFUGIES) {
		ROS_ERROR("Too many FUGIdevice devices referenced.");
	    return(NULL);
	}

	/* There is room, try to match the name to the list of supported models */
	for (i = 0; i < sizeof(FUGImodels)/sizeof(FUGImodel); i++) {
		if (strncasecmp(name, FUGImodels[i].Name, strlen(FUGImodels[i].Name))==0) {

			/* found model, check the device number */
			devnum = name[strlen(FUGImodels[i].Name)] - '0' - 1;
			if (devnum < 0 || devnum > MAXFUGIES) {
				ROS_ERROR("Device number is incorrect.");
			    return(NULL);
			}

			/* everything checked out, add board to list of active boards */
			memset(&Port[FUGIcount], 0, sizeof(FUGIdevice));
			strncpy(Port[FUGIcount].Name, name, MAXFUGINAME);
			strncpy(Port[FUGIcount].HwCode, FUGImodels[i].HwCode, MAXFUGINAME);
			Port[FUGIcount].Device = devnum;
			Port[FUGIcount].Handle = INVALID_USB_DEVICE;
			Port[FUGIcount].ReadFn = FUGImodels[i].ReadFn;
			Port[FUGIcount].WriteFn = FUGImodels[i].WriteFn;
			return(&Port[FUGIcount++]);
		}
	}

	/* didn't find existing device or model, so it's invalid */
	ROS_ERROR("Invalid FUGIdevice device.");
    return(NULL);
}

/*****************************************************************************
;*
;*		USB Functions
;*		highly system dependent code to open and access USB devices
;*
;****************************************************************************/

#define MAX_INTEL_HEX_RECORD_LENGTH 16
typedef struct _INTEL_HEX_RECORD {
   uint32_t  	Length;
   uint32_t 	Address;
   uint32_t  	Type;
   uint8_t  	Data[MAX_INTEL_HEX_RECORD_LENGTH];
} INTEL_HEX_RECORD;

static FpStatus FX2Write(libusb_device_handle *handle, uint32_t address, uint16_t length, uint8_t *data) {
    int result;

    if (length == 0) return(OK);
    result = libusb_control_transfer(handle,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
            0xA0, address & 0xFFFF, address >> 16,
            data, length, 1000);
    if (result != length) return(OPERATION_FAILED);
    return(OK);
}

static FpStatus hexRead(INTEL_HEX_RECORD *record, FILE *hexFile) {
	int c;
	uint16_t i;
	int n, c1, check, len;

    while (((c = getc(hexFile)) != EOF) && c != ':') {
        n = fscanf(hexFile, "%*s\n");
    }
    if (c == EOF) return(NO_DATA_READ);

    if (fscanf(hexFile, "%2X%4X%2X", &record->Length, &record->Address, &record->Type) != 3) return(NO_DATA_READ);

    if ((len = record->Length) > MAX_INTEL_HEX_RECORD_LENGTH) return(NO_DATA_READ);

    for (i = 0; i < len; i++) {
        n = fscanf(hexFile, "%2X", &c1);
        if (n != 1) {
            if (i != record->Length) {
                return(NO_DATA_READ);
            }
        }
        record->Data[i] = c1;

    }

    if (fscanf(hexFile, "%2X\n", &check) != 1) return(NO_DATA_READ);

    return(OK);
}

static FpStatus FX2LoadRam(libusb_device_handle *handle, FILE *image) {
	INTEL_HEX_RECORD hexLine;
    uint8_t byteVal;
    FpStatus result;

    /* Make sure we start at the beginning of the image file */
    rewind(image);

    /* Stop the FX2 CPU */
    byteVal = 1;
    if (FX2Write(handle, 0xE600, 1, &byteVal) != OK) return(OPERATION_FAILED);

    // Download code
    while (1) {
        result = hexRead(&hexLine, image);
        if (hexLine.Type != 0) break;
        if (result != OK) return(OPERATION_FAILED);
        if (FX2Write(handle, hexLine.Address, hexLine.Length, hexLine.Data) != OK) return(OPERATION_FAILED);
    }

    /* De-assert reset, ignore errors, we can't do anything anymore */
    byteVal = 0;
    FX2Write(handle, 0xE600, 1, &byteVal);
    return(OK);
}

static FpStatus DownloadFirmware(saxo port) {
    char *path2hex;
    FILE *hexfile;
    int sleeptime = 0;
    libusb_device **list;
    libusb_device_handle *handle;
    ssize_t cnt;
    FpStatus status = OK;
    struct libusb_device_descriptor desc;

    /* Download firmware in uninitialized FX2 chips if a firmware image can be found */
    if (((path2hex = getenv("DCSCUSB_HEX_FILE")) != NULL) && ((hexfile = fopen(path2hex, "rb")) != NULL)) {

        /* Get a list of all USB devices or skip the initialization if we can't get a list */
        if ((cnt = libusb_get_device_list(port->Context, &list)) > 0) {

            /* Look for uninitialized devices in the list and initialize those */
            for (int i = 0; i < cnt; i++) {
                if (libusb_get_device_descriptor(list[i], &desc) == 0) {
                    if (desc.idVendor == 0x04B4 && desc.idProduct == 0x8613) {
                        if (libusb_open(list[i], &handle) == 0) {
                            status = OK;
                            if (libusb_claim_interface(handle, 0) == 0) {
                                status = FX2LoadRam(handle, hexfile);
                                libusb_release_interface(handle, 0);
                            }
                            libusb_close(handle);
                            if (status != OK) break;
                            sleeptime = 3;
                        }
                    }
                }
            }

            /* We're done with the list of devices */
            libusb_free_device_list(list, 1);
        }

        /* Give the devices some time to renumerate if we initialized any device */
        if (sleeptime) {
            ROS_WARN("FX2 device(s) initialized, waiting for re-enumeration.\n");
            sleep(sleeptime);
        }

        /* We're done with the file */
        fclose(hexfile);
    }

	return(status);
}

static FpStatus USBopen(saxo port) {
    char *hwcode;
    char message[200];
    int deviceCount = 0;
    int deviceNumber = port->Device;
    int result;
    libusb_device **list;
    libusb_device_handle *found = NULL;
    libusb_device_handle *handle;
    ssize_t cnt;
    struct libusb_device_descriptor desc;
    unsigned char boardName[200];

    /* Open a context for the USB driver if not open yet */
    if (port->Context == NULL) {
        if ((result = libusb_init(&port->Context)) < 0) {
            ROS_ERROR("Unable to create USB context: %s.\n", libusb_error_name(result));
			return(NOT_OPEN);
        }
    }

    /* Download firmware to unintialized FUGIboards */
	if (DownloadFirmware(port) != OK) {
		ROS_ERROR("Unable to load firmware into FX2 chips.\n");
		return(NOT_OPEN);
	}

    /* Get a list of all USB devices */
    if ((cnt = libusb_get_device_list(port->Context, &list)) < 1) {
        libusb_exit(port->Context);
        port->Context = NULL;
        if (cnt < 0) {
            ROS_ERROR("Unable to get list of USB devices: %s.\n", libusb_error_name(result));
			return(NOT_OPEN);
        } else if (cnt == 0) {
            /* no USB devices at all */
            ROS_ERROR("No USB devices found!");
			return(NOT_OPEN);
        }
    }

    /* Look for the desired devices in the list, count them and find the requested number */
    for (int i = 0; i < cnt; i++) {
        if (libusb_get_device_descriptor(list[i], &desc) == 0) {
            if (libusb_open(list[i], &handle) == 0) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iProduct, boardName, sizeof(boardName)) > 0) {
                    if ((hwcode = strrchr((char *)boardName, '(')) != NULL) {

                        /* Is this the desired device type? */
                        if (strcasecmp(hwcode, port->HwCode) == 0) {

                            /* This is a desired device, but is it the right one? */
                            if (deviceCount == deviceNumber) {

                                /* Yes, it is */
                                if ((result = libusb_claim_interface(handle, 0)) < 0) {
                                    ROS_ERROR("Unable to claim USB interface: %s.\n", libusb_error_name(result));
									return(NOT_OPEN);
                                }
                                port->Handle = handle;
                                libusb_free_device_list(list, 1);
                                return(OK);
                            } else {

                                /* No, it is not, try next device */
                                deviceCount++;
                            }
                        }
                    }
                }
                libusb_close(handle);
            }
        }
    }

	/* Clean up and return because we didn't find anything */
    libusb_free_device_list(list, 1);
    libusb_exit(port->Context);
    port->Context = NULL;
    ROS_ERROR("No matching device found!");
    return(NOT_OPEN);
}

static FpStatus USBread(saxo port, void *buffer, uint32_t len) {
    unsigned int timeout = 0;
    int halflen = len / 2;
    int firsthalftransferred, secondhalftransferred;
    int result;

    /* Port must be open */
    if ((port->Handle != INVALID_USB_DEVICE) && (port->Context != NULL)) {

        /* Read the data from the device in two steps */
        result = libusb_bulk_transfer(port->Handle, 0x86, (unsigned char *)buffer , halflen, &firsthalftransferred, timeout);
        if (result != 0) {
            port->ReadError = result;
            return(OPERATION_FAILED);       /* USB operation failed */
        }
        result = libusb_bulk_transfer(port->Handle, 0x86, ((unsigned char *)buffer) + halflen, halflen, &secondhalftransferred, timeout);
        if (result != 0) {
            port->ReadError = result;
            return(OPERATION_FAILED);       /* USB operation failed */
        }

        /* Make sure we got all data */
        if ((firsthalftransferred + secondhalftransferred) != len) {
            return(NO_DATA_READ);           /* wrong number of bytes read */
        } else {
            return(OK);                     /* everything okay */
        }
    } else {
        return(NOT_OPEN);                   /* port not connected */
    }
}

static FpStatus USBwrite(saxo port, void *data, int len) {
    unsigned int timeout = 100;
    int transferred, result;

    /* Port must be open */
    if ((port->Handle != INVALID_USB_DEVICE) && (port->Context != NULL)) {

        /* Read the data from the device */
        result = libusb_bulk_transfer(port->Handle, 0x02, (unsigned char *) data, len, &transferred, timeout);

        /* Chek the result */
        if (result != 0) {
            port->WriteError = result;
            return(OPERATION_FAILED);       /* USB operation failed */
        }  else if (transferred != len) {
            return(NO_DATA_READ);           /* wrong number of bytes read */
        } else {
            return(OK);                     /* everything okay */
        }
    } else {
        return(NOT_OPEN);                   /* port not connected */
    }
}

static void USBclose(saxo port) {
	/* First close the handle to the USB device */
	if (port->Handle != INVALID_USB_DEVICE) {
        libusb_release_interface(port->Handle, 0);
		libusb_close(port->Handle);
		port->Handle = INVALID_USB_DEVICE;
	}

	/* Then close the libusb context */
	if (port->Context != NULL) {
		libusb_exit(port->Context);
		port->Context = NULL;
	}
}

/*****************************************************************************
******************************************************************************

experimental setup routines

******************************************************************************
*****************************************************************************/

#define GETBYTE(t, s, m) t = ((temp = (s)) & (m))
#define GETRELTIME(t, s, r) t = ((s##h << 8) + s##l) * r
#define GETPOSITION(t, s, fp, pc) if (s##h < 128) { temp = ((0 << 24) + (s##h << 16) + (s##m << 8) + s##l); } else { temp = ((-1 << 24) + (s##h << 16) + (s##m << 8) + s##l); } t = (2.0 * PI * ((double) temp)) / (fp * pc)
#define GETPOSITION16(t, s, fp, pc) if (s##h < 128) { temp = ((0 << 16) + (s##h << 8) + s##l); } else { temp = ((-1 << 16) + (s##h << 8) + s##l); } t = (2.0 * PI * ((double) temp)) / (fp * pc)
#define GETRAWPOSITION20ODD(t, s) temp = ((s##b3 & 0x0F) << 16) + (s##b2 << 8) + s##b1; if (temp & 0x00080000) temp |= 0xFFF00000; t = (double)temp
#define GETRAWPOSITION20EVEN(t, s) temp = (s##b5 << 12) + (s##b4 << 4) + ((s##b3 & 0xF0) >> 4); if (temp & 0x00080000) temp |= 0xFFF00000; t = (double)temp

#define GETSPEED(t, s, st, cl, pc) temp = ((s##h << 16) + (s##m << 8) + s##l); if (temp == 0x00FFFFFF) { t = 0.0; } else if (st < 128) { t = ((2.0 * PI * cl) / pc) / ((double) temp); } else { t = -((2.0 * PI * cl) / pc) / ((double) temp); }
#define GETADC(t, s, i, a) temp = ((s##h << 8) + s##l); t = (((a - i) * ((double) temp))/0xFFFF) + i
#define GETSADC(t, s, i, a) temp = (s); t = (((a - i) * ((double) temp))/0xFF) + i

#define SETBYTE(t, s) temp = (int32_t)(s); if (temp > 0xFF) temp = 0xFF; if (temp < 0) temp = 0; t = (uint8_t)temp
#define SETWORD(t, s) temp = (int32_t)(s); if (temp > 0xFFFF) temp = 0xFFFF; if (temp < 0) temp = 0; t##l = (uint8_t) (temp & 0xFF); t##h = (uint8_t) ((temp & 0xFF00) >> 8)
#define SETDAC(t, s, i, a) temp = (int32_t)((((s) - (i)) * 0xFFFF) / ((a) - (i)) + 0.5); if (temp > 0xFFFF) temp = 0xFFFF; if (temp < 0) temp = 0; t##l = (uint8_t) (temp & 0xFF); t##h = (uint8_t) ((temp & 0xFF00) >> 8)
#define SETSERVO(t, a, v) if ((a) < 0) { temp = 0; } else if ((a) > 63) { temp = 0; } else { temp = ((a) << 10); } if (v < -500) { temp += 0; } else if (v > 500) { temp += 1000; } else { temp += v + 500; } t.datal = (uint8_t) (temp & 0xFF); t.datah = (uint8_t) ((temp & 0xFF00) >> 8)

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the MOPS
;*
;*
;****************************************************************************/

static FpStatus ReadMOPS(saxo port, const int dstn, float *dstp) {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t positionl, positionm, positionh;
        uint8_t speedl, speedm, speedh;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t voltagel, voltageh;
        uint8_t externl, externh;
        uint8_t status;
    } data;
    FpStatus ret;
    int32_t temp;

    if ((ret = USBread(port, &data, 32)) == OK) {

        /* Make sure that everything will fit in the data */
		if (dstn < 8) return(INVALID_BUFFER_SIZE);

        /* Get the status */
        GETBYTE(dstp[0], data.status, 0x7F);

        /* Get the time since the previous sample and remember its raw value */
        GETRELTIME(dstp[1], data.reltime, RELTIMERES);

        /* Get the postion from the last measurement in this sample and remember its raw value */
        GETPOSITION(dstp[2], data.position, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the momentary speed */
        GETSPEED(dstp[3], data.speed, data.status, CLOCK, PULSESPERCIRCLE);

        /* Get the motor current */
        GETADC(dstp[4], data.current, -3.0, 3.0);

        /* Get the motor voltage */
        GETADC(dstp[5], data.voltage, -12.5, 12.5);

        /* Get the extern voltage */
        GETADC(dstp[6], data.voltage, -10.0, 10.0);

        /* Get the digital inputs */
        GETBYTE(dstp[7], data.digin, 0xFF);

        return(OK);
    } else {
        return(ret);
    }
}

static FpStatus WriteMOPS(saxo port, const int srcn, const float *srcp) {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } data;
    int32_t temp;

	/* Don't do anything if not enough input data */
	if (srcn < 5) return(INVALID_BUFFER_SIZE);

    /* Convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.cmd, srcp[0]);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.digout, srcp[1]);

    /* Set the first DAC value */
    SETDAC(data.dac0, srcp[2], -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(data.dac1, srcp[3], -10.0, 10.0);

    /* Set the timeout */
    SETWORD(data.timeout, srcp[4] / WATCHDOGRES);

    /* Now write it */
    return(USBwrite(port, &data, sizeof(data)));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the Ball-and-Plate
;*
;*
;****************************************************************************/

static FpStatus ReadBALLPLATE(saxo port, const int dstn, float *dstp) {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position1l, position1m, position1h;
        uint8_t position2l, position2m, position2h;
        uint8_t digin;
        uint8_t voltageXl, voltageXh;
        uint8_t voltageYl, voltageYh;
        uint8_t sadcX, sadcY;
        uint8_t status;
    } data;
    FpStatus ret;
    int32_t temp;

    if ((ret = USBread(port, &data, 32)) == OK) {

        /* Make sure that everything will fit in the data */
		if (dstn < 9) return(INVALID_BUFFER_SIZE);

        /* Get the status */
        GETBYTE(dstp[0], data.status, 0x7F);

        /* Get the time since the previous sample */
        GETRELTIME(dstp[1], data.reltime, RELTIMERES);

        /* Get the postion from the first encoder */
        GETPOSITION(dstp[2], data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the postion from the second encoder */
        GETPOSITION(dstp[3], data.position2, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the voltage from the inclinometer X and Y*/
        GETADC(dstp[4], data.voltageX, 0.0, 5.0);
        GETADC(dstp[5], data.voltageY, 0.0, 5.0);

        /* Get the voltages from the joystick */
        GETSADC(dstp[6], data.sadcX, 0.0, 3.3);
        GETSADC(dstp[7], data.sadcY, 0.0, 3.3);

        /* Get the digital inputs */
        GETBYTE(dstp[8], data.digin, 0xFF);

        return(OK);
    } else {
        return(ret);
    }
    return(OK);
}

static FpStatus WriteBALLPLATE(saxo port, const int srcn, const float *srcp) {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } data;
    int32_t temp;

	/* Don't do anything if not enough input data */
	if (srcn < 5) return(INVALID_BUFFER_SIZE);

    /* Convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.cmd, srcp[0]);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.digout, srcp[1]);

    /* DAC input value is between +12 and -12, move up to between +24 and 0, scale and limit between 0xFFFF and 0x0000  */
    SETDAC(data.dac0, srcp[2], -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(data.dac1, srcp[3], -10.0, 10.0);

    /* Set the timeout */
    SETWORD(data.timeout, srcp[4] / WATCHDOGRES);

    /* Now write it */
    return(USBwrite(port, &data, sizeof(data)));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the HuBoat
;*
;*
;****************************************************************************/

static FpStatus ReadHUBOAT(saxo port, const int dstn, float *dstp) {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position1l, position1m, position1h;
        uint8_t position2l, position2m, position2h;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t voltagel, voltageh;
        uint8_t externl, externh;
        uint8_t status;
    } data;
    FpStatus ret;
    int32_t temp;

    if ((ret = USBread(port, &data, 32)) == OK) {

        /* Make sure that everything will fit in the data */
		if (dstn < 8) return(INVALID_BUFFER_SIZE);

        /* Get the status */
        GETBYTE(dstp[0], data.status, 0x7F);

        /* Get the time since the previous sample */
        GETRELTIME(dstp[1], data.reltime, RELTIMERES);

        /* Get the postion from the first encoder */
        GETPOSITION(dstp[2], data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the postion from the second encoder */
        GETPOSITION(dstp[3], data.position2, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the motor current */
        GETADC(dstp[4], data.current, -5.1, 5.1);

        /* Get the voltage from the first potentiometer */
        GETADC(dstp[5], data.voltage, 0.0, 5.0);

        /* Get the voltage from the second potentiometer */
        GETADC(dstp[6], data.extern, 0.0, 5.0);

        /* Get the digital inputs */
        GETBYTE(dstp[7], data.digin, 0xFF);

        return(OK);
    } else {
        return(ret);
    }
    return(OK);
}

static FpStatus WriteHUBOAT(saxo port, const int srcn, const float *srcp) {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
        struct {
            uint8_t datal, datah;
        } servo[4];
    } data;
    FpStatus ret;
    int32_t temp;
    double dac2;
    int M, N, servocount, nextfull, i, address, value;
    double *ptr;
    //char message[300];

	/* Don't do anything if not enough input data */
	if (srcn < 5) return(INVALID_BUFFER_SIZE);

    /* convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.cmd, srcp[0]);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.digout, srcp[1]);

    /* DAC input value is between +12 and -12, move up to between +24 and 0, scale and limit between 0xFFFF and 0x0000  */
    SETDAC(data.dac0, srcp[2], -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(data.dac1, srcp[3], -10.0, 10.0);

    /* Set the timeout */
    SETWORD(data.timeout, srcp[4] / WATCHDOGRES);

    // /* Set the servo values */
    // M = mxGetM(srcp[4]);
    // N = mxGetN(srcp[4]);
    // ptr = mxGetPr(srcp[4]);
    // servocount = (N <= 2) ? M : N;
    // nextfull = 4;
    // for (i = 0; i < servocount;) {
    //     if (N == 2) {
    //         /* 2 columns, servo address in 1st column, servo value in 2nd column */
    //         address = (int)dstp[i];
    //         value = (int)dstp[i + M];
    //     } else {
    //         /* vector, servo address increments, servo value in vector */
    //         address = i + 1;
    //         value = (int)ptr[i];
    //     }
    //     SETSERVO(data.servo[i - nextfull + 4], address, value);

    //     /* Write data if data full or no more data */
    //     i += 1;
    //     if ((i == nextfull) || (i == servocount)) {
    //         if ((ret = USBwrite(port, &data, sizeof(data))) != OK) return(ret);
    //         memset(&data.servo, 0, sizeof(data.servo));
    //         nextfull += 4;
    //     }
    // }
    return(OK);
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the Rotating Pendulum 2
;*
;*
;****************************************************************************/

static FpStatus ReadPENDULUM(saxo port, const int dstn, float *dstp) {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position1l, position1m, position1h;
        uint8_t position2l, position2m, position2h;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t beaml, beamh;
        uint8_t penduluml, pendulumh;
        uint8_t status;
    } data;
    FpStatus ret;
    int32_t temp;

    if ((ret = USBread(port, &data, 32)) == OK) {

        /* Make sure that everything will fit in the data */
		if (dstn < 8) return(INVALID_BUFFER_SIZE);

        /* Get the status */
        GETBYTE(dstp[0], data.status, 0x7F);

        /* Get the time since the previous sample */
        GETRELTIME(dstp[1], data.reltime, RELTIMERES);

        /* Get the postion from the first encoder */
        GETPOSITION(dstp[2], data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the postion from the second encoder */
        GETPOSITION(dstp[3], data.position2, FLANKSPERPULSE, PULSESPERCIRCLE);

        /* Get the motor current */
        GETADC(dstp[4], data.current, -5.1, 5.1);

        /* Get the voltage from the beam potentiometer */
        GETADC(dstp[5], data.beam, 0.0, 5.0);

        /* Get the voltage from the pendulum potentiometer */
        GETADC(dstp[6], data.pendulum, 0.0, 5.0);

        /* Get the digital inputs */
        GETBYTE(dstp[7], data.digin, 0xFF);

        return(OK);
    } else {
        return(ret);
    }
}

static FpStatus WritePENDULUM(saxo port, const int srcn, const float *srcp) {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } data;
    int32_t temp;

	/* Don't do anything if not enough input data */
	if (srcn < 5) return(INVALID_BUFFER_SIZE);

    /* convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.cmd, srcp[0]);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(data.digout, srcp[1]);

    /* DAC input value is between +12 and -12, move up to between +24 and 0, scale and limit between 0xFFFF and 0x0000  */
    SETDAC(data.dac0, srcp[2], -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(data.dac1, srcp[3], -10.0, 10.0);

    /* Set the timeout */
    SETWORD(data.timeout, srcp[4] / WATCHDOGRES);

    /* Now write it */
    return(USBwrite(port, &data, sizeof(data)));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the EncoderPWM
;*
;*
;****************************************************************************/

static FpStatus ReadENCODERPWM(saxo port, const int dstn, float *dstp) {
    struct {
        uint8_t dummy[16];
        uint8_t pospair1b1, pospair1b2, pospair1b3, pospair1b4, pospair1b5;
        uint8_t pospair2b1, pospair2b2, pospair2b3, pospair2b4, pospair2b5;
        uint8_t pospair3b1, pospair3b2, pospair3b3, pospair3b4, pospair3b5;
        uint8_t status;
    } data;
    FpStatus ret;
    int32_t temp;
    double *ptr;

    if ((ret = USBread(port, &data, 32)) == OK) {

        /* Make sure that everything will fit in the data */
		if (dstn < 7) return(INVALID_BUFFER_SIZE);

        /* Get the status */
        GETBYTE(dstp[0], data.status, 0x7F);

        /* Get the postion from the first encoder */
        GETRAWPOSITION20ODD(dstp[1], data.pospair1);

        /* Get the postion from the second encoder */
        GETRAWPOSITION20EVEN(dstp[2], data.pospair1);

        /* Get the postion from the third encoder */
        GETRAWPOSITION20ODD(dstp[3], data.pospair2);

        /* Get the postion from the fourth encoder */
        GETRAWPOSITION20EVEN(dstp[4], data.pospair2);

        /* Get the postion from the fifth encoder */
        GETRAWPOSITION20ODD(dstp[5], data.pospair3);

        /* Get the postion from the sixth encoder */
        GETRAWPOSITION20EVEN(dstp[6], data.pospair3);

        return(OK);
    } else {
        return(ret);
    }
}

static FpStatus WriteENCODERPWM(saxo port, const int srcn, const float *srcp) {
    #define FPGA_CLOCK
    struct {
        uint8_t cmd;
        uint8_t ontime1l, ontime1h;
        uint8_t offtime1l, offtime1h;
        uint8_t ontime2l, ontime2h;
        uint8_t offtime2l, offtime2h;
        uint8_t ontime3l, ontime3h;
        uint8_t offtime3l, offtime3h;
    } data;
    int32_t temp;
    float frequency, ratio;
    uint32_t count_total, count_on;

	/* Don't do anything if not enough input data */
	if (srcn < 5) return(INVALID_BUFFER_SIZE);

    /* Just limit command byte between 0x0F and 0x00 (inclusive) */
    SETBYTE(data.cmd, srcp[0]);
    data.cmd &= 0x0F;

    /* Check and process frequency parameter */
    frequency = srcp[1];
    if (frequency < 50.0 || frequency > 25000.0) {
		ROS_ERROR("Frequency %d out of range.", frequency);
		return(INVALID_PARAM);
	}
    count_total = CLOCK / (frequency * PWMCLOCKDIVIDER);

    /* Process first ratio parameter */
    ratio = srcp[2];
    if (ratio < -1.0 || ratio > 1.0) {
		ROS_ERROR("Ratio 1 value %g out of range.", ratio);
		return(INVALID_PARAM);
	}
    data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 0;
    SETWORD(data.ontime1, count_total);
    SETWORD(data.offtime1, count_on);

    /* Process second ratio parameter */
    ratio = srcp[3];
    if (ratio < -1.0 || ratio > 1.0) {
		ROS_ERROR("Ratio 2 value %g out of range.", ratio);
		return(INVALID_PARAM);
	}
    data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 1;
    SETWORD(data.ontime2, count_total);
    SETWORD(data.offtime2, count_on);

    /* Process third ratio parameter */
    ratio = srcp[4];
    if (ratio < -1.0 || ratio > 1.0) {
		ROS_ERROR("Ratio 3 value %g out of range.", ratio);
		return(INVALID_PARAM);
	}
    data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 2;
    SETWORD(data.ontime3, count_total);
    SETWORD(data.offtime3, count_on);

    /* Now write it */
    return(USBwrite(port, &data, sizeof(data)));
}

static inline uint8_t ConvertENCODERPWM(float frequency, float ratio, uint32_t *count_total, uint32_t *count_on) {
    uint8_t result = 0;

    /* Handle low frequencies in Servo Mode, higher frequencies as PWM */
    if (frequency <= 500) {
        *count_on = ((((ratio * 0.6E-3) + 1.5E-3) * CLOCK) / PWMCLOCKDIVIDER);
    } else {
        if (ratio < 0.0) {
            ratio = -ratio;
            result = 0x10;
        }
        *count_on = (int)(*count_total * ratio);
    }
    return(result);
}


/*****************************************************************************
******************************************************************************

Interface actions

******************************************************************************
*****************************************************************************/


/*****************************************************************************
;*
;*		FPOPEN
;*		open FUGIdevice
;*
;*		Input:	device = name and number of device to open
;*              port = handle to open device
;*		Output:	status
;*
;****************************************************************************/

FpStatus FpOpen(const char setupname[], saxo *port) {
	FpStatus status;

    /* Reuse or create new port handle  */
	if ((*port)->Handle == INVALID_USB_DEVICE) {
		USBopen(*port);
	}

	return(status);
}



/*****************************************************************************
;*
;*		FPCLOSE
;*		close FUGIdevice
;*
;*      Input:  port = FUGIdevice
;*      Output: none
;*
;****************************************************************************/

FpStatus FpClose(saxo port) {
	FpStatus status;

	/* Close the port */
	if (port->Handle != INVALID_USB_DEVICE) {
		status = OK;
		USBclose(port);
	} else {
		status = NOT_OPEN;
	}

	return(status);
}

/*****************************************************************************
;*
;*		FPCLOSEALL
;*		close all FUGIdevice ports
;*
;*      Input:  none
;*      Output: status (always OK)
;*
;****************************************************************************/

FpStatus FpCloseAll() {
	int i;

	/* Close all ports */
	for (i=0; i < FUGIcount; i++) FpClose(&Port[i]);

	return(OK);
}

/*****************************************************************************
;*
;*		FPREAD
;*		read FUGIdevice data
;*
;*		Input:	port = FUGIdevice
;*				dstn = number of destination values
;*				dstp = pointer to destination values
;*		Output:	status
;*
;****************************************************************************/

FpStatus FpRead(saxo port, const int dstn, float *dstp) {
	FpStatus status;

    if (port->Handle != INVALID_USB_DEVICE && port->ReadFn != NULL) {
        status = port->ReadFn(port, dstn, dstp);
	} else {
		status = NOT_OPEN;
	}

	return(status);
}

/*****************************************************************************
;*
;*		FPWRITE
;*		write FUGIdevice data
;*
;* 		Input:	port = FUGIdevice
;*				srcn = number of source values
;*				srcp = pointer to source values
;*		Output:	none
;*
;****************************************************************************/

FpStatus FpWrite(saxo port, const int srcn, const float *srcp) {
	FpStatus status;

    if (port->Handle != INVALID_USB_DEVICE && port->WriteFn != NULL) {
        status = port->WriteFn(port, srcn, srcp);
	} else {
		status = NOT_OPEN;
	}

	return(status);
}
