using namespace std;
#include <eagerx_dcsc_setups/usb_utils.h>
#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/*****************************************************************************
;*
;*		scaling constants
;*
;****************************************************************************/

#define WATCHDOGRES (250E-6)
#define RELTIMERES (1E-6)
#define CLOCK (48E6)
#define FLANKSPERPULSE (4)
#define PULSESPERCIRCLE (500)
#define PWMCLOCKDIVIDER (15)

/*****************************************************************************
;*
;*		other constants
;*
;****************************************************************************/

#define MAXFUGINAME 20
#define MAXFUGIES 8
#define MAXCOMMANDLEN 20
#define MAXVALUENAMELEN 20
#define MAXERRORLEN (40 + MAXVALUENAMELEN)
#define PI 3.14159265358979323846264
#define INVALID_USB_DEVICE ((libusb_device_handle*)-1)

/*****************************************************************************
;*
;*		typedefs
;*
;****************************************************************************/

typedef struct {
    char Name[MAXFUGINAME];
    char HwCode[MAXFUGINAME];
} FUGImodel;

typedef struct FUGIstruct {
    char Name[MAXFUGINAME];
    int Number;

    char HwCode[MAXFUGINAME];

    uint32_t ReadError;
    uint32_t WriteError;

    libusb_device_handle* Handle;
    struct libusb_context* Context;
} FUGIdevice;

/*****************************************************************************
;*
;*		prototypes
;*
;****************************************************************************/

static inline uint8_t ConvertENCODERPWM(float frequency, float ratio, uint32_t* count_on, uint32_t* count_off);

/*****************************************************************************
;*
;*		global variables
;*
;****************************************************************************/

static FUGImodel FUGImodels[] = {
    "MOPS", "(0.0)", "BALLPLATE", "(0.1)", "HUBOAT", "(0.2)", "PENDULUM", "(0.3)",
    "ENCODERPWM", "(0.4)", "DRIFTER", "(0.5)",
};

static int FUGIcount = 0;
static FUGIdevice Port[MAXFUGIES];

/*****************************************************************************
;*
;*		GETSTRINGPORT
;*		get port pointer by string or FUGIdevice object
;*
;*		Input:	obj = string or possible FUGIdevice object
;*		Output:	FUGIdevice pointer
;*
;****************************************************************************/

static fugi_device GetStringPort(char const *name, int const device_number) {
    int i;

    /* Try to find the name in the list of already active FUGIdevices */
    for (i = 0; i < FUGIcount; i++) {
        if (strcasecmp(Port[i].Name, name) == 0 && Port[i].Number == device_number) {
            return &Port[i];
        }
    }

    /* Not an active board, see if there is room for another one */
    if (FUGIcount >= MAXFUGIES) {
        ROS_ERROR("Too many FUGIBoard devices referenced.");
        return NULL;
    }

    /* There is room, try to match the name to the list of supported models */
    for (i = 0; i < sizeof(FUGImodels) / sizeof(FUGImodel); i++) {
        if (strncasecmp(name, FUGImodels[i].Name, strlen(FUGImodels[i].Name)) == 0) {
            /* Found model, check the device number */
            if (device_number < 0 || device_number >= MAXFUGIES) {
                ROS_ERROR("FUGIBoard %d number is incorrect.", device_number);
                return NULL;
            }

            /* Everything checked out, add board to list of active boards */
            memset(&Port[FUGIcount], 0, sizeof(FUGIdevice));
            strncpy(Port[FUGIcount].Name, name, MAXFUGINAME);
            strncpy(Port[FUGIcount].HwCode, FUGImodels[i].HwCode, MAXFUGINAME);
            Port[FUGIcount].Number = device_number;
            Port[FUGIcount].Handle = INVALID_USB_DEVICE;
            return &Port[FUGIcount++];
        }
    }

    /* Didn't find existing device or model, so it's invalid */
    ROS_ERROR("Invalid FUGIBoard name.");
    return NULL;
}

/*****************************************************************************
;*
;*		USB Functions
;*		highly system dependent code to open and access USB devices
;*
;****************************************************************************/

#define MAX_INTEL_HEX_RECORD_LENGTH 16
typedef struct _INTEL_HEX_RECORD {
    uint32_t Length;
    uint32_t Address;
    uint32_t Type;
    uint8_t Data[MAX_INTEL_HEX_RECORD_LENGTH];
} INTEL_HEX_RECORD;

static int FX2Write(libusb_device_handle* handle, uint32_t address, uint16_t length, uint8_t* data) {
    int result;

    if (length == 0) return EOK;
    result = libusb_control_transfer(handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                     0xA0, address & 0xFFFF, address >> 16, data, length, 1000);
    if (result != length) return EIO;
    return EOK;
}

static int hexRead(INTEL_HEX_RECORD* record, FILE* hexFile) {
    int c;
    uint16_t i;
    int n, c1, check, len;

    while (((c = getc(hexFile)) != EOF) && c != ':') {
        n = fscanf(hexFile, "%*s\n");
    }
    if (c == EOF) return ENODATA;

    if (fscanf(hexFile, "%2X%4X%2X", &record->Length, &record->Address, &record->Type) != 3) return ENODATA;

    if ((len = record->Length) > MAX_INTEL_HEX_RECORD_LENGTH) return ENODATA;

    for (i = 0; i < len; i++) {
        n = fscanf(hexFile, "%2X", &c1);
        if (n != 1) {
            if (i != record->Length) {
                return ENODATA;
            }
        }
        record->Data[i] = c1;
    }

    if (fscanf(hexFile, "%2X\n", &check) != 1) return ENODATA;

    return EOK;
}

static int FX2LoadRam(libusb_device_handle* handle, FILE* image) {
    INTEL_HEX_RECORD hexLine;
    uint8_t byteVal;
    int result;

    /* Make sure we start at the beginning of the image file */
    rewind(image);

    /* Stop the FX2 CPU */
    byteVal = 1;
    if (FX2Write(handle, 0xE600, 1, &byteVal) != EOK) return EIO;

    // Download code
    while (1) {
        result = hexRead(&hexLine, image);
        if (hexLine.Type != 0) break;
        if (result != EOK) return EIO;
        if (FX2Write(handle, hexLine.Address, hexLine.Length, hexLine.Data) != EOK) return EIO;
    }

    /* De-assert reset, ignore errors, we can't do anything anymore */
    byteVal = 0;
    FX2Write(handle, 0xE600, 1, &byteVal);
    return EOK;
}

static int DownloadFirmware(fugi_device port) {
    char* path2hex;
    FILE* hexfile;
    int sleeptime = 0;
    libusb_device** list;
    libusb_device_handle* handle;
    ssize_t cnt;
    int status = EOK;
    struct libusb_device_descriptor desc;

    /* Download firmware in uninitialized FX2 chips if a firmware image can be
     * found */
    if (((path2hex = getenv("DCSCUSB_HEX_FILE")) != NULL) && ((hexfile = fopen(path2hex, "rb")) != NULL)) {
        /* Get a list of all USB devices or skip the initialization if we can't
         * get a list */
        if ((cnt = libusb_get_device_list(port->Context, &list)) > 0) {
            /* Look for uninitialized devices in the list and initialize those
             */
            for (int i = 0; i < cnt; i++) {
                if (libusb_get_device_descriptor(list[i], &desc) == 0) {
                    if (desc.idVendor == 0x04B4 && desc.idProduct == 0x8613) {
                        if (libusb_open(list[i], &handle) == 0) {
                            status = EOK;
                            if (libusb_claim_interface(handle, 0) == 0) {
                                status = FX2LoadRam(handle, hexfile);
                                libusb_release_interface(handle, 0);
                            }
                            libusb_close(handle);
                            if (status != EOK) break;
                            sleeptime = 3;
                        }
                    }
                }
            }

            /* We're done with the list of devices */
            libusb_free_device_list(list, 1);
        }

        /* Give the devices some time to renumerate if we initialized any device
         */
        if (sleeptime) {
            ROS_WARN("FX2 device(s) initialized, waiting for re-enumeration.\n");
            sleep(sleeptime);
        }

        /* We're done with the file */
        fclose(hexfile);
    }

    return status;
}

static int USBopen(fugi_device port) {
    char* hwcode;
    char message[200];
    int deviceCount = 0;
    int device_number = port->Number;
    int result;
    libusb_device** list;
    libusb_device_handle* found = NULL;
    libusb_device_handle* handle;
    ssize_t cnt;
    struct libusb_device_descriptor desc;
    unsigned char boardName[200];

    /* Open a context for the USB driver if not open yet */
    if (port->Context == NULL) {
        if ((result = libusb_init(&port->Context)) < 0) {
            ROS_ERROR("Unable to create USB context: %s.\n", libusb_error_name(result));
            return ENOTCONN;
        }
    }

    /* Download firmware to unintialized FUGIboards */
    if (DownloadFirmware(port) != EOK) {
        ROS_ERROR("Unable to load firmware into FX2 chips.\n");
        return ENOTCONN;
    }

    /* Get a list of all USB devices */
    if ((cnt = libusb_get_device_list(port->Context, &list)) < 1) {
        libusb_exit(port->Context);
        port->Context = NULL;
        if (cnt < 0) {
            ROS_ERROR("Unable to get list of USB devices: %s.\n", libusb_error_name(result));
            return ENOTCONN;
        } else if (cnt == 0) {
            /* no USB devices at all */
            ROS_ERROR("No USB devices found!");
            return ENOTCONN;
        }
    }

    /* Look for the desired devices in the list, count them and find the
     * requested number */
    for (int i = 0; i < cnt; i++) {
        if (libusb_get_device_descriptor(list[i], &desc) == 0) {
            if (libusb_open(list[i], &handle) == 0) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iProduct, boardName, sizeof(boardName)) > 0) {
                    if ((hwcode = strrchr((char*)boardName, '(')) != NULL) {
                        /* Is this the desired device type? */
                        if (strcasecmp(hwcode, port->HwCode) == 0) {
                            /* This is a desired device, but is it the right
                             * one? */
                            if (deviceCount == device_number) {
                                /* Yes, it is */
                                if ((result = libusb_claim_interface(handle, 0)) < 0) {
                                    ROS_ERROR("Unable to claim USB interface: %s.\n", libusb_error_name(result));
                                    return ENOTCONN;
                                }
                                port->Handle = handle;
                                libusb_free_device_list(list, 1);
                                return EOK;
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
    return ENOTCONN;
}

static int USBread(fugi_device port, void* buffer, uint32_t len) {
    unsigned int timeout = 0;
    int halflen = len / 2;
    int firsthalftransferred, secondhalftransferred;
    int result;

    /* First read, to trigger the updating of the data in the FPGA */
    result = libusb_bulk_transfer(port->Handle, 0x86, (unsigned char*)buffer, halflen, &firsthalftransferred, timeout);
    if (result != 0) {
        port->ReadError = result;
        return EIO; /* USB operation failed */
    }

    /* Second read, the retreive the fresh data from the FPGA */
    result = libusb_bulk_transfer(port->Handle, 0x86, ((unsigned char*)buffer) + halflen, halflen,
                                  &secondhalftransferred, timeout);
    if (result != 0) {
        port->ReadError = result;
        return EIO; /* USB operation failed */
    }

    /* Make sure we got all data */
    if ((firsthalftransferred + secondhalftransferred) != len) {
        return ENODATA; /* wrong number of bytes read */
    }
    port->ReadError = 0;
    return EOK; /* everything okay */
}

static int USBwrite(fugi_device port, void* data, int len) {
    unsigned int timeout = 100;
    int transferred, result;

    /* Read the data from the device */
    result = libusb_bulk_transfer(port->Handle, 0x02, (unsigned char*)data, len, &transferred, timeout);

    /* Chek the result */
    if (result != 0) {
        port->WriteError = result;
        return EIO; /* USB operation failed */
    } else if (transferred != len) {
        return ENODATA; /* wrong number of bytes read */
    }
    port->WriteError = 0;
    return EOK; /* everything okay */
}

static void USBclose(fugi_device port) {
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
#define GETPOSITION(t, s, fp, pc)                                \
    if (s##h < 128) {                                            \
        temp = ((0 << 24) + (s##h << 16) + (s##m << 8) + s##l);  \
    } else {                                                     \
        temp = ((-1 << 24) + (s##h << 16) + (s##m << 8) + s##l); \
    }                                                            \
    t = (2.0 * PI * ((double)temp)) / (fp * pc)
#define GETPOSITION16(t, s, fp, pc)               \
    if (s##h < 128) {                             \
        temp = ((0 << 16) + (s##h << 8) + s##l);  \
    } else {                                      \
        temp = ((-1 << 16) + (s##h << 8) + s##l); \
    }                                             \
    t = (2.0 * PI * ((double)temp)) / (fp * pc)
#define GETRAWPOSITION20ODD(t, s)                         \
    temp = ((s##b3 & 0x0F) << 16) + (s##b2 << 8) + s##b1; \
    if (temp & 0x00080000) temp |= 0xFFF00000;            \
    t = (double)temp
#define GETRAWPOSITION20EVEN(t, s)                               \
    temp = (s##b5 << 12) + (s##b4 << 4) + ((s##b3 & 0xF0) >> 4); \
    if (temp & 0x00080000) temp |= 0xFFF00000;                   \
    t = (double)temp

#define GETSPEED(t, s, st, cl, pc)                    \
    temp = ((s##h << 16) + (s##m << 8) + s##l);       \
    if (temp == 0x00FFFFFF) {                         \
        t = 0.0;                                      \
    } else if (st < 128) {                            \
        t = ((2.0 * PI * cl) / pc) / ((double)temp);  \
    } else {                                          \
        t = -((2.0 * PI * cl) / pc) / ((double)temp); \
    }
#define GETADC(t, s, i, a)       \
    temp = ((s##h << 8) + s##l); \
    t = (((a - i) * ((double)temp)) / 0xFFFF) + i
#define GETSADC(t, s, i, a) \
    temp = (s);             \
    t = (((a - i) * ((double)temp)) / 0xFF) + i

#define SETBYTE(t, s)             \
    temp = (int32_t)(s);          \
    if (temp > 0xFF) temp = 0xFF; \
    if (temp < 0) temp = 0;       \
    t = (uint8_t)temp
#define SETWORD(t, s)                 \
    temp = (int32_t)(s);              \
    if (temp > 0xFFFF) temp = 0xFFFF; \
    if (temp < 0) temp = 0;           \
    t##l = (uint8_t)(temp & 0xFF);    \
    t##h = (uint8_t)((temp & 0xFF00) >> 8)
#define SETDAC(t, s, i, a)                                        \
    temp = (int32_t)((((s) - (i)) * 0xFFFF) / ((a) - (i)) + 0.5); \
    if (temp > 0xFFFF) temp = 0xFFFF;                             \
    if (temp < 0) temp = 0;                                       \
    t##l = (uint8_t)(temp & 0xFF);                                \
    t##h = (uint8_t)((temp & 0xFF00) >> 8)
#define SETSERVO(t, a, v)             \
    if ((a) < 0) {                    \
        temp = 0;                     \
    } else if ((a) > 63) {            \
        temp = 0;                     \
    } else {                          \
        temp = ((a) << 10);           \
    }                                 \
    if (v < -500) {                   \
        temp += 0;                    \
    } else if (v > 500) {             \
        temp += 1000;                 \
    } else {                          \
        temp += v + 500;              \
    }                                 \
    t.datal = (uint8_t)(temp & 0xFF); \
    t.datah = (uint8_t)((temp & 0xFF00) >> 8)

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the MOPS
;*
;*
;****************************************************************************/

int FUGIMops::read() {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t positionl, positionm, positionh;
        uint8_t speedl, speedm, speedh;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t voltagel, voltageh;
        uint8_t externl, externh;
        uint8_t flags;
    } raw_data;
    int status;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    /* Get the flags */
    GETBYTE(sensors.flags, raw_data.flags, 0x7F);

    /* Get the time since the previous sample and remember its raw value */
    GETRELTIME(sensors.relative_time, raw_data.reltime, RELTIMERES);

    /* Get the postion from the last measurement in this sample and remember its
     * raw value */
    GETPOSITION(sensors.position0, raw_data.position, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the momentary speed */
    GETSPEED(sensors.speed, raw_data.speed, raw_data.flags, CLOCK, PULSESPERCIRCLE);

    /* Get the motor current */
    GETADC(sensors.current, raw_data.current, -3.0, 3.0);

    /* Get the motor voltage */
    GETADC(sensors.voltage, raw_data.voltage, -12.5, 12.5);

    /* Get the extern voltage */
    GETADC(sensors.external_voltage, raw_data.voltage, -10.0, 10.0);

    /* Get the digital inputs */
    GETBYTE(sensors.digital_inputs, raw_data.digin, 0xFF);

    return EOK;
}

int FUGIMops::write() {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } raw_data;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    // /* Convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.digout, actuators.digital_outputs);

    /* Set the first DAC value */
    SETDAC(raw_data.dac0, actuators.voltage0, -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(raw_data.dac1, actuators.voltage1, -10.0, 10.0);

    /* Set the timeout */
    SETWORD(raw_data.timeout, actuators.timeout / WATCHDOGRES);

    /* Now write it */
    return USBwrite(device, &raw_data, sizeof(raw_data));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the Ball-and-Plate
;*
;*
;****************************************************************************/

int FUGIBallPlate::read() {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position0l, position0m, position0h;
        uint8_t position1l, position1m, position1h;
        uint8_t digin;
        uint8_t voltageXl, voltageXh;
        uint8_t voltageYl, voltageYh;
        uint8_t sadcX, sadcY;
        uint8_t flags;
    } raw_data;
    int status;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    /* Get the flags */
    GETBYTE(sensors.flags, raw_data.flags, 0x7F);

    /* Get the time since the previous sample */
    GETRELTIME(sensors.relative_time, raw_data.reltime, RELTIMERES);

    /* Get the postion from the first encoder */
    GETPOSITION(sensors.position0, raw_data.position0, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the postion from the second encoder */
    GETPOSITION(sensors.position1, raw_data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the voltage from the inclinometer X and Y*/
    GETADC(sensors.inclinometer_x, raw_data.voltageX, 0.0, 5.0);
    GETADC(sensors.inclinometer_y, raw_data.voltageY, 0.0, 5.0);

    /* Get the voltages from the joystick */
    GETSADC(sensors.joystick_x, raw_data.sadcX, 0.0, 3.3);
    GETSADC(sensors.joystick_y, raw_data.sadcY, 0.0, 3.3);

    /* Get the digital inputs */
    GETBYTE(sensors.digital_inputs, raw_data.digin, 0xFF);

    return EOK;
}

int FUGIBallPlate::write() {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } raw_data;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.digout, actuators.digital_outputs);

    /* DAC input value is between +12 and -12, move up to between +24 and 0,
     * scale and limit between 0xFFFF and 0x0000  */
    SETDAC(raw_data.dac0, actuators.voltage0, -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(raw_data.dac1, actuators.voltage1, -10.0, 10.0);

    /* Set the timeout */
    SETWORD(raw_data.timeout, actuators.timeout / WATCHDOGRES);

    /* Now write it */
    return USBwrite(device, &raw_data, sizeof(raw_data));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the HuBoat
;*
;*
;****************************************************************************/

int FUGIHuBoat::read() {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position0l, position0m, position0h;
        uint8_t position1l, position1m, position1h;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t voltagel, voltageh;
        uint8_t externl, externh;
        uint8_t flags;
    } raw_data;
    int status;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    /* Get the flags */
    GETBYTE(sensors.flags, raw_data.flags, 0x7F);

    /* Get the time since the previous sample */
    GETRELTIME(sensors.relative_time, raw_data.reltime, RELTIMERES);

    /* Get the postion from the first encoder */
    GETPOSITION(sensors.position0, raw_data.position0, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the postion from the second encoder */
    GETPOSITION(sensors.position1, raw_data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the motor current */
    GETADC(sensors.current, raw_data.current, -5.1, 5.1);

    /* Get the voltage from the first potentiometer */
    GETADC(sensors.voltage_pot0, raw_data.voltage, 0.0, 5.0);

    /* Get the voltage from the second potentiometer */
    GETADC(sensors.voltage_pot1, raw_data.extern, 0.0, 5.0);

    /* Get the digital inputs */
    GETBYTE(sensors.digital_inputs, raw_data.digin, 0xFF);

    return EOK;
}

int FUGIHuBoat::write() {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
        struct {
            uint8_t datal, datah;
        } servo[4];
    } raw_data;
    int status;
    int32_t temp;
    double dac2;
    int nextfull, i, address, value;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.digout, actuators.digital_outputs);

    /* DAC input value is between +12 and -12, move up to between +24 and 0,
     * scale and limit between 0xFFFF and 0x0000  */
    SETDAC(raw_data.dac0, actuators.voltage0, -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(raw_data.dac1, actuators.voltage1, -10.0, 10.0);

    /* Set the timeout */
    SETWORD(raw_data.timeout, actuators.timeout / WATCHDOGRES);

    /* Set the servo values */
    address = -1;
    nextfull = 4;
    for (i = 0; i < actuators.servo_count;) {
        temp = actuators.servos[i].address;
        address = (temp == -1) ? address + 1 : temp;
        value = actuators.servos[i].value;
        SETSERVO(raw_data.servo[i - nextfull + 4], address, value);

        /* Write data if data full or no more data */
        i += 1;
        if ((i == nextfull) || (i == actuators.servo_count)) {
            if ((status = USBwrite(device, &raw_data, sizeof(raw_data))) != EOK) {
                return status;
            }
            memset(&raw_data.servo, 0, sizeof(raw_data.servo));
            nextfull += 4;
        }
    }
    return EOK;
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the Rotating Pendulum 2
;*
;*
;****************************************************************************/

int FUGIPendulum::read() {
    struct {
        uint8_t dummy[16];
        uint8_t reltimel, reltimeh;
        uint8_t position0l, position0m, position0h;
        uint8_t position1l, position1m, position1h;
        uint8_t digin;
        uint8_t currentl, currenth;
        uint8_t beaml, beamh;
        uint8_t penduluml, pendulumh;
        uint8_t flags;
    } raw_data;
    int status;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    /* Get the flags */
    GETBYTE(sensors.flags, raw_data.flags, 0x7F);

    /* Get the time since the previous sample */
    GETRELTIME(sensors.relative_time, raw_data.reltime, RELTIMERES);

    /* Get the postion from the first encoder */
    GETPOSITION(sensors.position0, raw_data.position0, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the postion from the second encoder */
    GETPOSITION(sensors.position1, raw_data.position1, FLANKSPERPULSE, PULSESPERCIRCLE);

    /* Get the motor current */
    GETADC(sensors.current, raw_data.current, -5.1, 5.1);

    /* Get the voltage from the beam potentiometer */
    GETADC(sensors.voltage_beam, raw_data.beam, 0.0, 5.0);

    /* Get the voltage from the pendulum potentiometer */
    GETADC(sensors.voltage_pendulum, raw_data.pendulum, 0.0, 5.0);

    /* Get the digital inputs */
    GETBYTE(sensors.digital_inputs, raw_data.digin, 0xFF);

    return EOK;
}

int FUGIPendulum::write() {
    struct {
        uint8_t cmd;
        uint8_t dac0l, dac0h;
        uint8_t dac1l, dac1h;
        uint8_t digout;
        uint8_t timeoutl, timeouth;
    } raw_data;
    int32_t temp;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* convert the data and put it in the data buffer */

    /* Just limit command byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);

    /* Just limit digital output byte between 0xFF and 0x00 (inclusive) */
    SETBYTE(raw_data.digout, actuators.digital_outputs);

    /* DAC input value is between +12 and -12, move up to between +24 and 0,
     * scale and limit between 0xFFFF and 0x0000  */
    SETDAC(raw_data.dac0, actuators.voltage0, -10.0, 10.0);

    /* Set the second DAC value */
    SETDAC(raw_data.dac1, actuators.voltage1, -10.0, 10.0);

    /* Set the timeout */
    SETWORD(raw_data.timeout, actuators.timeout / WATCHDOGRES);

    /* Now write it */
    return USBwrite(device, &raw_data, sizeof(raw_data));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the EncoderPWM
;*
;*
;****************************************************************************/

int FUGIEncoderPWM::read() {
    struct {
        uint8_t dummy[16];
        uint8_t pospair1b1, pospair1b2, pospair1b3, pospair1b4, pospair1b5;
        uint8_t pospair2b1, pospair2b2, pospair2b3, pospair2b4, pospair2b5;
        uint8_t pospair3b1, pospair3b2, pospair3b3, pospair3b4, pospair3b5;
        uint8_t flags;
    } raw_data;
    int status;
    int32_t temp;
    double* ptr;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    /* Get the flags */
    GETBYTE(sensors.flags, raw_data.flags, 0x7F);

    /* Get the postion from the first encoder */
    GETRAWPOSITION20ODD(sensors.encoder0, raw_data.pospair1);

    /* Get the postion from the second encoder */
    GETRAWPOSITION20EVEN(sensors.encoder1, raw_data.pospair1);

    /* Get the postion from the third encoder */
    GETRAWPOSITION20ODD(sensors.encoder2, raw_data.pospair2);

    /* Get the postion from the fourth encoder */
    GETRAWPOSITION20EVEN(sensors.encoder3, raw_data.pospair2);

    /* Get the postion from the fifth encoder */
    GETRAWPOSITION20ODD(sensors.encoder4, raw_data.pospair3);

    /* Get the postion from the sixth encoder */
    GETRAWPOSITION20EVEN(sensors.encoder5, raw_data.pospair3);

    return EOK;
}

int FUGIEncoderPWM::write() {
    struct {
        uint8_t cmd;
        uint8_t ontime0l, ontime0h;
        uint8_t offtime0l, offtime0h;
        uint8_t ontime1l, ontime1h;
        uint8_t offtime1l, offtime1h;
        uint8_t ontime2l, ontime2h;
        uint8_t offtime2l, offtime2h;
    } raw_data;
    int32_t temp;
    double frequency, ratio;
    uint32_t count_total, count_on;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Just limit command byte between 0x0F and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);
    raw_data.cmd &= 0x0F;

    /* Check and process frequency parameter */
    frequency = actuators.frequency;
    if (frequency < 50.0 || frequency > 25000.0) {
        ROS_ERROR("Frequency %g out of range.", frequency);
        return EINVAL;
    }
    count_total = CLOCK / (frequency * PWMCLOCKDIVIDER);

    /* Process first ratio parameter */
    ratio = actuators.pwm0;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 0 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 0;
    SETWORD(raw_data.ontime0, count_total);
    SETWORD(raw_data.offtime0, count_on);

    /* Process second ratio parameter */
    ratio = actuators.pwm1;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 1 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 1;
    SETWORD(raw_data.ontime1, count_total);
    SETWORD(raw_data.offtime1, count_on);

    /* Process third ratio parameter */
    ratio = actuators.pwm2;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 2 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 2;
    SETWORD(raw_data.ontime2, count_total);
    SETWORD(raw_data.offtime2, count_on);

    /* Now write it */
    return USBwrite(device, &raw_data, sizeof(raw_data));
}

/*****************************************************************************
;*
;*
;*		The Read and Write functions for the Drifter (absolute and incremental encoders, servo PWM)
;*
;*
;****************************************************************************/

int FUGIDrifter::read() {
    struct __attribute__((packed)) {
        uint8_t dummy[16];
        int incr0: 24;
        int incr1: 24;
        int incr2: 24;
        u_int abs0: 12;
        u_int abs1: 12;
        u_int abs2: 12;
        u_int stamp: 20;
    } raw_data;
    int status;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Read the data from the device */
    if ((status = USBread(device, &raw_data, 32)) != EOK) {
        return status;
    }

    sensors.incremental0 = raw_data.incr0;
    sensors.incremental1 = raw_data.incr1;
    sensors.incremental2 = raw_data.incr2;

    sensors.absolute0 = raw_data.abs0;
    sensors.absolute1 = raw_data.abs1;
    sensors.absolute2 = raw_data.abs2;

    sensors.stamp = raw_data.stamp;

    return EOK;
}

int FUGIDrifter::write() {
    struct {
        uint8_t cmd;
        uint8_t totalcount0l, totalcount0h;
        uint8_t oncount0l, oncount0h;
        uint8_t totalcount1l, totalcount1h;
        uint8_t oncount1l, oncount1h;
        uint8_t totalcount2l, totalcount2h;
        uint8_t oncount2l, oncount2h;
    } raw_data;
    int32_t temp;
    double frequency, ratio;
    uint32_t count_total, count_on;

    /* Verify that the device is open */
    if (device == NULL || device->Handle == INVALID_USB_DEVICE || device->Context == NULL) {
        return ENOTCONN;
    }

    /* Just limit command byte between 0x0F and 0x00 (inclusive) */
    SETBYTE(raw_data.cmd, actuators.command);
    raw_data.cmd &= 0x0F;

    /* Check and process frequency parameter */
    frequency = actuators.frequency;
    if (frequency < 50.0 || frequency > 25000.0) {
        ROS_ERROR("Frequency %g out of range.", frequency);
        return EINVAL;
    }
    count_total = CLOCK / (frequency * PWMCLOCKDIVIDER);

    /* Process first ratio parameter */
    ratio = actuators.ratio0;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 0 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 0;
    SETWORD(raw_data.totalcount0, count_total);
    SETWORD(raw_data.oncount0, count_on);

    /* Process second ratio parameter */
    ratio = actuators.ratio1;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 1 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 1;
    SETWORD(raw_data.totalcount1, count_total);
    SETWORD(raw_data.oncount1, count_on);

    /* Process third ratio parameter */
    ratio = actuators.ratio2;
    if (ratio < -1.0 || ratio > 1.0) {
        ROS_ERROR("Ratio 2 value %g out of range.", ratio);
        return EINVAL;
    }
    raw_data.cmd |= (ConvertENCODERPWM(frequency, ratio, &count_total, &count_on)) << 2;
    SETWORD(raw_data.totalcount2, count_total);
    SETWORD(raw_data.oncount2, count_on);

    /* Now write it */
    return USBwrite(device, &raw_data, sizeof(raw_data));
}

static inline uint8_t ConvertENCODERPWM(float frequency, float ratio, uint32_t* count_total, uint32_t* count_on) {
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
    return result;
}

/*****************************************************************************
******************************************************************************

Interface actions

******************************************************************************
*****************************************************************************/

/*****************************************************************************
;*
;*		FUGI_CLOSE
;*		close FUGIdevice
;*
;*      Input:  port = FUGIdevice
;*      Output: none
;*
;****************************************************************************/

int fugi_close(fugi_device port) {
    int status;

    /* Close the port */
    if (port->Handle != INVALID_USB_DEVICE) {
        status = EOK;
        USBclose(port);
    } else {
        status = ENOTCONN;
    }

    return status;
}

/*****************************************************************************
;*
;*		FUGI_CLOSEALL
;*		close all FUGIdevice ports
;*
;*      Input:  none
;*      Output: status (always EOK)
;*
;****************************************************************************/

int fugi_close_all() {
    int i;

    /* Close all ports */
    for (i = 0; i < FUGIcount; i++) fugi_close(&Port[i]);

    return EOK;
}

FUGIBoard::FUGIBoard(char const *device_name, int const device_number) {
    int status;
    fugi_device dev = GetStringPort(device_name, device_number);
    if (dev == NULL) {
        device = NULL;
    } else {
        status = USBopen(dev);
        if (status != EOK) {
            device = NULL;
        } else {
            device = dev;
        }
    }
}

FUGIBoard::~FUGIBoard() {
    if (device != NULL) {
        USBclose(device);
        device = NULL;
    }
}

int FUGIBoard::read_error() {
    if (device == NULL) return ENOTCONN;
    return device->ReadError;
}

int FUGIBoard::write_error() {
    if (device == NULL) return ENOTCONN;
    return device->WriteError;
}
