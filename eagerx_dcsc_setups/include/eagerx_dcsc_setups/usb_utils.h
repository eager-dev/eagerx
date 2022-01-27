#ifndef USB_UTILS_H
#define USB_UTILS_H

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#ifndef EOK
#define EOK 0
#endif

typedef struct FUGIstruct* fugi_device;

class FUGIBoard {
  public:
    FUGIBoard(char const *device_name, int const device_number);
    ~FUGIBoard();
    bool is_open() { return (device != NULL); };
    int close();
    int read_error();
    int write_error();

  protected:
    fugi_device device;
};

class FUGIMops : public FUGIBoard {
  public:
    FUGIMops(int device_number) : FUGIBoard("mops", device_number) {}
    int read();
    int write();
    struct {
        uint8_t command;
        uint8_t digital_outputs;
        double voltage0, voltage1;
        double timeout = 1.0;
    } actuators;
    struct {
        uint8_t flags;
        double relative_time;
        double position0, position1;
        double speed;
        double voltage, current;
        double external_voltage;
        uint8_t digital_inputs;
    } sensors;
};

class FUGIBallPlate : public FUGIBoard {
  public:
    FUGIBallPlate(int device_number) : FUGIBoard("mops", device_number){};
    int read();
    int write();
    struct {
        uint8_t command;
        uint8_t digital_outputs;
        double voltage0, voltage1;
        double timeout = 1.0;
    } actuators;
    struct {
        uint8_t flags;
        double relative_time;
        double position0, position1;
        double inclinometer_x, inclinometer_y;
        double joystick_x, joystick_y;
        uint8_t digital_inputs;
    } sensors;
};

class FUGIHuBoat : public FUGIBoard {
  public:
    FUGIHuBoat(int device_number) : FUGIBoard("huboat", device_number){};
    int read();
    int write();
    struct {
        uint8_t command;
        uint8_t digital_outputs;
        double voltage0, voltage1;
        double timeout = 1.0;
        uint8_t servo_count;
        struct {
            int8_t address;
            uint16_t value;
        } servos[64];
    } actuators;

    struct {
        uint8_t flags;
        double relative_time;
        double position0, position1;
        double current;
        double voltage_pot0, voltage_pot1;
        uint8_t digital_inputs;
    } sensors;
};

class FUGIPendulum : public FUGIBoard {
  public:
    FUGIPendulum(int device_number) : FUGIBoard("pendulum", device_number){};
    int read();
    int write();
    struct {
        uint8_t command;
        uint8_t digital_outputs;
        double voltage0, voltage1;
        double timeout = 1.0;
    } actuators;
    struct {
        uint8_t flags;
        double relative_time;
        double position0, position1;
        double digital_inputs;
        double current;
        double voltage_beam, voltage_pendulum;
    } sensors;
};

class FUGIEncoderPWM : public FUGIBoard {
  public:
    FUGIEncoderPWM(int device_number) : FUGIBoard("encoderpwm", device_number){};
    int read();
    int write();
    struct {
        uint8_t command;
        double frequency;
        double pwm0, pwm1, pwm2, pwm3;
    } actuators;
    struct {
        uint8_t flags;
        double encoder0, encoder1, encoder2, encoder3, encoder4, encoder5;
    } sensors;
};

class FUGIDrifter : public FUGIBoard {
  public:
    FUGIDrifter(int device_number) : FUGIBoard("drifter", device_number){};
    int read();
    int write();
    struct {
        uint8_t command;
        double frequency;
        double ratio0, ratio1, ratio2;
    } actuators;
    struct {
        uint32_t stamp;
        double absolute0, absolute1, absolute2, incremental0, incremental1, incremental2;
    } sensors;
};

#endif