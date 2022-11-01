#include "pca9685.h"

PCA9685::PCA9685(i2c_inst_t *i2cInst, uint8_t i2c_addr)
{
    _i2cInst = i2cInst;
    _i2c_addr = i2c_addr;
}

PCA9685::~PCA9685()
{
    if (i2c_dev != NULL)
    {
        delete i2c_dev;
    }
}

void PCA9685::begin(uint8_t prescale)
{
    i2c_dev = new I2CDevice(_i2c_addr, _i2cInst);
    i2c_dev->begin();

    reset();

    if (prescale > 0)
    {
        setExtClk(prescale);
    }
    else
    {
        setPWMFreq(1000);
    }

    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
}

void PCA9685::reset()
{
    BusRegister pca9685_mode_1 =
        BusRegister(i2c_dev, PCA9685_MODE1, 1);
    pca9685_mode_1.write(MODE1_RESTART);
}

void PCA9685::sleep()
{
    BusRegister pca9685_mode_1 =
        BusRegister(i2c_dev, PCA9685_MODE1, 1);

    uint8_t awake = pca9685_mode_1.read() & 0xFF;
    uint8_t sleep = awake | MODE1_SLEEP;

    pca9685_mode_1.write(sleep);
}

void PCA9685::wakeup()
{
    BusRegister pca9685_mode_1 =
        BusRegister(i2c_dev, PCA9685_MODE1, 1);

    uint8_t sleep = pca9685_mode_1.read() & 0xFF;
    uint8_t wakeup = sleep & ~MODE1_SLEEP;

    pca9685_mode_1.write(wakeup);
}

void PCA9685::setExtClk(uint8_t prescale)
{
    BusRegister pca9685_mode_1 =
        BusRegister(i2c_dev, PCA9685_MODE1, 1);
    BusRegister pca9685_prescale =
        BusRegister(i2c_dev, PCA9685_PRESCALE, 1);

    uint8_t oldmode = pca9685_mode_1.read() & 0xFF;
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;

    pca9685_mode_1.write(newmode);
    pca9685_mode_1.write((newmode |= MODE1_EXTCLK));

    pca9685_prescale.write(prescale);

    pca9685_mode_1.write((newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

void PCA9685::setPWMFreq(float freq)
{
    BusRegister pca9685_mode_1 =
        BusRegister(i2c_dev, PCA9685_MODE1, 1);
    BusRegister pca9685_prescale =
        BusRegister(i2c_dev, PCA9685_PRESCALE, 1);

    float _freq = freq;
    if (_freq < 1)
    {
        _freq = 1;
    }

    if (_freq > 3500)
    {
        _freq = 3500;
    }

    float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
    {
        prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX)
    {
        prescaleval = PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = pca9685_mode_1.read() & 0xFF;
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep

    pca9685_mode_1.write(newmode);
    pca9685_prescale.write(prescale);
    pca9685_mode_1.write(oldmode);

    pca9685_mode_1.write(oldmode | MODE1_RESTART | MODE1_AI);
}

void PCA9685::setOscillatorFrequency(uint32_t freq)
{
    _oscillator_freq = freq;
}

uint32_t PCA9685::getOscillatorFrequency()
{
    return _oscillator_freq;
}

void PCA9685::setOutputMode(bool totempole)
{
    BusRegister pca9685_mode_2 =
        BusRegister(i2c_dev, PCA9685_MODE2, 1);

    uint8_t oldmode = pca9685_mode_2.read() & 0xFF;
    uint8_t newmode;

    if (totempole)
    {
        newmode = oldmode | MODE2_OUTDRV;
    }
    else
    {
        newmode = oldmode & ~MODE2_OUTDRV;
    }

    pca9685_mode_2.write(newmode);
}

uint8_t PCA9685::getPWM(uint8_t num)
{
    BusRegister pca9685_led0_on_l =
        BusRegister(i2c_dev, PCA9685_LED0_ON_L + 4 * num, 1);

    return pca9685_led0_on_l.read() & 0xFF;
}

uint8_t PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    BusRegister pca9685_led0_on_l =
        BusRegister(i2c_dev, PCA9685_LED0_ON_L + 4 * num, 1);

    uint8_t buffer[4] = {
        (uint8_t)(on & 0xff),
        (uint8_t)(on >> 8),
        (uint8_t)(off & 0xff),
        (uint8_t)(off >> 8)};

    pca9685_led0_on_l.write(buffer, 4);
    return 0;
}

void PCA9685::setPin(uint8_t num, uint16_t val, bool invert)
{
    uint16_t _val = val;

    if (invert)
    {
        if (_val == 0)
        {
            setPWM(num, 4096, 0);
        }
        else if (_val == 4095)
        {
            setPWM(num, 0, 4096);
        }
        else
        {
            setPWM(num, 0, 4095 - _val);
        }
    }
    else
    {
        if (_val == 4095)
        {
            setPWM(num, 4096, 0);
        }
        else if (_val == 0)
        {
            setPWM(num, 0, 4096);
        }
        else
        {
            setPWM(num, 0, _val);
        }
    }
}

uint8_t PCA9685::readPrescale()
{
    BusRegister pca9685_Pprescale =
        BusRegister(i2c_dev, PCA9685_PRESCALE, 1);

    return pca9685_Pprescale.read() & 0xFF;
}

void PCA9685::writeMicroseconds(uint8_t num, uint16_t us)
{
    double pulse = us;
    double pulselength = 1000000;

    // Read prescale
    uint16_t prescale = readPrescale();

    prescale += 1;

    pulselength *= prescale;
    pulselength /= _oscillator_freq;

    pulse /= pulselength;

    setPWM(num, 0, pulse);
}
