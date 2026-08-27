#include "BNO055.h"
#include "mbed.h"

BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    : IMU(), _i2c(i2c), chip_addr(addr), _res(p_reset), cantReadDataCount(0)
{
    _i2c.frequency(100000);

    _res.write(0);
    ThisThread::sleep_for(10ms);
    _res.write(1);

    ThisThread::sleep_for(650ms);

}
void BNO055 :: init() noexcept {

    char config_mode[] ={0x3D, 0x00};
    _i2c.write(chip_addr << 1, config_mode, 2, false);
    ThisThread :: sleep_for(19ms);

    char dof_mode[]  = {0x3D, 0x0C};
    _i2c.write(chip_addr << 1, dof_mode, 2, false);
    ThisThread :: sleep_for(9ms);

}
void BNO055 :: reset() noexcept{
    _res.write(0);
    ThisThread :: sleep_for(10ms);

    _res.write(1);
    ThisThread :: sleep_for(650ms);

    init();
}
void BNO055 :: get_accel(BNO055_VECTOR_TypeDef *la) {
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }

    char accel[] = {0x08};
    int16_t x,y,z;
    int writeResult = _i2c.write(chip_addr << 1, accel, 1, true);

    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
    
        _i2c.read(chip_addr << 1 | 1 , dt, 6, false);
        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        la->x = (double)x / 100;
        la->y = (double)y / 100;
        la->z = (double)z / 100;
    }
    else{
        cantReadDataCount ++;
    }
 
}
//The gyro function I didnt copy the cant read data count from get quarternion
void BNO055 :: get_gyro(BNO055_VECTOR_TypeDef *gr){
    int16_t x,y,z;
    char gyro[] = {0x14};

    _i2c.write(chip_addr << 1, gyro, 1, true);
    _i2c.read(chip_addr << 1 | 1, dt, 6, false);

    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];

    gr -> x = (double)x / 16.0;
    gr -> y = (double)y / 16.0;
    gr -> z = (double)z / 16.0;
}
//read override
IMU :: EulerAngles BNO055 ::  read() {
    get_angular_position_quat(&imuAngles);
    return imuAngles;

}
IMU :: EulerAngles BNO055 :: getImuAngles(){
    return imuAngles;
}
//change fusion mode
void BNO055 :: change_fusion_mode(uint8_t mode){
    //first change to config mode
    char change[] = {0x3D, 0X00};
    _i2c.write(chip_addr << 1, change, 2, false);
    ThisThread :: sleep_for(19ms);

    //then change to the mode
    char change_to_mode[] = {0x3D, (char)mode};
    _i2c.write(chip_addr << 1, change_to_mode, 2, false);
    ThisThread :: sleep_for(9ms);
}


/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}