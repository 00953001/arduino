void ESP32MotorControl::attachMotor(uint8_t gpioIn1, uint8_t gpioIn2)
{
    attachMotors(gpioIn1, gpioIn2, 0, 0);
}

// Attach two motors
void ESP32MotorControl::attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4)
{
    // debug
    debug("init MCPWM Motor 0");

    // Attach motor 0 input pins.
    // Set MCPWM unit 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpioIn1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, gpioIn2);
    
    // Indicate the motor 0 is attached.
    this->mMotorAttached[0] = true;

    // Attach motor 1 input pins.
    if (!(gpioIn3 == 0 && gpioIn4 ==0)) {
        debug("init MCPWM Motor 1");
        // Set MCPWM unit 1
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, gpioIn3);
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, gpioIn4);

        // Indicate the motor 1 is attached.
        this->mMotorAttached[1] = true;
    }

    // Initial MCPWM configuration

    debug ("Configuring Initial Parameters of MCPWM...");

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency,
    pwm_config.cmpr_a = 0;              //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;              //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

    debug ("MCPWM initialized");
}
  
// Motor set speed forward

void ESP32MotorControl::motorForward(uint8_t motor, uint8_t speed)
{
    if (!isMotorValid(motor)) {
        return;
    }

    if (speed == 100) { // Full speed
        motorFullForward(motor);
    } else {
        // Set speed -> PWM duty 0-100
        if (motor == 0) {
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

        } else {
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
        }
        mMotorSpeed[motor] = speed; // Save it
        mMotorForward[motor] = true;
        debug("Motor %u forward speed %u", motor, speed);
    }
}
