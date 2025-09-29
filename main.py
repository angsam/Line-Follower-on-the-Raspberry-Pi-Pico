import time
import line_follower as lf

#Motors
motor = lf.motors_init(ena_pin=21, in1_pin=20, in2_pin=19, enb_pin=16, in3_pin=18, in4_pin=17, drive_dir=-1)
line_sensors = lf.sensors_init(left=26, center=27, right=28, alpha=0.7, flip_lr=False)
ultrasonic_sensor = lf.ultrasound_init(trig_pin=14, echo_pin=15)  # lub u=None jeśli bez czujnika

#Parameters
p = lf.params_default()
state = {'last_seen_ms': time.ticks_ms(), 'last_dist_ts': 0, 'last_dist': None}

print('Calibration (6 s)...')
lf.sensors_calibrate(line_sensors, ms=6000, debug=False)

pid = lf.pid_init(Kp=p['Kp'], Ki=p['Ki'], Kd=p['Kd'], I_max=p['I_MAX'])

while True:
    now = time.ticks_ms()

    # 1) Sensor reading and line error
    L, C, R = lf.sensors_read_range01(line_sensors)
    err = lf.line_error(L, C, R, error_sign=p['ERROR_SIGN'])
    if lf.sees_line(L, C, R, p['DETECT_THR']):
        state['last_seen_ms'] = now

    # 2) PID → steering correction
    corr_lin, deriv = lf.pid_step(pid, err)
    corr = lf.turn_correction(corr_lin, p['TURN_GAMMA'], p['TURN_GAIN'], p['TURN_MAX'])

    # 3) Speed ​​profile (curve/straight line)
    base_now = lf.base_speed_for(err, p['BASE_SPEED'], p['TURN_SLOWDOWN'], p['TURN_BETA'])

    # 4) Anti-collision + turbo (separate step)
    speed_factor = lf.measure_distance_periodic(state, ultrasonic_sensor, p, now)
    base_now = int(base_now * speed_factor)
    base_now = lf.straight_turbo(base_now, err, p['STRAIGHT_EPS'], p['STRAIGHT_BOOST'], speed_factor)

    # 5) Lost line (arcs/spin) 
    handled, state['last_seen_ms'] = lf.lost_line_act(motor, line_sensors, pid['prev'], state['last_seen_ms'], now, p)
    if not handled:
        # 6) Normal driving: left = base - corr, right = base + corr
        # 6a) Short form:
        lf.drive_with_correction(motor, base_now, corr, p['REV_THRESHOLD'])
        # 6b) Long form (equivalent):
        '''
        left  = base_now * (1.0 - corr)
        right = base_now * (1.0 + corr)
        if abs(corr) <= p['REV_THRESHOLD']:
            if left < 0:
                left = 0
            if right < 0:
                right = 0
        lf.set_motor_lr(motor, left, right)
        '''
        
    time.sleep_ms(p['LOOP_DT_MS'])
