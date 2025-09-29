import machine, time

# ===== Parameters =====

def params_default():
    return {
        'BASE_SPEED': 48000, # Base motor PWM speed (0..65535). Higher = faster on straights.
        'Kp': 0.70, # Proportional gain for PID line control
        'Ki': 0.00, # Integral gain (usually 0 in this project)
        'Kd': 0.22, # Derivative gain for PID
        'I_MAX': 2.0, # Maximum integral term (anti-windup)
        'TURN_GAIN': 2.4, # Amplification factor for steering correction
        'TURN_MAX': 2.6, # Maximum allowed steering correction
        'TURN_GAMMA': 0.85, # Nonlinearity factor: <1 boosts large errors, >1 softens them
        'TURN_SLOWDOWN': 0.7, # How much to slow down in curves (0..1). 0.7 = -70% at max error.
        'TURN_BETA': 1.7, # Nonlinearity for slowdown. >1 makes slowdown stronger for big errors.
        'STRAIGHT_EPS': 0.10, # If |error| < EPS, consider it a straight line
        'STRAIGHT_BOOST': 1.05, # Speed multiplier on straights (e.g. 1.05 = +5%)
        'REV_THRESHOLD': 1.1, # Allow inner wheel reversal only above this correction threshold
        'ERROR_SIGN': 1.0, # Flip sign if sensors are mounted reversed
        'DETECT_THR': 0.18, # Threshold for detecting black line (0..1)
        'SEARCH_SUM_THR': 0.12, # Threshold: if all sensors below, line is lost
        'LOST_MS1': 300, # Lost line for <300 ms → gentle curve
        'LOST_MS2': 1200, # Lost line for 300–1200 ms → stronger curve
        'SPIN_PWM': 52000, # Motor PWM during spin search
        'SPIN_MS_EACH': 1500, # Max spin duration in one direction (ms)
        'DIST_STOP_CM': 12, # Distance threshold (cm): stop completely
        'DIST_SLOW_CM': 30, # Distance threshold (cm): start slowing down
        'LOOP_DT_MS': 6 # Main loop delay (ms). Smaller = faster control loop.
    }

# ===== Motors (L298N) =====

def _pwm(pin, freq=1000):
    p = machine.PWM(machine.Pin(pin))
    p.freq(freq)
    return p

def motors_init(ena_pin, in1_pin, in2_pin, enb_pin, in3_pin, in4_pin, drive_dir=-1, max_pwm=65535):
    return {
        'ENA': _pwm(ena_pin), 'IN1': machine.Pin(in1_pin, machine.Pin.OUT), 'IN2': machine.Pin(in2_pin, machine.Pin.OUT),
        'ENB': _pwm(enb_pin), 'IN3': machine.Pin(in3_pin, machine.Pin.OUT), 'IN4': machine.Pin(in4_pin, machine.Pin.OUT),
        'drive_dir': drive_dir, 'max_pwm': max_pwm
    }

def _clip(m, x):
    x = int(abs(x)); return m['max_pwm'] if x > m['max_pwm'] else x
    
def set_motor_lr(m, left, right):
    left  *= m['drive_dir']
    right *= m['drive_dir']
    if left >= 0:
        m['IN1'].value(1)
        m['IN2'].value(0)
    else:
        m['IN1'].value(0)
        m['IN2'].value(1)
    m['ENA'].duty_u16(_clip(m, left))
    if right >= 0:
        m['IN3'].value(1)
        m['IN4'].value(0)
    else:
        m['IN3'].value(0)
        m['IN4'].value(1)
    m['ENB'].duty_u16(_clip(m, right))

def drive_with_correction(m, base_pwm, correction, rev_threshold):
    left  = base_pwm * (1.0 - correction)
    right = base_pwm * (1.0 + correction)
    if abs(correction) <= rev_threshold:
        if left < 0:
            left = 0
        if right < 0:
            right = 0
    set_motor_lr(m, left, right)
    
# ===== Line sensors =====

def sensors_init(left=26, center=27, right=28, alpha=0.7, flip_lr=False):
    return {
        'ADL': machine.ADC(left), 'ADC': machine.ADC(center), 'ADR': machine.ADC(right),
        'alpha': alpha, 'flip_lr': flip_lr,
        'minL': 65535, 'minC': 65535, 'minR': 65535,
        'maxL': 0, 'maxC': 0, 'maxR': 0,
        '_prev': None
    }

def _lp(s):
    l = s['ADL'].read_u16()
    c = s['ADC'].read_u16()
    r = s['ADR'].read_u16()
    if s['_prev'] is None:
        s['_prev'] = (l, c, r)
    else:
        pl, pc, pr = s['_prev']
        a = s['alpha']
        s['_prev'] = (int(a*l + (1-a)*pl), int(a*c + (1-a)*pc), int(a*r + (1-a)*pr))
    return s['_prev']

def sensors_calibrate(s, ms=6000, debug=False):
    t_end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        l, c, r = _lp(s)
        if l < s['minL']: s['minL'] = l
        if c < s['minC']: s['minC'] = c
        if r < s['minR']: s['minR'] = r
        if l > s['maxL']: s['maxL'] = l
        if c > s['maxC']: s['maxC'] = c
        if r > s['maxR']: s['maxR'] = r
        time.sleep_ms(10)
    if debug:
        print('Cal L:', s['minL'], s['maxL'], 'C:', s['minC'], s['maxC'], 'R:', s['minR'], s['maxR'])

def _norm(x, lo, hi):
    rng = hi - lo
    if rng < 5:
        return 0.5
    v = (x - lo) / rng
    if v < 0:
        return 0.0
    if v > 1:
        return 1.0
    return v

def sensors_read_range01(s):
    l, c, r = _lp(s)
    L = 1.0 - _norm(l, s['minL'], s['maxL'])
    C = 1.0 - _norm(c, s['minC'], s['maxC'])
    R = 1.0 - _norm(r, s['minR'], s['maxR'])
    if s['flip_lr']:
        L, R = R, L
    return L, C, R

def sees_line(L, C, R, detect_thr=0.18):
    return (L > detect_thr) or (C > detect_thr) or (R > detect_thr)

# ===== PID =====

def pid_init(Kp=0.7, Ki=0.0, Kd=0.22, I_max=2.0):
    return {'Kp':Kp,'Ki':Ki,'Kd':Kd,'I_MAX':I_max,'i':0.0,'prev':0.0}

def pid_step(pid, error):
    pid['i'] += error
    if pid['i'] > pid['I_MAX']:
        pid['i'] = pid['I_MAX']
    if pid['i'] < -pid['I_MAX']:
        pid['i'] = -pid['I_MAX']
    d = error - pid['prev']
    pid['prev'] = error
    out = pid['Kp']*error + pid['Ki']*pid['i'] + pid['Kd']*d
    return out, d

# ===== Other steps =====

def line_error(L, C, R, error_sign):
    num = (-1.0 * L) + (+1.0 * R)
    den = (L + C + R) + 1e-4
    return (num / den) * error_sign

def turn_correction(corr_lin, gamma, gain, turn_max):
    if corr_lin >= 0:
        sgn = 1.0
    else:
        sgn = -1.0
    corr_nl = sgn * (abs(corr_lin) ** gamma)
    corr = corr_nl * gain
    if corr >  turn_max:
        corr =  turn_max
    if corr < -turn_max:
        corr = -turn_max
    return corr

def base_speed_for(error, base_speed, turn_slowdown, beta):
    e = abs(error)
    if e > 1.0:
        e = 1.0 
    return int(base_speed * (1.0 - turn_slowdown * (e ** beta)))

def straight_turbo(base_now, error, eps, boost, speed_factor):
    if (abs(error) < eps) and (speed_factor >= 0.999):
        return int(base_now * boost)
    return base_now

# ===== Lost Line: Arcs + Spin =====

def lost_line_act(m, s, pid_prev, last_seen_ms, now_ms, params):
    """
    Returns (handled, new_last_seen_ms). If handled=True, engines already set.
    """
    L, C, R = sensors_read_range01(s)
    sum_sig = L + C + R
    if sum_sig >= params['SEARCH_SUM_THR']:
        return False, last_seen_ms

    lost_dt = time.ticks_diff(now_ms, last_seen_ms)
    base = params['BASE_SPEED']

    if lost_dt < params['LOST_MS1']:
        if pid_prev >= 0:
            set_motor_lr(m, int(base*0.6), int(base*1.0))
        else:
            set_motor_lr(m, int(base*1.0), int(base*0.6))
        return True, last_seen_ms

    if lost_dt < params['LOST_MS2']:
        if pid_prev >= 0:
            set_motor_lr(m, int(base*0.3), int(base*1.0))
        else:
            set_motor_lr(m, int(base*1.0), int(base*0.3))
        return True, last_seen_ms

    # spin search (both sides)
    set_motor_lr(m, 0, 0)
    if pid_prev >= 0:
        dir_sign = +1
    else:
        dir_sign = -1
    if _spin_once(m, s, dir_sign, params) or _spin_once(m, s, -dir_sign, params):
        return True, time.ticks_ms()
    return True, time.ticks_ms()

def _spin_once(m, s, direction_sign, p):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < p['SPIN_MS_EACH']:
        if direction_sign >= 0:
            set_motor_lr(m, -p['SPIN_PWM'], +p['SPIN_PWM'])
        else:
            set_motor_lr(m, +p['SPIN_PWM'], -p['SPIN_PWM'])
        l, c, r = _lp(s)
        L = 1.0 - _norm(l, s['minL'], s['maxL'])
        C = 1.0 - _norm(c, s['minC'], s['maxC'])
        R = 1.0 - _norm(r, s['minR'], s['maxR'])
        if s['flip_lr']:
            L, R = R, L
        if sees_line(L, C, R, p['DETECT_THR']):
            return True
        time.sleep_ms(8)
    return False

# ===== Ultrasonic sensor =====

def ultrasound_init(trig_pin=14, echo_pin=15, dist_max_cm=200):
    return {'TRIG': machine.Pin(trig_pin, machine.Pin.OUT), 'ECHO': machine.Pin(echo_pin, machine.Pin.IN), 'max': dist_max_cm}

def ultrasound_measure_cm(u, timeout_us=30000):
    u['TRIG'].value(0)
    time.sleep_us(2)
    u['TRIG'].value(1)
    time.sleep_us(10)
    u['TRIG'].value(0)
    try:
        dur = machine.time_pulse_us(u['ECHO'], 1, timeout_us)
    except OSError:
        return None
    if dur <= 0:
        return None
    dist = (dur * 0.0343) / 2.0
    if dist <= 0 or dist > u['max']:
        return None
    return dist

def obstacle_speed_factor(dist_cm, stop_cm, slow_cm):
    if dist_cm is None:
        return 1.0
    if dist_cm <= stop_cm:
        return 0.0
    if dist_cm < slow_cm:
        return (dist_cm - stop_cm) / (slow_cm - stop_cm)
    return 1.0

def measure_distance_periodic(state, u, params, now):
    if u is None:
        return 1.0
    if time.ticks_diff(now, state['last_dist_ts']) >= 60:
        state['last_dist'] = ultrasound_measure_cm(u)
        state['last_dist_ts'] = now
    return obstacle_speed_factor(state['last_dist'], params['DIST_STOP_CM'], params['DIST_SLOW_CM'])
