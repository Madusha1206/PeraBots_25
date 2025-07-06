from controller import Robot

def run_robot(robot):
    timestep = 64
    max_speed = 6.28
    min_speed = 0.2 * max_speed  

    # Devices
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Distance sensors
    sensors = {}
    sensor_names = ['ds_left', 'ds_right', 'ds_f1', 'ds_f2', 'ds_f3', 'ds_f4', 'ds_f5', 'ds_f6']
    for name in sensor_names:
        sensors[name] = robot.getDevice(name)
        sensors[name].enable(timestep)

    # Speed patterns (adjusted for stability)
    def left_1():
        return 0.4 * max_speed, max_speed

    def left_2():
        return 0.2 * max_speed, max_speed

    def right_1():
        return max_speed, 0.4 * max_speed

    def right_2():
        return max_speed, 0.2 * max_speed

    def slowdown():
        return 0.5 * max_speed, 0.5 * max_speed

    # PID (with derivative term)
    Kp = 0.004
    Kd = 0.001
    last_error = 0

    while robot.step(timestep) != -1:
        # Read sensor values
        values = {name: sensors[name].getValue() for name in sensor_names}
        left_val = values['ds_left']
        right_val = values['ds_right']
        f1_val = values['ds_f1']
        f2_val = values['ds_f2']
        f3_val = values['ds_f3']
        f4_val = values['ds_f4']
        f5_val = values['ds_f5']
        f6_val = values['ds_f6']

        # Default forward
        left_speed = right_speed = max_speed

        # PID forward control when no front obstacle
        if (left_val == right_val) and (f1_val == 0) and (f4_val == 0):
            error = right_val - left_val
            correction = Kp * error + Kd * (error - last_error)
            last_error = error
            left_speed = max_speed - correction
            right_speed = max_speed + correction
            print("⬜ Clear path → PID forward")

        # Approaching obstacle directly ahead
        elif (f2_val != 0) and (f5_val != 0) and (f1_val == 0) and (f4_val == 0):
            left_speed, right_speed = slowdown()
            print("⚠️ Narrow passage → slowing down")

        # PID correction when slightly drifting
        elif left_val != right_val and (f1_val == 0) and (f4_val == 0):
            error = right_val - left_val
            correction = Kp * error + Kd * (error - last_error)
            last_error = error
            left_speed = max_speed - correction
            right_speed = max_speed + correction
            print(f"↔ PID correction: error={error:.2f}, correction={correction:.2f}")

        # Obstacle ahead
        elif (f1_val != 0) or (f4_val != 0):
            print("🔴 Obstacle ahead")

            side_diff = f2_val - f5_val
            if abs(side_diff) < 50:
                left_speed, right_speed = slowdown()
                print("⏸️ Equal sides → Slow down")
            elif side_diff > 0:
                left_speed, right_speed = left_1()
                print("↪ Turn left")
                if f3_val > f6_val:
                    left_speed, right_speed = left_2()
                    print("↪ Turn left more")
            else:
                left_speed, right_speed = right_1()
                print("↩ Turn right")
                if f6_val > f3_val:
                    left_speed, right_speed = right_2()
                    print("↩ Turn right more")

        # Enforce min speed to avoid jitter
        left_speed = max(left_speed, min_speed)
        right_speed = max(right_speed, min_speed)

        # Clamp to max speed
        left_motor.setVelocity(min(left_speed, max_speed))
        right_motor.setVelocity(min(right_speed, max_speed))

if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
