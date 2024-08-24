from controller import Robot
import time
import matplotlib.pyplot as plt

waktuMulai = time.time()

#data plot
motorKanan_plot = []
motorKiri_plot = []
pid_plot_kanan = []
pid_plot_kiri = []
left_wall_plot = []
left_corner_plot = []
front_wall_plot = []
pid_real = []

# Fungsi PID sederhana
def calculate_motor(signal):
    return (signal/100)*6.28

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 100
    Motor_NoPID = 80

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    prox_sensor = []

    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensor.append(robot.getDevice(sensor_name))
        prox_sensor[ind].enable(timestep)

    # Inisialisasi PID Controller
    Kp = 2
    Ki = 0.00005
    Kd = 1.5
    target = 100
    prev_error = 0
    integral = 0
    pid = 0

    while robot.step(timestep) != -1:
        sensor_values = [prox_sensor[ind].getValue() for ind in range(8)]
        print("Sensor Values:", sensor_values)

        # Gunakan nilai sensor yang sesuai untuk kebutuhan kontrol
        left_wall = sensor_values[5] > 80
        left_corner = sensor_values[6] > 80
        front_wall = sensor_values[7] > 80

        error = target - sensor_values[5]
        # Hitung integral
        integral += error
        # derivatif
        derivative = error - prev_error
        # Simpan nilai error
        prev_error = error

        # Kontrol PID
        pid_controller = Kp * error + Ki * integral + Kd * derivative
        if pid_controller >= (max_speed-Motor_NoPID-1):
            pid_controller = (max_speed-Motor_NoPID-1)
        if pid_controller <= -(max_speed-Motor_NoPID-1):
            pid_controller = -(max_speed-Motor_NoPID-1)
        motor = calculate_motor(Motor_NoPID)
        maks = calculate_motor(max_speed)
        pid = calculate_motor(pid_controller)

        # Logika gerakan robot
        if front_wall:
            left_speed = maks
            right_speed = -maks
        elif left_corner:
            left_speed = motor-pid
            right_speed = motor+pid
        else:
            left_speed = motor
            right_speed = motor

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        print("Kiri Values:", left_speed)
        print("Kanan Values:", right_speed)
        print("pid:", pid)
        prev_error = sensor_values[0]
        waktu = time.time()

        motorKanan_plot.append(right_motor.getVelocity())
        pid_plot_kanan.append(right_speed)
        motorKiri_plot.append(left_motor.getVelocity())
        pid_plot_kiri.append(left_speed)
        pid_real.append(pid_controller)

        left_wall_plot.append(left_wall)
        left_corner_plot.append(left_corner)
        front_wall_plot.append(front_wall)

        # simulasi = waktu - waktuMulai
        # if simulasi >= 5:
        #     break

if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)
    
    # plt.figure()

    # plt.subplot(2, 1, 1)
    # plt.plot (left_wall_plot, label = "Sensor 5", color = "red" )
    # plt.plot (left_corner_plot, label = "Sensor 6", color = "Blue" )
    # plt.plot (front_wall_plot, label = "Sensor 7", color = "Green" )
    # plt.ylabel("Pembacaan Sensor")
    # plt.xlabel("Timestep")
    # plt.legend ()

    # plt.subplot(2, 2, 3)
    # plt.plot (pid_plot_kanan, label = "Motor Kanan", color = "red" )
    # plt.plot (pid_plot_kiri, label = "Motor Kiri", color = "Blue" )
    # plt.ylabel("Kecepatan Motor")
    # plt.xlabel("Timestep")
    # plt.legend ()

    # plt.subplot(2, 2, 4)
    # plt.plot (pid_real, label = "PID", color = "red" )
    # plt.ylabel("PID")
    # plt.xlabel("Timestep")
    # plt.legend ()

    # plt.show()
