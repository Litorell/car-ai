import numpy as np
import matplotlib.pyplot as plt
from car import Car


def main():
    car = Car()
    car.throttle = 1
    car.brake = 0

    dt = 0.1
    t = np.arange(0, 100, dt)
    pos = np.zeros((len(t), 2))
    yaw = np.zeros(len(t))
    vel = np.zeros((len(t), 2))
    gear = np.zeros(len(t))
    rpm = np.zeros(len(t))
    motor_force = np.zeros(len(t))

    max_rpm = car.max_rpm


    for i in range(len(t)):

        if i == len(t)//2:
            car.throttle = 0
            car.brake = 1

        pos[i] = car.pos
        yaw[i] = car.yaw
        vel[i] = car.vel
        gear[i] = car.gear
        rpm[i] = car.motor_rpm
        motor_force[i] = car.motor_force
        car.update(dt)
        if car.motor_rpm > 0.9 * max_rpm:
            car.gear += 1
    
    speed = np.linalg.norm(vel, axis=1) * 3.6

    power = motor_force * speed / 3.6 / 1000 * 1.34 # hp

    # Plot graphs on top of each other
    plt.figure()

    plt.subplot(411)
    plt.plot(t, speed)
    plt.ylabel('Speed (km/h)')

    plt.subplot(412)
    plt.plot(t, gear+1)
    plt.ylabel('Gear')

    plt.subplot(413)
    plt.plot(t, rpm)
    plt.ylabel('RPM')

    plt.subplot(414)
    plt.plot(t, power)
    plt.ylabel('Power (hp)')

    plt.xlabel('Time (s)')
    plt.show()

    
        
            


    





main()




