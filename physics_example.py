from physics import *




# Example usage

# Car
mass = 1500
wheel_base = 2
width = 1.5
moment_of_inertia = 1000

# Motor
motor_inertia = 0.1

# Wheel
wheel_radius = 0.3
wheel_inertia = 0.1
wheel_positions = {
    "wheel_fl_x": wheel_base/2,
    "wheel_fl_y": width/2,
    "wheel_fr_x": wheel_base/2,
    "wheel_fr_y": -width/2,
    "wheel_rl_x": -wheel_base/2,
    "wheel_rl_y": width/2,
    "wheel_rr_x": -wheel_base/2,
    "wheel_rr_y": -width/2,
}

max_braking_torque = 1000

gear_count = 6
gear_ratios = np.array([gear_count/n for n in range(1, gear_count+1)])


def get_centrifugal_acc(state, wheel_name):
    return state["body.yaw.vel"] ** 2 * wheel_positions[wheel_name]

def calculate_slip_force(slip_vel):
    normal_force = 9.82 * mass / 4
    mu = 0.8
    slip_norm = np.linalg.norm(slip_vel)
    if slip_norm == 0:
        return np.array([0,0])
    slip_direction = slip_vel / slip_norm
    force_magnitude = (1 - np.exp(-slip_norm)) * mu * normal_force
    return -slip_direction * force_magnitude

def calculate_motor_rpm(input_variables, state_defining):
    rl_rpm = state_defining["wheel_rl.shaft_in.vel"] / (2*np.pi) * 60
    rr_rpm = state_defining["wheel_rr.shaft_in.vel"] / (2*np.pi) * 60
    diff_rpm = (rl_rpm + rr_rpm) / 2
    gear_ratio = gear_ratios[input_variables["gear"] - 1]
    return diff_rpm * gear_ratio


def motor_torque_curve(throttle, rpm):
    max_rpm = 8000
    max_torque = 500
    x = rpm / max_rpm
    return throttle * (1-x)*(x+0.5)**2 * 2 * max_torque





system = PhysicsSystem()


motor = PhysicsComponent(
    "motor",
    [
        {
            "input.force": 1,
            "RHS": lambda state: state["motor_torque"],
        },
        {
            "input.force": 1,
            "shaft_out.force": 1,
            "shaft_out.acc": -motor_inertia,
        }
    ]
)

gearbox = PhysicsComponent(
    "gearbox",
    [
        {
            "shaft_out.force": 1,
            "shaft_in.force": lambda state: state["gear_ratio"],
        },
        {
            "shaft_in.acc": 1,
            "shaft_out.acc": lambda state: -state["gear_ratio"],
        }
    ]
)

differential = PhysicsComponent(
    "differential",
    [
        { # Equal torque to both wheels
            "shaft_right.force": 1,
            "shaft_left.force": -1,
        },
        { # Torque conservation
            "shaft_in.force": 1,
            "shaft_left.force": 1,
            "shaft_right.force": 1,
        },
        {
            "shaft_in.acc": 2,
            "shaft_left.acc": -1,
            "shaft_right.acc": -1,
        }
    ]
)

wheel_fl = PhysicsComponent(
    "wheel_fl",
    [
        {
            "brakes.force": 1,
            "RHS": lambda state: -state["fl_braking"],
        },
        { # Balance torques
            "brakes.force": 1,
            "contact_point_x.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        { # Shaft rotation acceleration
            "hub_x.acc": 1,
            "contact_point_x.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub_y.acc": 1,
            "contact_point_y.acc": -1,
        },
        { # Balance forces
            "hub_x.force": 1,
            "contact_point_x.force": 1,
        },
        {
            "hub_y.force": 1,
            "contact_point_y.force": 1,
        },
        {
            "contact_point_x.force": 1,
            "RHS": lambda state: state["wheel_fl_x.force"]
        },
        {
            "contact_point_y.force": 1,
            "RHS": lambda state: state["wheel_fl_y.force"]
        }
    ]
)

wheel_fr = PhysicsComponent(
    "wheel_fr",
    [
        {
            "brakes.force": 1,
            "RHS": lambda state: -state["fr_braking"],
        },
        { # Balance torques
            "brakes.force": 1,
            "contact_point_x.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        { # Shaft rotation acceleration
            "hub_x.acc": 1,
            "contact_point_x.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub_y.acc": 1,
            "contact_point_y.acc": -1,
        },
        { # Balance forces
            "hub_x.force": 1,
            "contact_point_x.force": 1,
        },
        {
            "hub_y.force": 1,
            "contact_point_y.force": 1,
        },
        {
            "contact_point_x.force": 1,
            "RHS": lambda state: state["wheel_fr_x.force"]
        },
        {
            "contact_point_y.force": 1,
            "RHS": lambda state: state["wheel_fr_y.force"]
        }
    ]
)

wheel_rl = PhysicsComponent(
    "wheel_rl",
    [
        { # Brake force
            "brakes.force": 1,
            "RHS": lambda state: -state["rl_braking"],
        },
        { # Balance torques
            "shaft_in.force": 1,
            "brakes.force": 1,
            "contact_point_x.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        { # Shaft rotation acceleration
            "hub_x.acc": 1,
            "contact_point_x.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub_y.acc": 1,
            "contact_point_y.acc": -1,
        },
        { # Balance forces
            "hub_x.force": 1,
            "contact_point_x.force": 1,
        },
        {
            "hub_y.force": 1,
            "contact_point_y.force": 1,
        },
        {
            "contact_point_x.force": 1,
            "RHS": lambda state: state["wheel_rl_x.force"]
        },
        {
            "contact_point_y.force": 1,
            "RHS": lambda state: state["wheel_rl_y.force"]
        }
    ]
)

wheel_rr = PhysicsComponent(
    "wheel_rr",
    [
        { # Brake force
            "brakes.force": 1,
            "RHS": lambda state: -state["rr_braking"],
        },
        { # Balance torques
            "shaft_in.force": 1,
            "brakes.force": 1,
            "contact_point_x.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        { # Shaft rotation acceleration
            "hub_x.acc": 1,
            "contact_point_x.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub_y.acc": 1,
            "contact_point_y.acc": -1,
        },
        { # Balance forces
            "hub_x.force": 1,
            "contact_point_x.force": 1,
        },
        {
            "hub_y.force": 1,
            "contact_point_y.force": 1,
        },
        {
            "contact_point_x.force": 1,
            "RHS": lambda state: state["wheel_rr_x.force"]
        },
        {
            "contact_point_y.force": 1,
            "RHS": lambda state: state["wheel_rr_y.force"]
        }
    ]
)

body = PhysicsComponent(
    "body",
    [
        { # X direction (longitudinal)
            "wheel_fl_x.force": 1,
            "wheel_fr_x.force": 1,
            "wheel_rl_x.force": 1,
            "wheel_rr_x.force": 1,
            "cg_x.acc": -mass,
            "RHS": 0,
        },
        {
            "wheel_rl_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_rl_x"),
        },
        {
            "wheel_rr_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_rr_x"),
        },
        {
            "wheel_fl_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_fl_x"),
        },
        {
            "wheel_fr_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_fr_x"),
        },
        # Y direction (lateral)
        {
            "wheel_fl_y.force": 1,
            "wheel_fr_y.force": 1,
            "wheel_rl_y.force": 1,
            "wheel_rr_y.force": 1,
            "cg_y.acc": -mass,
            "RHS": 0,
        },
        {
            "wheel_rl_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_rl_y"),
        },
        {
            "wheel_rr_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_rr_y"),
        },
        {
            "wheel_fl_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_fl_y"),
        },
        {
            "wheel_fr_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: get_centrifugal_acc(state, "wheel_fr_y"),
        },
        # Yaw
        {
            "wheel_fl_x.force": -wheel_positions["wheel_fl_y"],
            "wheel_fr_x.force": -wheel_positions["wheel_fr_y"],
            "wheel_rl_x.force": -wheel_positions["wheel_rl_y"],
            "wheel_rr_x.force": -wheel_positions["wheel_rr_y"],
            "wheel_fl_y.force":  wheel_positions["wheel_fl_x"],
            "wheel_fr_y.force":  wheel_positions["wheel_fr_x"],
            "wheel_rl_y.force":  wheel_positions["wheel_rl_x"],
            "wheel_rr_y.force":  wheel_positions["wheel_rr_x"],
            "yaw.acc": -moment_of_inertia,
        }
    ]
)



system.add_component(motor)
system.add_component(gearbox)
system.add_component(differential)
system.add_component(wheel_fl)
system.add_component(wheel_fr)
system.add_component(wheel_rl)
system.add_component(wheel_rr)
system.add_component(body)


system.create_solid_connection(motor, gearbox, "shaft_out", "shaft_in")
system.create_solid_connection(gearbox, differential, "shaft_out", "shaft_in")
system.create_solid_connection(differential, wheel_rl, "shaft_left", "shaft_in")
system.create_solid_connection(differential, wheel_rr, "shaft_right", "shaft_in")
system.create_solid_connection(wheel_fl, body, "hub_x", "wheel_fl_x")
system.create_solid_connection(wheel_fr, body, "hub_x", "wheel_fr_x")
system.create_solid_connection(wheel_rl, body, "hub_x", "wheel_rl_x")
system.create_solid_connection(wheel_rr, body, "hub_x", "wheel_rr_x")
system.create_solid_connection(wheel_fl, body, "hub_y", "wheel_fl_y")
system.create_solid_connection(wheel_fr, body, "hub_y", "wheel_fr_y")
system.create_solid_connection(wheel_rl, body, "hub_y", "wheel_rl_y")
system.create_solid_connection(wheel_rr, body, "hub_y", "wheel_rr_y")


# Defining variables of the system. These are the variables that are solved for
# and updated every timestep.
state_defining = {
    "body.cg_x.pos": 0,
    "body.cg_x.vel": 0,
    "body.cg_y.pos": 0,
    "body.cg_y.vel": 0,
    "body.yaw.pos": 0,
    "body.yaw.vel": 0,
    "wheel_fl.shaft_in.vel": 0,
    "wheel_fr.shaft_in.vel": 0,
    "wheel_rl.shaft_in.vel": 0,
    "wheel_rr.shaft_in.vel": 0,
}

state_computed = {
    # Friction from slip
    "wheel_fl_x.force": 0,
    "wheel_fl_y.force": 0,
    "wheel_fr_x.force": 0,
    "wheel_fr_y.force": 0,
    "wheel_rl_x.force": 0,
    "wheel_rl_y.force": 0,
    "wheel_rr_x.force": 0,
    "wheel_rr_y.force": 0,
    # Motor torque based on throttle curve
    "motor_torque": 400,
    # Braking force based on brake pedal
    "fl_braking": 0,
    "fr_braking": 0,
    "rl_braking": 0,
    "rr_braking": 0,
    # Gear ratio based on gear
    "gear_ratio": 0,
}

sym_A, sym_b, variables = system.create_linear_system()
def rotate90(vec):
    return np.array([-vec[1], vec[0]])


for t in range(10000):

    # Variables that control the system
    input_variables = {
        "steering_angle": 0,
        "throttle": 0.4,
        "brake": 0,
        "gear": 1,
    }

    state_computed["gear_ratio"] = gear_ratios[input_variables["gear"] - 1]

    rpm = calculate_motor_rpm(input_variables, state_defining)
    state_computed["motor_torque"] = motor_torque_curve(input_variables["throttle"], rpm)



    for quadrant in ["fl", "fr", "rl", "rr"]:
        wheel_name = f"wheel_{quadrant}"
        body_v_x = state_defining["body.cg_x.vel"]
        body_v_y = state_defining["body.cg_y.vel"]
        body_vel = np.array([body_v_x, body_v_y])
        wheel_local_pos_x = wheel_positions[f"wheel_{quadrant}_x"]
        wheel_local_pos_y = wheel_positions[f"wheel_{quadrant}_y"]
        wheel_local_pos = np.array([wheel_local_pos_x, wheel_local_pos_y])
        yaw_rate = state_defining["body.yaw.vel"]
        wheel_vel = body_vel + yaw_rate * rotate90(wheel_local_pos)
        rot_speed = state_defining[f"wheel_{quadrant}.shaft_in.vel"]

        steering_angle = 0
        if quadrant in ["fl", "fr"]:
            steering_angle = input_variables["steering_angle"]
        
        rot_mat = np.array([
            [np.cos(steering_angle), np.sin(steering_angle)],
            [-np.sin(steering_angle), np.cos(steering_angle)]
        ])
        slip_vel = wheel_vel - rot_mat @ np.array([1,0]) * rot_speed * wheel_radius

        force = calculate_slip_force(slip_vel)
        state_computed[f"{wheel_name}_x.force"] = force[0]
        state_computed[f"{wheel_name}_y.force"] = force[1]

        braking_torque = -input_variables["brake"] * max_braking_torque * np.sign(rot_speed)
        state_computed[f"{quadrant}_braking"] = braking_torque

    state = {**input_variables, **state_defining, **state_computed}


    A, b = system.numeric_linear_system(sym_A, sym_b, state)

    # t0 = timer()
    # t1 = timer()
    # print("Time:", round(1e3*(t1 - t0),2), "ms")


    x = solve(A, b)
    # x = lstsq(A, b)[0]


    # all_variables = dict(zip(variables, x))
    # for name in all_variables:
    #     print(name, round(all_variables[name], 2))

    # print("Changes:")
    
    # Update state
    dt = 0.001
    
    defining_vars = [n[:-4] for n in state_defining if n.endswith(".vel")]
    for name in defining_vars:
        # TODO: This uses Euler forward, leads to instability for the lighter
        # front wheels. Possible solutions:
        # - Use a more stable integration method (e.g. Runge-Kutta, Leapfrog)
        # - Linearize everything and use a linear solver
        # - Do an ad hoc fix by checking if the slip for the front wheels 
        #   changes direction. (Cheap, but might work well enough)



        state_defining[name + ".vel"] += x[variables.index(name + ".acc")]*dt
        print(name + ".vel", round(state_defining[name + ".vel"], 2))
    
    d_pos_local = np.array([
        state_defining["body.cg_x.vel"]*dt,
        state_defining["body.cg_y.vel"]*dt,
    ])

    yaw = state_defining["body.yaw.pos"]
    yaw_rot_mat = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])
    d_pos_global = yaw_rot_mat @ d_pos_local
    state_defining["body.cg_x.pos"] += d_pos_global[0]
    state_defining["body.cg_y.pos"] += d_pos_global[1]
    
    state_defining["body.yaw.pos"] += state_defining["body.yaw.vel"] * dt

    # print("")

for name in defining_vars:
    print(name + ".vel", round(state_defining[name + ".vel"], 2))









