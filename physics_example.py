from physics import *




# Example usage

# Car
mass = 1500
wheel_base = 2
width = 1.5

# Motor
motor_inertia = 0.1

# Wheel
wheel_radius = 0.3
wheel_inertia = 0.1
wheel_fl_pos = np.array([wheel_base/2, width/2])
wheel_fr_pos = np.array([wheel_base/2, -width/2])
wheel_rl_pos = np.array([-wheel_base/2, width/2])
wheel_rr_pos = np.array([-wheel_base/2, -width/2])


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
            # "brakes.force": lambda state: np.cos(state["steering_angle"]),
            "brakes.force": 1,
            "contact_point.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        { # Shaft rotation acceleration
            "hub.acc": 1,
            "contact_point.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        { # Balance forces
            "hub.force": 1,
            "contact_point.force": 1,
        },
        # TODO: Create slip equation
        { # No slip
            "contact_point.acc": 1,
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
        {
            # "brakes.force": lambda state: np.cos(state["steering_angle"]),
            "brakes.force": 1,
            "contact_point.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        {
            "hub.acc": 1,
            "contact_point.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub.force": 1,
            "contact_point.force": 1,
        },
        {
            "contact_point.acc": 1,
        }
    ]
)

wheel_rl = PhysicsComponent(
    "wheel_rl",
    [
        {
            "brakes.force": 1,
            "RHS": lambda state: -state["rl_braking"],
        },
        {
            "shaft_in.force": 1,
            "brakes.force": 1,
            "contact_point.force": -wheel_radius,
            "shaft_in.acc": -wheel_inertia,
        },
        {
            "hub.acc": 1,
            "contact_point.acc": -1,
            "shaft_in.acc": -wheel_radius,
        },
        {
            "hub.force": 1,
            "contact_point.force": 1,
        },
        {
            "contact_point.acc": 1,
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
        { # Balance forces
            "hub_x.force": 1,
            "contact_point_x.force": 1,
        },
        # { # Balance forces
        #     "hub_y.force": 1,
        #     "contact_point_y.force": 1,
        # },
        { # No slip
            "contact_point_x.acc": 1,
        },
        # {
        #     "contact_point_y.acc": 1,
        # }
    ]
)

body = PhysicsComponent(
    "body",
    [
        { # X direction (longitudinal)
            "wheel_fl.force": 1,
            "wheel_fr.force": 1,
            "wheel_rl.force": 1,
            "wheel_rr.force": 1,
            "cg.acc": -mass,
            "RHS": 0,
        },
        {
            "wheel_rl.acc": 1,
            "cg.acc": -1,
            "RHS": 0, # TODO: calculate based on wheel position (sin, cos, state etc).
        },
        {
            "wheel_rr.acc": 1,
            "cg.acc": -1,
            "RHS": 0,
        },
        {
            "wheel_fl.acc": 1,
            "cg.acc": -1,
            "RHS": 0,
        },
        {
            "wheel_fr.acc": 1,
            "cg.acc": -1,
            "RHS": 0,
        },
        # # Y direction (lateral)
        # {
        #     # TODO: Add lateral inertia
        # },
        # # Yaw
        # {
        #     # TODO: Add yaw inertia
        # }
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
system.create_solid_connection(wheel_fl, body, "hub", "wheel_fl")
system.create_solid_connection(wheel_fr, body, "hub", "wheel_fr")
system.create_solid_connection(wheel_rl, body, "hub", "wheel_rl")
system.create_solid_connection(wheel_rr, body, "hub_x", "wheel_rr")


# system.create_solid_connection(wheel_fl, car, "contact_point_y", "contact_point_fl_y")
# system.create_solid_connection(wheel_fr, car, "contact_point_y", "contact_point_fr_y")
# system.create_solid_connection(wheel_rl, car, "contact_point_y", "contact_point_rl_y")
# system.create_solid_connection(wheel_rr, car, "contact_point_y", "contact_point_rr_y")


state = {
    "motor_torque": 400,
    "gear_ratio": 10,
    "fl_braking": 0,
    "fr_braking": 0,
    "rl_braking": 0,
    "rr_braking": 0,
}

# t0 = timer()
# t1 = timer()
# print("Time:", 1e3*(t1 - t0), "ms")

sym_A, sym_b, variables = system.create_linear_system()
A, b = system.numeric_linear_system(sym_A, sym_b, state)

x = solve(A, b)
# x = lstsq(A, b)[0]


all_variables = dict(zip(variables, x))
for name in all_variables:
    print(name, all_variables[name])


















