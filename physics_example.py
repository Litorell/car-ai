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
        { # No slip
            "contact_point_x.acc": 1,
        },
        {
            "contact_point_y.acc": 1,
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
        { # No slip
            "contact_point_x.acc": 1,
        },
        {
            "contact_point_y.acc": 1,
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
        { # No slip
            "contact_point_x.acc": 1,
        },
        {
            "contact_point_y.acc": 1,
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
        { # No slip
            "contact_point_x.acc": 1,
        },
        {
            "contact_point_y.acc": 1,
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
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_rl_pos[0],
        },
        {
            "wheel_rr_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_rr_pos[0],
        },
        {
            "wheel_fl_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_fl_pos[0],
        },
        {
            "wheel_fr_x.acc": 1,
            "cg_x.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_fr_pos[0],
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
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_rl_pos[1],
        },
        {
            "wheel_rr_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_rr_pos[1],
        },
        {
            "wheel_fl_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_fl_pos[1],
        },
        {
            "wheel_fr_y.acc": 1,
            "cg_y.acc": -1,
            "RHS": lambda state: state["yaw_rate"] ** 2 * wheel_fr_pos[1],
        },
        # Yaw
        {
            "wheel_fl_x.force": wheel_fl_pos[1],
            "wheel_fr_x.force": wheel_fr_pos[1],
            "wheel_rl_x.force": wheel_rl_pos[1],
            "wheel_rr_x.force": wheel_rr_pos[1],
            "wheel_fl_y.force": wheel_fl_pos[0],
            "wheel_fr_y.force": wheel_fr_pos[0],
            "wheel_rl_y.force": wheel_rl_pos[0],
            "wheel_rr_y.force": wheel_rr_pos[0],
            "yaw.acc": -moment_of_inertia,
        }
    ]
)

# Same frictional loading
extra_equations = [
    {
        "wheel_fl.contact_point_y.force": 1,
        "wheel_fr.contact_point_y.force": -1,
    },
    {
        "wheel_rl.contact_point_y.force": 1,
        "wheel_rr.contact_point_y.force": -1,
    }
]



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

system.add_equation(extra_equations[0])
system.add_equation(extra_equations[1])


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
    "yaw_rate": 0,
}

# t0 = timer()
# t1 = timer()
# print("Time:", 1e3*(t1 - t0), "ms")

sym_A, sym_b, variables = system.create_linear_system()
A, b = system.numeric_linear_system(sym_A, sym_b, state)


import scipy
v = np.array(variables).reshape(-1,1)
ns = scipy.linalg.null_space(A)
print("\n".join(["\t".join(l) for l in np.hstack([v,np.round(ns*100).astype(int)])]))

# x = solve(A, b)
x = lstsq(A, b)[0]

print(A @ x - b)


all_variables = dict(zip(variables, x))
for name in all_variables:
    print(name, round(all_variables[name], 2))


















