

import numpy as np
from scipy.linalg import solve
from scipy.sparse.linalg import spsolve
from time import perf_counter as timer




class PhysicsComponent:

    def __init__(self, name: str, equations: dict) -> None:
        self.name = name
        self.equations = equations

    
    def generate_equations(self, variable_names):
        A = []
        b = []
        for eqn in self.equations:
            row = [0] * len(variable_names)
            for name in eqn:
                if name == "RHS":
                    b.append(eqn["RHS"])
                    continue
                namespaced_name = self.name + "." + name
                row[variable_names.index(namespaced_name)] = eqn[name]
            A.append(row)
        return A, b

    
class PhysicsConnection:

    def __init__(self, component1, component2, name1, name2, flip_sign=False) -> None:
        self.component1 = component1
        self.component2 = component2
        self.name1 = name1
        self.name2 = name2
        self.flip_sign = flip_sign


    def generate_equations(self, variable_names):
        row = [0] * len(variable_names)
        namespaced_name1 = self.component1.name + "." + self.name1
        namespaced_name2 = self.component2.name + "." + self.name2
        row[variable_names.index(namespaced_name1)] = 1
        row[variable_names.index(namespaced_name2)] = -1
        if self.flip_sign:
            row[variable_names.index(namespaced_name2)] = 1
        return row, 0



class PhysicsSystem:
    
        def __init__(self) -> None:
            self.components = []
            self.connections = []

        def add_component(self, component: PhysicsComponent):
            self.components.append(component)

        def add_connection(self, connection: PhysicsConnection):
            self.connections.append(connection)

        
        def create_solid_connection(
                self,
                component1: PhysicsComponent, 
                component2: PhysicsComponent, 
                name1: str, 
                name2: str
            ):
            
            acc_connection = PhysicsConnection(
                component1,
                component2,
                f"{name1}.acc",
                f"{name2}.acc",
            )

            torque_connection = PhysicsConnection(
                component1,
                component2,
                f"{name1}.force",
                f"{name2}.force",
                flip_sign=True
            )

            self.add_connection(acc_connection)
            self.add_connection(torque_connection)


        
        def create_linear_system(self): 
            variable_names = []

            for component in self.components:
                for eqn in component.equations:
                    for name in eqn:
                        if name == "RHS":
                            continue
                        namespaced_name = component.name + "." + name
                        if namespaced_name not in variable_names:
                            variable_names.append(namespaced_name)

            b = []
            A = []

            for component in self.components:
                component_A, component_b = component.generate_equations(variable_names)
                A += component_A
                b += component_b

            for connection in self.connections:
                row, rhs = connection.generate_equations(variable_names)
                A.append(row)
                b.append(rhs)
            
            
            return A, b, variable_names
        

        def numeric_linear_system(self, A, b, state):

            numeric_A = []
            for eqn in A:
                num_eqn = []
                for constant in eqn:
                    if callable(constant):
                        num_eqn.append(constant(state))
                    else:
                        num_eqn.append(constant)
                numeric_A.append(num_eqn)

            numeric_b = []
            for constant in b:
                if callable(constant):
                    numeric_b.append(constant(state))
                else:
                    numeric_b.append(constant)

            return np.array(numeric_A), np.array(numeric_b)
            
        
        
        # def solve_variables(self):
        #     A, b, variables = self.create_linear_system()
        #     x = np.linalg.solve(A, b)

        #     # Validate the solution
                        


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
            "RHS": 0,
        }
    ]
)

gearbox = PhysicsComponent(
    "gearbox",
    [
        {
            "shaft_out.force": 1,
            "shaft_in.force": lambda state: state["gear_ratio"],
            "RHS": 0,
        },
        {
            "shaft_in.acc": 1,
            "shaft_out.acc": lambda state: -state["gear_ratio"],
            "RHS": 0,
        }
    ]
)

differential = PhysicsComponent(
    "differential",
    [
        { # Equal torque to both wheels
            "shaft_right.force": 1,
            "shaft_left.force": -1,
            "RHS": 0,
        },
        { # Torque conservation
            "shaft_in.force": 1,
            "shaft_left.force": 1,
            "shaft_right.force": 1,
            "RHS": 0,
        },
        {
            "shaft_in.acc": 2,
            "shaft_left.acc": -1,
            "shaft_right.acc": -1,
            "RHS": 0,
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
        {
            # "brakes.force": lambda state: np.cos(state["steering_angle"]),
            "brakes.force": 1,
            "contact_point.force": wheel_radius,
            "shaft_in.acc": -wheel_inertia,
            "RHS": 0,
        },
        {
            "contact_point.acc": 1,
            "shaft_in.acc": -wheel_radius,
            "RHS": 0,
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
            "contact_point.force": wheel_radius,
            "shaft_in.acc": -wheel_inertia,
            "RHS": 0,
        },
        {
            "contact_point.acc": 1,
            "shaft_in.acc": -wheel_radius,
            "RHS": 0,
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
            "contact_point.force": wheel_radius,
            "shaft_in.acc": -wheel_inertia,
            "RHS": 0,
        },
        {
            "contact_point.acc": 1,
            "shaft_in.acc": -wheel_radius,
            "RHS": 0,
        }
    ]
)

wheel_rr = PhysicsComponent(
    "wheel_rr",
    [
        {
            "brakes.force": 1,
            "RHS": lambda state: -state["rr_braking"],
        },
        {
            "shaft_in.force": 1,
            "brakes.force": 1,
            "contact_point.force": wheel_radius,
            "shaft_in.acc": -wheel_inertia,
            "RHS": 0,
        },
        {
            "contact_point.acc": 1,
            "shaft_in.acc": -wheel_radius,
            "RHS": 0,
        }
    ]
)


car = PhysicsComponent(
    "car",
    [
        { # X direction (longitudinal)
            "contact_point_fl.force": 1,
            "contact_point_fr.force": 1,
            "contact_point_rl.force": 1,
            "contact_point_rr.force": 1,
            "body.acc": -mass,
            "RHS": 0,
        },
        {
            "contact_point_rl.acc": 1,
            "body.acc": -1,
            "RHS": 0, # TODO: calculate based on wheel position (sin, cos, state etc).
        },
        {
            "contact_point_rr.acc": 1,
            "body.acc": -1,
            "RHS": 0,
        },
        {
            "contact_point_fl.acc": 1,
            "body.acc": -1,
            "RHS": 0,
        },
        {
            "contact_point_fr.acc": 1,
            "body.acc": -1,
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
system.add_component(car)


system.create_solid_connection(motor, gearbox, "shaft_out", "shaft_in")
system.create_solid_connection(gearbox, differential, "shaft_out", "shaft_in")
system.create_solid_connection(differential, wheel_rl, "shaft_left", "shaft_in")
system.create_solid_connection(differential, wheel_rr, "shaft_right", "shaft_in")
system.create_solid_connection(wheel_fl, car, "contact_point", "contact_point_fl")
system.create_solid_connection(wheel_fr, car, "contact_point", "contact_point_fr")
system.create_solid_connection(wheel_rl, car, "contact_point", "contact_point_rl")
system.create_solid_connection(wheel_rr, car, "contact_point", "contact_point_rr")
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


all_variables = dict(zip(variables, x))
for name in all_variables:
    print(name, all_variables[name])


















