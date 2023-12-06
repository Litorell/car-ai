

import numpy as np
from scipy.linalg import solve, lstsq
from scipy.sparse.linalg import spsolve
from time import perf_counter as timer


def dict_eqn_to_list(eqn: dict, variable_names: list, component_name: str = None):
    row = [0] * len(variable_names)
    rhs = 0
    for name in eqn:
        if name == "RHS":
            rhs = eqn["RHS"]
            continue
        if component_name is not None:
            namespaced_name = component_name + "." + name
        else:
            namespaced_name = name
        row[variable_names.index(namespaced_name)] = eqn[name]
    return row, rhs


class PhysicsComponent:

    def __init__(self, name: str, equations: dict) -> None:
        self.name = name
        self.equations = equations

    
    def generate_equations(self, variable_names):
        A = []
        b = []
        for eqn in self.equations:
            row, rhs = dict_eqn_to_list(eqn, variable_names, self.name)
            A.append(row)
            b.append(rhs)
        return A, b

    
class PhysicsConnection:

    def __init__(
            self, 
            component1: PhysicsComponent, 
            component2: PhysicsComponent, 
            name1: str, 
            name2: str, 
            flip_sign: bool = False,
            validation_function: callable = None,
            fallback_equation: dict = None,
            ) -> None:
        
        self.component1 = component1
        self.component2 = component2
        self.name1 = name1
        self.name2 = name2
        self.flip_sign = flip_sign
        self.equation = [
            {
                name1: 1,
                name2: -1 if not flip_sign else 1,
                "RHS": 0,
            }
        ]
        self.validation_function = validation_function
        self.fallback_equation = fallback_equation


    def generate_equation(self, variable_names):
        row = [0] * len(variable_names)
        namespaced_name1 = self.component1.name + "." + self.name1
        namespaced_name2 = self.component2.name + "." + self.name2
        row[variable_names.index(namespaced_name1)] = 1
        row[variable_names.index(namespaced_name2)] = -1
        if self.flip_sign:
            row[variable_names.index(namespaced_name2)] = 1
        return row, 0
        
    
    def validate(self, state):
        if self.validation_function is not None:
            return self.validation_function(state)
        return True


    def get_fallback_equation(self, state):
        if self.fallback_equation is not None:
            return self.fallback_equation(state)
        return None


class PhysicsSystem:
    

        def __init__(self) -> None:
            self.components = []
            self.connections = []
            self.extra_equations = []

        def add_component(self, component: PhysicsComponent):
            self.components.append(component)

        def add_connection(self, connection: PhysicsConnection):
            self.connections.append(connection)

        def add_equation(self, equation: dict):
            self.extra_equations.append(equation)
        
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


        def create_slip_connection(
                self,
                component1: PhysicsComponent,
                component2: PhysicsComponent,
                name1: str,
                name2: str,
                slip_condition: callable,
                slip_force: callable,
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

            slip_connection = PhysicsConnection(
                component1,
                component2,
                f"{name1}.force",
                f"{name2}.force",
                flip_sign=True
            )

        
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

            for eqn in self.extra_equations:
                for name in eqn:
                    if name == "RHS":
                        continue
                    if name not in variable_names:
                        variable_names.append(name)

            b = []
            A = []

            for component in self.components:
                component_A, component_b = component.generate_equations(variable_names)
                A += component_A
                b += component_b

            for connection in self.connections:
                row, rhs = connection.generate_equation(variable_names)
                A.append(row)
                b.append(rhs)
            
            for eqn in self.extra_equations:
                row, rhs = dict_eqn_to_list(eqn, variable_names)
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
                        

