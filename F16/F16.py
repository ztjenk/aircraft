"""
F-16 Aircraft Simulation using MachUpX
"""

# Import required modules
import machupX as MX
import numpy as np
import json
import pandas as pd
import os

class MachUpXWrapper:
    """Wrapper class for handling MachUpX operations."""

    def __init__(self, input_file):
        # Initialize the Scene using the input file
        self.input_file = input_file
        self.my_scene = MX.Scene(scene_input=input_file)
        # Initialize the Scene using the input file
        self.input_file = input_file
        self.my_scene = MX.Scene(scene_input=input_file)
    
    def display_geometry(self):
        """Displays the aircraft geometry as a wireframe"""
        self.my_scene.display_wireframe(show_legend=False)

    def pitch_trim(self, aircraft_name):
        """Trims the aircraft in pitch using the elevator"""

        # Trim the aircraft in pitch and apply the trim state to the scene
        trim_state = self.my_scene.pitch_trim(set_trim_state=True, verbose=False)

        # Print the trimmed state in a JSON format
        print(f"Trim state for {aircraft_name}:")
        print(json.dumps(trim_state[aircraft_name], indent=4))

        # Saves and returns the trimmed alpha and elevator angles
        trimmed_alpha = trim_state[aircraft_name]["alpha"]
        trimmed_elevator = trim_state[aircraft_name]["elevator"]
        return trimmed_alpha, trimmed_elevator
    
    def derivatives(self, aircraft_name):
        """Calculates aerodynamic derivatives"""

        derivs = self.my_scene.derivatives()
        # Print derivatives in the terminal
        print(json.dumps(derivs[aircraft_name], indent=4))

        return derivs[aircraft_name]

    def solve_forces(self, velocity, alpha, beta, p, q, r, elevator, rudder, aileron, aircraft_name, dimensional=True, non_dimensional=True):
        """
        Sets the aircraft state and control, solves for forces, and returns them.
        
        Parameters:
        - velocity (float): Aircraft velocity.
        - alpha (float): Angle of attack (degrees).
        - beta (float): Sideslip angle (degrees).
        - p, q, r (float): Angular rates (roll, pitch, yaw in deg/s).
        - elevator, rudder, aileron (float): Control surface deflections (degrees).
        - aircraft_name (str): Name of the aircraft in the scene (e.g., 'F16').

        Returns:
        - Force_array (numpy array): Array containing the forces and moments.
        """
        # Set aircraft flight state
        state = {
            "velocity": velocity,
            "alpha": alpha,
            "beta": beta,
            "angular_rates": [np.radians(p), np.radians(q), np.radians(r)],
        }
        # Set control surface deflections
        control_state = {
            "elevator": elevator,
            "rudder": rudder,
            "aileron": aileron,
        }
        
        # Apply the aircraft state and control state to the scene
        self.my_scene.set_aircraft_state(state=state)
        self.my_scene.set_aircraft_control_state(control_state=control_state)

        # Solve for forces and moments in the body frame
        forces = self.my_scene.solve_forces(dimensional=dimensional, non_dimensional=non_dimensional, verbose=False)

        # Show forces in terminal.
        # print(json.dumps(forces[aircraft_name]["total"], indent=4))

        # Parse and return the relevant forces and moments
        return forces[aircraft_name]["total"]

    def export_csv(self, filename, velocity, dimensional=True, non_dimensional=True, aircraft_name="F16"):
        "Generates a CSV file with simulation results"
        # Define ranges for the variables to iterate over
        # velocity = 222.5211                      # ft/s
        alphas = np.linspace(-25, 25, 11)         # deg
        betas = np.linspace(-4, 4, 5)            # deg
        ailerons = np.linspace(-21.5, 21.5, 3)       # deg
        # elevators = np.linspace(-20, 20, 3)      # deg
        # rudders = np.linspace(-30, 30, 3)        # deg
        # p_vals = np.concatenate([np.linspace(-.1058863402253694, -1e-12, 2), np.linspace(0, .1058863402253694, 2)    ])     # rad/s
        # q_vals = np.concatenate([np.linspace(-.013318148570568686, -1e-12, 2), np.linspace(0, .013318148570568686, 2)    ])     # rad/s
        # r_vals = np.concatenate([np.linspace(-.0352954467417898, -1e-12, 2), np.linspace(0, .0352954467417898, 2)    ])     # rad/s
        # beta = 0
        # aileron = 0
        elevator = 0
        rudder = 0
        p = 0
        q = 0
        r = 0

        # Create a list to store the data
        data = []
        # Iterate through all the combinations of values
        for aileron in ailerons:
            for beta in betas:
            #     for elevator in elevators:
            #         for r in r_vals:
            #             for rudder in rudders:
                            for alpha in alphas:
                                # for p in p_vals:
                                #     for q in q_vals:
                                        state = {
                                              "velocity": velocity,
                                              "alpha": alpha,
                                              "beta": beta,
                                              "angular_rates": [p, q, r],
                                        }
                                        self.my_scene.set_aircraft_state(state=state)
                                        control_state = {
                                            "elevator": elevator,
                                            "rudder": rudder,
                                            "aileron": aileron,
                                        }
                                        self.my_scene.set_aircraft_control_state(control_state=control_state)
                                        forces = self.solve_forces(velocity, alpha, beta, p, q, r, elevator, rudder, aileron, aircraft_name,
                                                                   dimensional=dimensional, non_dimensional=non_dimensional)
                                        # derivs = self.derivatives(aircraft_name)
                                        # damping = derivs.get("damping", {})
                                        # Cx_pbar = damping.get("Cx,pbar", None)
                                        # Cy_pbar = damping.get("Cy,pbar", None)
                                        # Cz_pbar = damping.get("Cz,pbar", None)
                                        # Cl_pbar = damping.get("Cl,pbar", None)
                                        # Cm_pbar = damping.get("Cm,pbar", None)
                                        # Cn_pbar = damping.get("Cn,pbar", None)

                                        # Cx_qbar = damping.get("Cx,qbar", None)
                                        # Cy_qbar = damping.get("Cy,qbar", None)
                                        # Cz_qbar = damping.get("Cz,qbar", None)
                                        # Cl_qbar = damping.get("Cl,qbar", None)
                                        # Cm_qbar = damping.get("Cm,qbar", None)
                                        # Cn_qbar = damping.get("Cn,qbar", None)

                                        # Cx_rbar = damping.get("Cx,rbar", None)
                                        # Cy_rbar = damping.get("Cy,rbar", None)
                                        # Cz_rbar = damping.get("Cz,rbar", None)
                                        # Cl_rbar = damping.get("Cl,rbar", None)
                                        # Cm_rbar = damping.get("Cm,rbar", None)
                                        # Cn_rbar = damping.get("Cn,rbar", None)

                                        Cx = forces.get("Cx", None)
                                        Cy = forces.get("Cy", None)
                                        Cz = forces.get("Cz", None)
                                        Cl = forces.get("Cl", None)
                                        Cm = forces.get("Cm", None)
                                        Cn = forces.get("Cn", None)

                                        # Add data to the list
                                        # data.append([beta, alpha, Cx_pbar, Cy_pbar, Cl_pbar, Cm_pbar, Cn_pbar, Cx, Cy, Cz, Cl, Cm, Cn]) # use this for (alpha,beta)
                                        # data.append([p, alpha, Cx_pbar, Cy_pbar, Cz_pbar, Cl_pbar, Cm_pbar, Cn_pbar]) # use this for p(alpha,p)
                                        # data.append([q, alpha, Cx_qbar, Cy_qbar, Cz_qbar, Cl_qbar, Cm_qbar, Cn_qbar]) # use this for q(alpha,q)
                                        # data.append([r, alpha, Cx_rbar, Cy_rbar, Cz_rbar, Cl_rbar, Cm_rbar, Cn_rbar]) # use this for r(alpha,r)
                                        # data.append([elevator, beta, alpha, Cx, Cy, Cz, Cl, Cm, Cn]) # use this for DC(alpha,beta,elevator)
                                        data.append([aileron, beta, alpha, Cx, Cy, Cz, Cl, Cm, Cn]) # use this for DC(alpha,beta,aileron)
        # Convert to Pandas
        # df = pd.DataFrame(data, columns=["beta", "alpha", "Cx_pbar", "Cy_pbar", "Cl_pbar", "Cm_pbar", "Cn_pbar", "Cx", "Cy", "Cz", "Cl", "Cm", "Cn"]) # use this for (alpha,beta)
        # df = pd.DataFrame(data, columns=["pbar(rad/s)", "alpha(deg)", "Cx_pbar", "Cy_pbar", "Cz_pbar", "Cl_pbar", "Cm_pbar", "Cn_pbar"]) # use this for p(alpha,p)
        # df = pd.DataFrame(data, columns=["qbar(rad/s)", "alpha(deg)", "Cx_pbar", "Cy_qbar", "Cz_qbar", "Cl_qbar", "Cm_qbar", "Cn_qbar"]) # use this for q(alpha,q)
        # df = pd.DataFrame(data, columns=["rbar(rad/s)", "alpha(deg)", "Cx_rbar", "Cy_rbar", "Cz_rbar", "Cl_rbar", "Cm_rbar", "Cn_rbar"]) # use this for r(alpha,r)
        # df = pd.DataFrame(data, columns=["elevator(deg)", "beta(deg)", "alpha(deg)", "Cx", "Cy", "Cz", "Cl", "Cm", "Cn"]) # use this for DC(alpha,beta,elevator)
        df = pd.DataFrame(data, columns=["aileron(deg)", "beta(deg)", "alpha(deg)", "Cx", "Cy", "Cz", "Cl", "Cm", "Cn"]) # use this for DC(alpha,beta,aileron)
        # Export to csv file
        df.to_csv(filename, index=False)
        print(f"Data exported to {filename}")

    def export_stl(self, stl_filename):
        """Exports aircraft geometry as STL
        - stl_filename: Name of STL file to be saved (None if STL is not needed)
        - dxf_filename: Name of DXF file to be saved (None if DXF is not needeed)
        """
        if export_stl:
            print(f"Exporting aircraft geometry to {stl_filename}...")
            self.my_scene.export_stl(filename=stl_filename)

    def export_dxf_file(self):
        """Exports aircraft geometry as DXF
        """
        if export_dxf:
            print(f"Exporting aircraft geometry as a DXF")
            self.my_scene.export_dxf()

if __name__ == "__main__":
    # Define the input file for the scene
    input_file = "F16_input.json"

    # Call the MachUpX wrapper
    machupX_wrapper = MachUpXWrapper(input_file)

    # Display the aircraft geometry
    # machupX_wrapper.display_geometry()
    
    # Define flight parameters if not using export_csv
    alphas = np.linspace(20,5,2)         # deg
    beta = 0                            # deg
    p,q,r = 0,0,0                       # rad/s
    elevator, rudder, aileron = 0,0,0   # deg
    velocity = 222.5211                 # ft/s

    # Which forces to solve for
    calc_dimensional = True
    calc_non_dimensional = True
    # Loop through different angles of attack
    # for alpha in alphas:
    #     print(f"alpha: {alpha} degrees")
    #     forces = machupX_wrapper.solve_forces(velocity, alpha, beta, p, q, r, elevator, rudder, aileron, "F16", dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    # To export simulation data to a csv file. If using this, must define flight parameters in export_csv function.
    machupX_wrapper.export_csv("F16_simulation_results.csv", velocity, dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    # Trim aircraft in pitch
    # trimmed_alpha, trimmed_elevator = machupX_wrapper.pitch_trim("F16")

    # Get aerodynamic derivatives after aircraft is trimmed
    # machupX_wrapper.derivatives("F16")

    # If you want to solve forces again in trimmed state
    # machupX_wrapper.solve_forces(velocity, trimmed_alpha, beta, p, q, r, trimmed_elevator, rudder, aileron, "F16", dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    #### Export geometry
    # Decide which files to export
    export_stl = False
    export_dxf = False
    # Name of files to be exported
    stl_filename = "F16.stl"

    machupX_wrapper.export_stl(stl_filename)
    machupX_wrapper.export_dxf_file()