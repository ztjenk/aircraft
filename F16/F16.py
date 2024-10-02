"""
F-16 Aircraft Simulation using MachUpX
"""

# Import required modules
import machupX as MX
import numpy as np
import json
import pandas as pd

class MachUpXWrapper:
    """Wrapper class for handling MachUpX operations."""
    
    def __init__(self, input_file):
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

        # print(json.dumps(forces[aircraft_name]["total"], indent=4))

        # Parse and return the relevant forces and moments
        return forces[aircraft_name]["total"]

    def export_to_excel(self, filename, velocity, dimensional=False, non_dimensional=True, aircraft_name="F16"):
        "Generates an Excel file with simulation results"
        # Define ranges for the variables to iterate over
        alphas = np.linspace(-10, 10, 5)  # Range of angles of attack
        betas = np.linspace(-6, 6, 7)
        ailerons = np.linspace(-10, 10, 1)
        elevators = np.linspace(-20, 20, 1)
        rudders = np.linspace(-30, 30, 1)
        p_vals = np.linspace(-90, 90, 1)
        q_vals = np.linspace(-30, 30, 1)
        r_vals = np.linspace(-30, 30, 1)

        # Create a list to store the data
        data = []

        # Iterate through all the combinations of values
        for alpha in alphas:
            for beta in betas:
                for aileron in ailerons:
                    for elevator in elevators:
                        for rudder in rudders:
                            for p in p_vals:
                                for q in q_vals:
                                    for r in r_vals:
                                        forces = self.solve_forces(velocity, alpha, beta, p, q, r, elevator, rudder, aileron, aircraft_name,
                                                                   dimensional=dimensional, non_dimensional=non_dimensional)
                                        CD = forces.get("CD", None)
                                        CS = forces.get("CS", None)
                                        CL = forces.get("CL", None)
                                        Cl = forces.get("Cl", None)
                                        Cm = forces.get("Cm", None)
                                        Cn = forces.get("Cn", None)

                                        # Add data to the list
                                        data.append([alpha, beta, aileron, elevator, rudder, p, q, r, CD, CS, CL, Cl, Cm, Cn])
        # Convert to Pandas
        df = pd.DataFrame(data, columns=["alpha", "beta", "aileron", "elevator", "rudder", "p", "q", "r", "CD", "CS", "CL", "Cl", "Cm", "Cn"])
        # Export to excel file
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
    
    # Define flight parameters
    velocity = 222.5211  # Velocity (ft/s)

    # Which forces to solve for
    calc_dimensional = False
    calc_non_dimensional = True
    # Loop through different angles of attack
    # for alpha in alphas:
    #     print(f"alpha: {alpha} degrees")
    #     forces = machupX_wrapper.solve_forces(velocity, alpha, beta, p, q, r, elevator, rudder, aileron, "F16", dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    machupX_wrapper.export_to_excel("F16_simulation_results.csv", velocity)

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