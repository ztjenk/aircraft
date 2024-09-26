"""
F-16 Aircraft Simulation using MachUpX
"""

# Import required modules
import machupX as MX
import numpy as np
import json
import os

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

        # Solve for forces and moments in the body frame (non-dimensional)
        forces = self.my_scene.solve_forces(dimensional=dimensional, non_dimensional=non_dimensional, verbose=False)

        print(json.dumps(forces[aircraft_name]["total"], indent=4))

        # Parse and return the relevant forces and moments
        return forces[aircraft_name]["total"]

    def export_stl(self, stl_filename=None):
        """Exports aircraft geometry as STL
        - stl_filename: Name of STL file to be saved (None if STL is not needed)
        - dxf_filename: Name of DXF file to be saved (None if DXF is not needeed)
        """
        print(f"Exporting aircraft geometry to {stl_filename}...")
        self.my_scene.export_stl()

        if stl_filename:
            default_filename = "scene.stl"  # This is the default name used by MachUpX
            if os.path.exists(default_filename):
                os.rename(default_filename, stl_filename)
                print(f"STL file renamed to {stl_filename}")

    def export_dxf(self, dxf_filename_prefix=None):
        """Exports aircraft geometry as DXF
        - dxf_filename: Name of DXF file to be saved (None if DXF is not needeed)
        """
        if dxf_filename_prefix:
            print(f"Exporting aircraft geometry to {dxf_filename_prefix}...")
            self.my_scene.export_aircraft_dxf(dxf_filename_prefix)

if __name__ == "__main__":
    # Define the input file for the scene
    input_file = "F16_input.json"

    # Call the MachUpX wrapper
    machupX_wrapper = MachUpXWrapper(input_file)

    # Display the aircraft geometry
    #machupX_wrapper.display_geometry()
    
    # Define flight parameters
    alphas = np.linspace(0,5,2)  # Range of angles of attack
    beta = 0  # Sideslip angle
    velocity = 222.5211  # Velocity (ft/s)
    p, q, r = 0, 0, 0  # Angular rates (roll, pitch, yaw)
    elevator, rudder, aileron = 0, 0, 0  # Control deflections

    # Which forces to solve for
    calc_dimensional = True
    calc_non_dimensional = False
    # Loop through different angles of attack
    for alpha in alphas:
        print(f"alpha: {alpha} degrees")
        forces = machupX_wrapper.solve_forces(velocity, alpha, beta, p, q, r, elevator, rudder, aileron, "F16", dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    # Trim aircraft in pitch
    trimmed_alpha, trimmed_elevator = machupX_wrapper.pitch_trim("F16")

    # Get aerodynamic derivatives after aircraft is trimmed
    machupX_wrapper.derivatives("F16")

    # If you want to solve forces again in trimmed state
    # machupX_wrapper.solve_forces(velocity, trimmed_alpha, beta, p, q, r, trimmed_elevator, rudder, aileron, "F16", dimensional=calc_dimensional, non_dimensional=calc_non_dimensional)

    #### Export geometry
    # Decide which files to export
    export_stl = True
    export_dxf = False
    # Name of files to be exported
    stl_filename = "F16.stl" if export_stl else None
    dxf_filename_prefix = "F16" if export_dxf else None

    machupX_wrapper.export_stl(stl_filename=stl_filename)
    machupX_wrapper.export_dxf(dxf_filename_prefix=dxf_filename_prefix)