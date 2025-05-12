#!/usr/bin/python3

import subprocess
import sys
import time
from typing import Dict, List

class InteractiveSpawner:
    def __init__(self):
        self.base_cmd = "ros2 run ros2_conveyorbelt SpawnObject.py"
        self.package = "nikiro_gazebo"
        # Define available objects and their default heights
        self.available_objects = {
            '1': {'name': 'redcube', 'default_z': 1.0},
            '2': {'name': 'greencube', 'default_z': 1.0},
            '3': {'name': 'bluecube', 'default_z': 1.0},
            '4': {'name': 'aruco1', 'default_z': 1.0},           
            '5': {'name': 'aruco2', 'default_z': 1.0},
            '6': {'name': 'aruco3', 'default_z': 1.0}                        
        }
        
    def display_menu(self):
        """Display the menu of available objects."""
        print("\n=== Available Objects to Spawn ===")
        print("1. Red Cube")
        print("2. Green Cube")
        print("3. Blue Cube")
        print("4. Aruco1 Cube")
        print("5. Aruco2 Cube")
        print("6. Aruco3 Cube")                        
        print("q. Quit")
        print("===========================")

    def get_position_input(self) -> Dict[str, float]:
        """Get position coordinates from user."""
        while True:
            try:
                x = float(input("Enter X position (default -1.5): ") or "-1.5")
                y = float(input("Enter Y position (default -1.1): ") or "-1.1")
                z = float(input("Enter Z position (default 1.0): ") or "2.0")
                return {'x': x, 'y': y, 'z': z}
            except ValueError:
                print("Invalid input. Please enter numbers only.")

    def spawn_object(self, object_name: str, position: Dict[str, float]) -> None:
        """Spawn a single object with the specified parameters."""
        cmd = (f"{self.base_cmd} "
               f"--package {self.package} "
               f"--urdf {object_name}.urdf "
               f"--name {object_name} "
               f"--x {position['x']:.2f} "
               f"--y {position['y']:.2f} "
               f"--z {position['z']:.2f}")
        
        print(f"\nSpawning {object_name} at position: "
              f"x={position['x']:.2f}, y={position['y']:.2f}, z={position['z']:.2f}")
        
        try:
            process = subprocess.Popen(cmd, shell=True)
            process.wait()
            if process.returncode == 0:
                print(f"Successfully spawned {object_name}")
            else:
                print(f"Error spawning {object_name}")
        except Exception as e:
            print(f"Error executing command: {e}")

    def run(self):
        """Main interactive loop."""
        print("\nWelcome to the Interactive URDF Spawner!")
        print("This tool helps you spawn objects in Gazebo.")
        
        while True:
            self.display_menu()
            choice = input("\nEnter your choice (1-6 or q to quit): ").lower()
            
            if choice == 'q':
                print("Exiting spawner. Goodbye!")
                break
                
            if choice not in self.available_objects:
                print("Invalid choice. Please try again.")
                continue
                
            # Get object info
            object_info = self.available_objects[choice]
            
            # Get position
            print(f"\nEnter position for {object_info['name']}:")
            position = self.get_position_input()
            
            # Spawn the object
            self.spawn_object(object_info['name'], position)
            
            # Ask if user wants to spawn another object
            if input("\nWould you like to spawn another object? (y/n): ").lower() != 'y':
                print("Exiting spawner. Goodbye!")
                break

def main():
    try:
        spawner = InteractiveSpawner()
        spawner.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
