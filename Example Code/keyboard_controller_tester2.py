#  file keyboard_controller_tester.py
#  author Sebastien Sauter
#  description Complete keyboard controller for ROS2 node, used for testing. Must use space bar to stop movement.
#  version 1.0
#  date 2025-03-30


import msvcrt # Keyboard reading
import time # Used for sampling

class KeyboardController:
    def __init__(self):
        self.setpoint_translational_speed = 0.0 # [m/s]
        self.setpoint_angular_rate = 0.0 # [rad/s]
        self.current_translational_speed = 0.0
        self.current_angular_rate = 0.0

        # Dictionary, stores velocity data for each keyboard input
        self.key_library = {
            'w' : lambda : (self.setpoint_translational_speed, 0.0),
            's' : lambda : (self.setpoint_translational_speed, 0.0),
            'a' : lambda : (0.0, self.setpoint_angular_rate),
            'd' : lambda : (0.0, self.setpoint_angular_rate),
            ' ' : lambda : (0.0, 0.0),
            'x' : 'exit'
        }

        # OPTION 2 CODE
        # self.key_library = {
        #     'w' : self.setpoint_translational_speed,
        #     's' : -self.setpoint_translational_speed,
        #     'a' : self.setpoint_angular_rate,
        #     'd' : -self.setpoint_angular_rate,
        #     ' ' : 'exit'
        # }

        # OPTION 2 CODE
        # self.motion = {
        #     'w' : False,
        #     's' : False,
        #     'a' : False,
        #     'd' : False
        # }
        
        print("Keyboard Controller Ready")
    
    def get_key(self):
        if msvcrt.kbhit():  # Check if key was pressed
            return msvcrt.getch().decode('utf-8')  # Read the key and decode
        return None  # No key pressed

    # OPTION 2 CODE
    # def update_movement(self):
    #     self.current_translational_speed = 0.0
    #     self.current_angular_rate = 0.0

    #     if self.motion['w']: self.current_translational_speed = self.key_library['w']
    #     elif self.motion['s']: self.current_translational_speed = self.key_library['s']
    #     elif self.motion['a']: self.current_angular_rate = self.key_library['a']
    #     elif self.motion['d']: self.current_angular_rate = self.key_library['d']

    def run(self):
        try:
            time_last = time.monotonic()
            period = 1 # Sampling period

            while True:
                
                time_now = time.monotonic()
                
                # OPTION 2 CODE
                # key = self.get_key()
                # if key == ' ':
                #     break
                # elif key in self.motion:
                #     self.motion[key] = True
                # else:
                #     for k in self.motion:
                #         self.motion[k] = False
                # self.update_movement()

                key = self.get_key() 
                if key in self.key_library:
                    if key == 'w':
                        self.setpoint_translational_speed += 0.1
                        self.setpoint_angular_rate = 0
                    if key == 'a':
                        self.setpoint_angular_rate += 0.1
                        self.setpoint_translational_speed = 0
                    if key == 'd':
                        self.setpoint_angular_rate -= 0.1
                        self.setpoint_translational_speed = 0
                    if key == 's':
                        self.setpoint_translational_speed -= 0.1
                        self.setpoint_angular_rate = 0
                    if key == ' ':
                        self.setpoint_translational_speed = 0
                        self.setpoint_angular_rate = 0
                    movement = self.key_library[key] # Set the current movements based on the key
                    if movement == 'exit':
                        break # Break when 'x' is pressed, loop inf. if 'x' is not pressed
                    else:
                        self.current_translational_speed, self.current_angular_rate = movement()
                
                # Printing statements, in ROS2 this will be publish data
                if ((time_now - time_last) > period):
                    print(f"\rLinear: {self.current_translational_speed:.2f}, Angular: {self.current_angular_rate:.2f}")
                    time_last = time.monotonic()
                
        except Exception as e: 
            print(f"\nError: {str(e)}")

if __name__ == '__main__':
    controller = KeyboardController()
    controller.run()
