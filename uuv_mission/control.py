#class of PD controller 
class PDController:
    def __init__(self, K_P=0.15, K_D=0.6):
        """
        Initializes the PD controller with given proportional and derivative gains.
        """
        self.K_P = K_P  # Proportional gain
        self.K_D = K_D  # Derivative gain
        self.previous_error = 0  # To store the error from the previous timestep

    def compute_control_action(self, reference, output):
        """
        Computes the control action using PD control law.
        
        :param reference: The desired depth (reference depth) r[t].
        :param output: The current depth (output depth) y[t].
        :return: The computed control action u[t].
        """
        # Calculate the current error (e[t] = r[t] - y[t])
        error = reference - output
        
        # PD control law: u[t] = K_P * e[t] + K_D * (e[t] - e[t-1])
        control_action = self.K_P * error + self.K_D * (error - self.previous_error)
        
        # Update previous error for the next iteration
        self.previous_error = error
        
        return control_action
