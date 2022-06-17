import numpy as np

class Controller:
    """
    Parameters
    ----------
    Kp_rho : The linear velocity gain to translate the robot along a line
             towards the goal
    Kp_alpha : The angular velocity gain to rotate the robot towards the goal
    """

    def __init__(self, Kp_rho, Kp_alpha):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        v : Command linear velocity
        w : Command angular velocity
        """
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha*(theta_goal - theta)
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return v, w

